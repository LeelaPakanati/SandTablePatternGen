#include "PathPlanner.h"
#include "Utils.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <queue>
#include <map>
#include <set>
#include <iostream>
#include <mutex>

struct Component {
    int id;
    std::vector<Point> points;
    int min_x, min_y, max_x, max_y;
};

static inline double dist_sq(Point p1, Point p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return dx * dx + dy * dy;
}

static inline double dist_sq_to_bbox(Point p, int min_x, int min_y, int max_x, int max_y) {
    double dx = std::max(0, std::max(min_x - p.x, p.x - max_x));
    double dy = std::max(0, std::max(min_y - p.y, p.y - max_y));
    return dx * dx + dy * dy;
}

std::vector<Point> PathPlanner::plan_path(const std::vector<Point>& input_points, int width, int height) {
    if (input_points.empty()) return {};

    // 1. Identify Components (BFS) for noise filtering
    std::vector<int> grid(width * height, -1);
    for (const auto& p : input_points) {
        if (p.x >= 0 && p.x < width && p.y >= 0 && p.y < height)
            grid[p.y * width + p.x] = -2;
    }

    std::vector<Component> components;
    int comp_id = 0;
    for (const auto& p : input_points) {
        int idx = p.y * width + p.x;
        if (grid[idx] == -2) {
            Component comp;
            comp.id = comp_id++;
            comp.min_x = comp.max_x = p.x;
            comp.min_y = comp.max_y = p.y;
            std::queue<Point> q;
            q.push(p);
            grid[idx] = comp.id;
            comp.points.push_back(p);
            while (!q.empty()) {
                Point curr = q.front(); q.pop();
                comp.min_x = std::min(comp.min_x, curr.x);
                comp.max_x = std::max(comp.max_x, curr.x);
                comp.min_y = std::min(comp.min_y, curr.y);
                comp.max_y = std::max(comp.max_y, curr.y);
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = curr.x + dx, ny = curr.y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            int nidx = ny * width + nx;
                            if (grid[nidx] == -2) {
                                grid[nidx] = comp.id;
                                Point np = {nx, ny};
                                comp.points.push_back(np);
                                q.push(np);
                            }
                        }
                    }
                }
            }
            components.push_back(comp);
        }
    }

    // Filter small components (noise)
    const size_t MIN_COMPONENT_SIZE = 1;
    components.erase(
        std::remove_if(components.begin(), components.end(), 
            [](const Component& c) { return c.points.size() < MIN_COMPONENT_SIZE; }),
        components.end()
    );

    if (components.empty()) return {};

    // 2. Setup for traversal
    std::vector<uint8_t> has_point(width * height, 0);
    std::vector<uint8_t> visited(width * height, 0);
    std::vector<Point> unvisited_list;
    std::vector<int> point_to_comp(width * height, -1);
    std::vector<int> comp_remaining_counts(components.size(), 0);

    for (const auto& comp : components) {
        for (const auto& p : comp.points) {
            int idx = p.y * width + p.x;
            if (!has_point[idx]) {
                has_point[idx] = 1;
                unvisited_list.push_back(p);
                point_to_comp[idx] = comp.id;
                comp_remaining_counts[comp.id]++;
            }
        }
    }

    std::vector<int> active_comp_indices;
    for(size_t i=0; i<components.size(); ++i) {
        if (comp_remaining_counts[i] > 0) active_comp_indices.push_back(i);
    }

    int cx = width / 2;
    int cy = height / 2;

    size_t remaining = unvisited_list.size();
    Point curr_p = { cx, cy };
    std::vector<Point> path;
    path.push_back(curr_p);
    std::vector<uint8_t> is_in_path(width * height, 0);
    is_in_path[curr_p.y * width + curr_p.x] = 1;

    // Spatial Grid for path points to speed up "nearest" queries
    const int GRID_SIZE = 16;
    int grid_cols = (width + GRID_SIZE - 1) / GRID_SIZE;
    int grid_rows = (height + GRID_SIZE - 1) / GRID_SIZE;
    std::vector<std::vector<Point>> spatial_grid(grid_cols * grid_rows);
    spatial_grid[(curr_p.y / GRID_SIZE) * grid_cols + (curr_p.x / GRID_SIZE)].push_back(curr_p);

    auto mark_visited = [&](int x, int y) {
        int idx = y * width + x;
        if (has_point[idx] && !visited[idx]) {
            visited[idx] = 1;
            remaining--;
            int cid = point_to_comp[idx];
            if (cid != -1) comp_remaining_counts[cid]--;
        }
        if (!is_in_path[idx]) {
            is_in_path[idx] = 1;
            spatial_grid[(y / GRID_SIZE) * grid_cols + (x / GRID_SIZE)].push_back({x, y});
        }
    };

    auto add_straight = [&](Point start, Point end) {
        // Bresenham line from start to end, excluding both start and endpoint (caller adds endpoint)
        if (start.x == end.x && start.y == end.y) return;
        int x0 = start.x, y0 = start.y, x1 = end.x, y1 = end.y;
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;
        while (true) {
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
            if (x0 == x1 && y0 == y1) break;  // Stop before endpoint
            path.push_back({x0, y0});
            mark_visited(x0, y0);
        }
    };

    // Strategy 1: Start in the center, go to the nearest point on an edge
    {
        double min_d = std::numeric_limits<double>::max();
        int best_idx = -1;
        for(size_t i=0; i<unvisited_list.size(); ++i) {
            double d = dist_sq(curr_p, unvisited_list[i]);
            if (d < min_d) { min_d = d; best_idx = i; }
        }
        if (best_idx != -1) {
            Point target = unvisited_list[best_idx];
            add_straight(curr_p, target);
            curr_p = target;
            path.push_back(curr_p);  // Add target to path
            mark_visited(curr_p.x, curr_p.y);
        }
    }

    while (remaining > 0) {
        Point next_p = {-1, -1};
        
        // Strategy 2: Follow along any lines (unvisited neighbors)
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                int nx = curr_p.x + dx, ny = curr_p.y + dy;
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    int idx = ny * width + nx;
                    if (has_point[idx] && !visited[idx]) {
                        next_p = {nx, ny};
                        goto found_point;
                    }
                }
            }
        }

        // Strategy 3: Global search for the best jumping-off point on the existing path
        {
            Point best_v = {-1, -1};
            Point best_u = {-1, -1};
            double global_min_dist_sq = std::numeric_limits<double>::max();
            std::mutex min_mutex;

            // Parallelize the search across active components.
            Utils::parallel_for(0, (int)active_comp_indices.size(), [&](int i) {
                int c_idx = active_comp_indices[i];
                if (comp_remaining_counts[c_idx] == 0) return;

                const auto& comp = components[c_idx];
                
                // Sample points from the component to find the nearest path point
                size_t step = 1;
                if (comp.points.size() > 100) step = comp.points.size() / 10;
                else if (comp.points.size() > 20) step = 4;

                for (size_t i = 0; i < comp.points.size(); i += step) {
                    const auto& u = comp.points[i];
                    if (visited[u.y * width + u.x]) continue;

                    int ux = u.x / GRID_SIZE;
                    int uy = u.y / GRID_SIZE;
                    double local_min_dist_sq = std::numeric_limits<double>::max();
                    Point local_best_v = {-1, -1};

                    bool found_in_radius = false;
                    for (int r = 0; r < std::max(grid_cols, grid_rows); ++r) {
                        if (found_in_radius && r > (int)(std::sqrt(local_min_dist_sq)/GRID_SIZE) + 1) break;

                        // Prune search radius based on global best
                        {
                            std::lock_guard<std::mutex> lock(min_mutex);
                            if (found_in_radius && local_min_dist_sq >= global_min_dist_sq) break;
                            if (!found_in_radius && (double)r * GRID_SIZE * r * GRID_SIZE > global_min_dist_sq) break;
                        }

                        for (int dy = -r; dy <= r; ++dy) {
                            for (int dx = -r; dx <= r; ++dx) {
                                if (std::abs(dx) < r && std::abs(dy) < r) continue;
                                int gx = ux + dx, gy = uy + dy;
                                if (gx >= 0 && gx < grid_cols && gy >= 0 && gy < grid_rows) {
                                    for (const auto& v : spatial_grid[gy * grid_cols + gx]) {
                                        double d2 = (double)(u.x-v.x)*(u.x-v.x) + (double)(u.y-v.y)*(u.y-v.y);
                                        if (d2 < local_min_dist_sq) {
                                            local_min_dist_sq = d2;
                                            local_best_v = v;
                                            found_in_radius = true;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    if (local_best_v.x != -1) {
                        std::lock_guard<std::mutex> lock(min_mutex);
                        if (local_min_dist_sq < global_min_dist_sq) {
                            global_min_dist_sq = local_min_dist_sq;
                            best_u = u;
                            best_v = local_best_v;
                        }
                    }
                }
            });

            if (best_u.x != -1) {
                // Periodically clean up active_comp_indices to keep search fast
                if (remaining % 100 == 0) { // Using remaining as a proxy for progress
                    active_comp_indices.erase(
                        std::remove_if(active_comp_indices.begin(), active_comp_indices.end(),
                            [&](int cid) { return comp_remaining_counts[cid] == 0; }),
                        active_comp_indices.end()
                    );
                }

                // Trace from current position to best_v through already traversed points
                if (curr_p.x != best_v.x || curr_p.y != best_v.y) {
                    std::queue<Point> q;
                    q.push(curr_p);
                    
                    static std::vector<int> parent;
                    if (parent.size() != (size_t)width * height) parent.assign(width * height, -1);
                    else std::fill(parent.begin(), parent.end(), -1);
                    
                    parent[curr_p.y * width + curr_p.x] = -2;
                    bool found = false;
                    while(!q.empty()) {
                        Point c = q.front(); q.pop();
                        if (c.x == best_v.x && c.y == best_v.y) { found = true; break; }
                        for (int dy = -1; dy <= 1; ++dy) {
                            for (int dx = -1; dx <= 1; ++dx) {
                                if (dx == 0 && dy == 0) continue;
                                int nx = c.x + dx, ny = c.y + dy;
                                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                    int nidx = ny * width + nx;
                                    if (is_in_path[nidx] && parent[nidx] == -1) {
                                        parent[nidx] = c.y * width + c.x;
                                        q.push({nx, ny});
                                    }
                                }
                            }
                        }
                    }
                    if (found) {
                        std::vector<Point> trace;
                        int cur = best_v.y * width + best_v.x;
                        while(cur != -2) {
                            trace.push_back({cur % width, cur / width});
                            cur = parent[cur];
                        }
                        std::reverse(trace.begin(), trace.end());
                        for(size_t i=1; i<trace.size(); ++i) {
                            path.push_back(trace[i]);
                            mark_visited(trace[i].x, trace[i].y);
                        }
                        curr_p = best_v;
                    }
                }

                // Jump from the best jumping-off point to the target segment
                add_straight(curr_p, best_u);
                next_p = best_u;
                goto found_point;
            }
        }
        break;

    found_point:
        mark_visited(next_p.x, next_p.y);
        curr_p = next_p;
        path.push_back(curr_p);
    }

    return path;
}
