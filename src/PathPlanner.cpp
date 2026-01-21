#include "PathPlanner.h"
#include "Utils.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <queue>
#include <map>
#include <set>
#include <iostream>

struct Component {
    int id;
    std::vector<Point> points;
};

static inline double dist_sq(Point p1, Point p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
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
            std::queue<Point> q;
            q.push(p);
            grid[idx] = comp.id;
            comp.points.push_back(p);
            while (!q.empty()) {
                Point curr = q.front(); q.pop();
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
    for (const auto& comp : components) {
        for (const auto& p : comp.points) {
            int idx = p.y * width + p.x;
            if (!has_point[idx]) {
                has_point[idx] = 1;
                unvisited_list.push_back(p);
            }
        }
    }

    int cx = width / 2;
    int cy = height / 2;

    size_t remaining = unvisited_list.size();
    Point curr_p = { cx, cy };
    std::vector<Point> path;
    path.push_back(curr_p);
    std::vector<uint8_t> is_in_path(width * height, 0);
    is_in_path[curr_p.y * width + curr_p.x] = 1;

    auto mark_visited = [&](int x, int y) {
        int idx = y * width + x;
        if (has_point[idx] && !visited[idx]) {
            visited[idx] = 1;
            remaining--;
        }
        is_in_path[idx] = 1;
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

            // Multi-source BFS from all unvisited points to find the closest point in the path
            {
                std::queue<Point> q;
                std::vector<Point> origin_u(width * height, {-1, -1});
                for (const auto& p : unvisited_list) {
                    int idx = p.y * width + p.x;
                    if (!visited[idx]) {
                        q.push(p);
                        origin_u[idx] = p;
                    }
                }

                while (!q.empty()) {
                    Point u_curr = q.front(); q.pop();
                    int u_idx = u_curr.y * width + u_curr.x;
                    
                    if (is_in_path[u_idx]) {
                        best_v = u_curr;
                        best_u = origin_u[u_idx];
                        break;
                    }
                    
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (dx == 0 && dy == 0) continue;
                            int nx = u_curr.x + dx, ny = u_curr.y + dy;
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                int nidx = ny * width + nx;
                                if (origin_u[nidx].x == -1) {
                                    origin_u[nidx] = origin_u[u_idx];
                                    q.push({nx, ny});
                                }
                            }
                        }
                    }
                }
            }

            if (best_u.x != -1) {
                // Trace from current position to best_v through already traversed points
                if (curr_p.x != best_v.x || curr_p.y != best_v.y) {
                    std::queue<Point> q;
                    q.push(curr_p);
                    std::vector<int> parent(width * height, -1);
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
