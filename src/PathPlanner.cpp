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

const double PI = 3.14159265358979323846;

struct Component {
    int id;
    std::vector<Point> points;
};

// Squared distance
inline double dist_sq(Point p1, Point p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return dx*dx + dy*dy;
}

// Polar conversion helper
inline void to_polar(double x, double y, double cx, double cy, double max_r, double& theta, double& rho) {
    double dx = x - cx;
    double dy = y - cy; // Y inverted? Standard math: y up. Image: y down. 
    // Sisyphus logic usually expects standard cartesian or inverted Y. 
    // Let's stick to standard atan2(-dy, dx) as per ThrGenerator to match.
    theta = std::atan2(-dy, dx);
    rho = std::sqrt(dx*dx + dy*dy) / max_r;
}

inline Point to_cartesian(double theta, double rho, double cx, double cy, double max_r) {
    // Inverse of above
    double dx = rho * max_r * std::cos(theta);
    double dy = -rho * max_r * std::sin(theta);
    return { static_cast<int>(cx + dx), static_cast<int>(cy + dy) };
}

std::vector<Point> PathPlanner::plan_path(const std::vector<Point>& input_points, int width, int height) {
    if (input_points.empty()) return {};

    // 1. Identify Components (BFS)
    // Grid for fast lookup: -1 = empty, -2 = point (unvisited), >=0 = component ID
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
                Point curr = q.front();
                q.pop();

                // 8-way connectivity
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = curr.x + dx;
                        int ny = curr.y + dy;
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
    // Sisyphus resolution is limited; tiny specs (< 15 px) are just noise/jitter.
    const size_t MIN_COMPONENT_SIZE = 15;
    components.erase(
        std::remove_if(components.begin(), components.end(), 
            [](const Component& c) { return c.points.size() < MIN_COMPONENT_SIZE; }),
        components.end()
    );

    // 2. Connect Components (MST)
    // If only 1 component, just traverse it.
    // If multiple, we need to add "bridge" points.
    
    // We need a graph of ALL points now, including original and bridges.
    // Let's use an adjacency list for the final traversal.
    // Map point index to adjacency list.
    // Since points are unique in grid, we can map (x,y) to an index 0..N-1.
    
    std::vector<Point> all_points;
    std::vector<int> point_grid(width * height, -1); // Map (x,y) -> index in all_points

    // Add original points
    for (const auto& comp : components) {
        for (const auto& p : comp.points) {
            if (point_grid[p.y * width + p.x] == -1) {
                point_grid[p.y * width + p.x] = all_points.size();
                all_points.push_back(p);
            }
        }
    }

    // Adjacency list: index -> list of indices
    std::vector<std::vector<int>> adj(all_points.size());

    if (all_points.empty()) return {};

    // Add internal edges (original neighbor relationships)
    for (size_t i = 0; i < all_points.size(); ++i) {
        Point p = all_points[i];
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                int nx = p.x + dx;
                int ny = p.y + dy;
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    int neighbor_idx = point_grid[ny * width + nx];
                    if (neighbor_idx != -1) {
                        adj[i].push_back(neighbor_idx);
                    }
                }
            }
        }
    }

    // If we have > 1 component, find bridges
    if (components.size() > 1) {
        // Simple Greedy strategy: Start with component closest to center.
        // Connect to nearest unvisited component.
        // Repeat.
        // This is Prim's algorithm for MST effectively.

        struct CompDist {
            int from_comp;
            int to_comp;
            double dist;
            Point p_from;
            Point p_to;
            bool operator>(const CompDist& other) const { return dist > other.dist; }
        };

        std::vector<bool> comp_connected(components.size(), false);
        
        // Find center-most component
        double cx = width / 2.0;
        double cy = height / 2.0;
        int start_comp = 0;
        double min_center_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < components.size(); ++i) {
            for (const auto& p : components[i].points) {
                double d = dist_sq(p, {static_cast<int>(cx), static_cast<int>(cy)});
                if (d < min_center_dist) {
                    min_center_dist = d;
                    start_comp = i;
                }
            }
        }

        comp_connected[start_comp] = true;
        int connected_count = 1;

        while (connected_count < (int)components.size()) {
            // Find shortest link from ANY connected component to ANY disconnected component
            // Optimization: This can be N^2. With many components, slow.
            // But typical image has < 100 components.
            
            double best_dist = std::numeric_limits<double>::max();
            int best_from = -1;
            int best_to = -1;
            Point best_p1 = {0,0};
            Point best_p2 = {0,0};

            std::mutex mtx;

            Utils::parallel_for(size_t(0), components.size(), [&](size_t i) {
                if (!comp_connected[i]) return;

                // Optimization: Don't check every single point.
                // Sample points to find rough distance.
                // For large components, checking every 20th point reduces work by 400x.
                size_t step_i = std::max(size_t(1), components[i].points.size() / 20);

                double local_best_dist = std::numeric_limits<double>::max();
                int local_best_to = -1;
                Point local_best_p1 = {0,0};
                Point local_best_p2 = {0,0};

                for (size_t j = 0; j < components.size(); ++j) {
                    if (comp_connected[j]) continue;

                    size_t step_j = std::max(size_t(1), components[j].points.size() / 20);

                    // Find distance between comp i and comp j
                    // Strided check
                    for (size_t idx1 = 0; idx1 < components[i].points.size(); idx1 += step_i) {
                        const auto& p1 = components[i].points[idx1];
                        for (size_t idx2 = 0; idx2 < components[j].points.size(); idx2 += step_j) {
                            const auto& p2 = components[j].points[idx2];
                            double d = dist_sq(p1, p2);
                            if (d < local_best_dist) {
                                local_best_dist = d;
                                local_best_to = j;
                                local_best_p1 = p1;
                                local_best_p2 = p2;
                            }
                        }
                    }
                }

                if (local_best_to != -1) {
                    std::lock_guard<std::mutex> lock(mtx);
                    if (local_best_dist < best_dist) {
                        best_dist = local_best_dist;
                        best_from = i;
                        best_to = local_best_to;
                        best_p1 = local_best_p1;
                        best_p2 = local_best_p2;
                    }
                }
            });

            if (best_to == -1) break; // Should not happen

            // Refine: We found the best pair of *sampled* points.
            // Now do a local search around those points in the original arrays to find the *true* closest pixel
            // within that neighborhood.
            // Actually, for sand table purposes, exact pixel-perfect shortest bridge isn't strictly required,
            // but let's do a quick full check if the components are small enough, or just accept the sampled one.
            // Given the visual nature, the sampled one is likely "good enough" or close enough. 
            // If we want to be better, we could iterate all points of component J against just 'best_p1' (constant time vs size of J).
            
            // Connect best_from -> best_to via best_p1 -> best_p2
            comp_connected[best_to] = true;
            connected_count++;

            // GENERATE BRIDGE
            // If distance is large, use perimeter
            double raw_dist = std::sqrt(best_dist);
            bool use_perimeter = raw_dist > (std::min(width, height) * 0.10); // 10% threshold

            std::vector<Point> bridge_points;
            if (use_perimeter) {
                double max_r = std::min(width, height) / 2.0;
                double t1, r1, t2, r2;
                to_polar(best_p1.x, best_p1.y, cx, cy, max_r, t1, r1);
                to_polar(best_p2.x, best_p2.y, cx, cy, max_r, t2, r2);

                // Move to perimeter
                // Distance to perimeter is max_r * (1.0 - r1)
                // Steps should be at least that many pixels
                int steps1 = static_cast<int>(std::ceil(max_r * std::abs(1.0 - r1))) * 2; // *2 for safety
                if (steps1 < 1) steps1 = 1;

                for (int s = 1; s <= steps1; ++s) {
                    double r = r1 + (1.0 - r1) * s / steps1;
                    bridge_points.push_back(to_cartesian(t1, r, cx, cy, max_r));
                }

                // Move along perimeter (shortest arc)
                double dt = t2 - t1;
                while (dt > PI) dt -= 2*PI;
                while (dt < -PI) dt += 2*PI;
                
                // Arc length = r * theta
                int arc_steps = static_cast<int>(std::ceil(std::abs(dt) * max_r)) * 2;
                if (arc_steps < 1) arc_steps = 1;
                
                for (int s = 1; s <= arc_steps; ++s) {
                    double t = t1 + dt * s / arc_steps;
                    bridge_points.push_back(to_cartesian(t, 1.0, cx, cy, max_r));
                }

                // Move from perimeter to p2
                int steps2 = static_cast<int>(std::ceil(max_r * std::abs(r2 - 1.0))) * 2;
                if (steps2 < 1) steps2 = 1;

                for (int s = 1; s <= steps2; ++s) {
                    double r = 1.0 + (r2 - 1.0) * s / steps2;
                    bridge_points.push_back(to_cartesian(t2, r, cx, cy, max_r));
                }

            } else {
                // Bresenham line
                int x0 = best_p1.x, y0 = best_p1.y;
                int x1 = best_p2.x, y1 = best_p2.y;
                int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
                int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
                int err = dx + dy, e2;

                while (true) {
                    if (x0 == x1 && y0 == y1) break;
                    e2 = 2 * err;
                    if (e2 >= dy) { err += dy; x0 += sx; }
                    if (e2 <= dx) { err += dx; y0 += sy; }
                    if (x0 != x1 || y0 != y1) bridge_points.push_back({x0, y0});
                }
            }

            // Add bridge points to all_points and graph
            int last_idx = point_grid[best_p1.y * width + best_p1.x];
            
            for (const auto& bp : bridge_points) {
                // Ensure unique
                int b_idx = -1;
                if (bp.x >= 0 && bp.x < width && bp.y >= 0 && bp.y < height) {
                    b_idx = point_grid[bp.y * width + bp.x];
                }
                
                if (b_idx == -1) {
                    b_idx = all_points.size();
                    all_points.push_back(bp);
                    // resize adj
                    adj.resize(all_points.size());
                    if (bp.x >= 0 && bp.x < width && bp.y >= 0 && bp.y < height) {
                        point_grid[bp.y * width + bp.x] = b_idx;
                    }
                }

                // Connect last_idx <-> b_idx
                adj[last_idx].push_back(b_idx);
                adj[b_idx].push_back(last_idx);
                last_idx = b_idx;
            }
            // Connect final bridge point to best_p2
            int end_idx = point_grid[best_p2.y * width + best_p2.x];
            adj[last_idx].push_back(end_idx);
            adj[end_idx].push_back(last_idx);
        }
    }

    // 3. Generate Traversal (DFS)
    // Start at center-most point
    double cx = width / 2.0;
    double cy = height / 2.0;
    int start_node = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < all_points.size(); ++i) {
        double d = dist_sq(all_points[i], {static_cast<int>(cx), static_cast<int>(cy)});
        if (d < min_dist) {
            min_dist = d;
            start_node = i;
        }
    }

    std::vector<Point> final_path;
    std::vector<bool> visited(all_points.size(), false);
    
    // Iterative DFS to avoid stack overflow
    std::vector<int> stack;
    stack.push_back(start_node);
    
    // For "doubling back", we just traverse.
    // Standard DFS on a graph visits nodes. 
    // To ensure we cover all edges (drawing lines), we need to be careful.
    // If we treat pixels as nodes, visiting all nodes is enough to "draw" the image.
    // Backtracking (popping stack) implicitly traverses back up the tree.
    // We should record the point when we push AND when we pop (if we need to draw the return stroke).
    // Sisyphus: Yes, we must draw the return stroke to move the ball back.
    
    // We need to keep track of neighbor iteration index for each node on stack
    std::vector<int> neighbor_ptr(all_points.size(), 0);
    
    // Explicit stack for path reconstruction
    struct StackFrame {
        int u;
        int p; // parent index
    };
    std::vector<StackFrame> dfs_stack;
    dfs_stack.push_back({start_node, -1});
    visited[start_node] = true;
    final_path.push_back(all_points[start_node]);

    while (!dfs_stack.empty()) {
        int u = dfs_stack.back().u;
        
        bool found = false;
        // Look for unvisited neighbor
        const auto& neighbors = adj[u];
        for (int& i = neighbor_ptr[u]; i < (int)neighbors.size(); ++i) {
            int v = neighbors[i];
            if (!visited[v]) {
                visited[v] = true;
                dfs_stack.push_back({v, u});
                final_path.push_back(all_points[v]);
                found = true;
                i++; // advance for next time
                break;
            }
        }

        if (!found) {
            // Backtrack
            dfs_stack.pop_back();
            if (!dfs_stack.empty()) {
                // Add the parent to path to draw the back-travel
                int parent = dfs_stack.back().u;
                final_path.push_back(all_points[parent]);
            }
        }
    }

    return final_path;
}