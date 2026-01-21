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
    const size_t MIN_COMPONENT_SIZE = 15;
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

    Point curr_p = { cx, cy };
    std::vector<Point> path;
    path.push_back(curr_p);
    std::vector<Point> stack;
    stack.push_back(curr_p);

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
        }
    };

    size_t remaining = unvisited_list.size();

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
            visited[curr_p.y * width + curr_p.x] = 1;
            stack.push_back(curr_p);
            remaining--;
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

        // Strategy 3: If there are segments close by, jump to those segments (15px radius)
        for (int r = 2; r <= 15; ++r) {
            for (int dy = -r; dy <= r; ++dy) {
                for (int dx = -r; dx <= r; ++dx) {
                    if (std::abs(dx) < r && std::abs(dy) < r) continue;
                    int nx = curr_p.x + dx, ny = curr_p.y + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int idx = ny * width + nx;
                        if (has_point[idx] && !visited[idx]) {
                            next_p = {nx, ny};
                            add_straight(curr_p, next_p);
                            goto found_point;
                        }
                    }
                }
            }
        }

        // Strategy 4: Otherwise, double back (backtrack) to reach parts we havent yet touched
        // Draw the backtrack path so the ball physically returns
        if (stack.size() > 1) {
            stack.pop_back();
            Point back_to = stack.back();
            add_straight(curr_p, back_to);
            path.push_back(back_to);
            curr_p = back_to;
            continue;
        }

        // Strategy 5: If stack is empty but points remain, jump directly to nearest unvisited
        {
            double min_d = std::numeric_limits<double>::max();
            int best_idx = -1;
            for(size_t i=0; i<unvisited_list.size(); ++i) {
                Point p = unvisited_list[i];
                if (!visited[p.y * width + p.x]) {
                    double d = dist_sq(curr_p, p);
                    if (d < min_d) { min_d = d; best_idx = i; }
                }
            }
            if (best_idx != -1) {
                Point target = unvisited_list[best_idx];
                // Use straight line - no perimeter arcs
                add_straight(curr_p, target);
                next_p = target;
                goto found_point;
            }
        }
        break;

    found_point:
        visited[next_p.y * width + next_p.x] = 1;
        remaining--;
        curr_p = next_p;
        stack.push_back(curr_p);
        path.push_back(curr_p);
    }

    return path;
}
