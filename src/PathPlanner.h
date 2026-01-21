#pragma once

#include "EdgeDetector.h"
#include <vector>
#include <set>

class PathPlanner {
public:
    static std::vector<Point> plan_path(const std::vector<Point>& points, int width, int height);

private:
    static void find_components(
        const std::vector<Point>& points, 
        int width, int height,
        std::vector<std::vector<Point>>& components
    );

    // DFS to traverse the graph of pixels
    static void dfs(
        int current_idx, 
        const std::vector<Point>& all_points, 
        const std::vector<std::vector<int>>& adjacency, 
        std::vector<bool>& visited, 
        std::vector<Point>& path
    );
};