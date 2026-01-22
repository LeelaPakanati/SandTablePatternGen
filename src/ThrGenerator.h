#pragma once

#include "EdgeDetector.h"
#include <vector>
#include <string>

struct ThrPoint {
    double theta;
    double rho;
};

class ThrGenerator {
public:
    static std::vector<ThrPoint> generate_thr(const std::vector<Point>& points, int width, int height);
    static std::string to_string(const std::vector<ThrPoint>& thr_points);

    // Parse .thr file content back to polar points
    static std::vector<ThrPoint> parse(const std::string& thr_content);

    // Convert polar points back to Cartesian path
    static std::vector<Point> to_cartesian(const std::vector<ThrPoint>& thr_points, int width, int height);
};
