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
};
