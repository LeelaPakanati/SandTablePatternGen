#pragma once

#include "EdgeDetector.h"
#include <vector>
#include <string>

class GifGenerator {
public:
    static void generate_gif(const std::vector<Point>& points, int width, int height, const std::string& output_filename);
    static void generate_png(const std::vector<Point>& points, int width, int height, const std::string& output_filename);
private:
    static void draw_line(std::vector<uint8_t>& image, int width, int height, Point p1, Point p2, uint8_t r, uint8_t g, uint8_t b);
};
