#pragma once

#include <vector>
#include <string>
#include <cstdint>

struct Point {
    int x;
    int y;
    
    // For testing comparison
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

class EdgeDetector {
public:
    // Main entry point for files
    static std::vector<Point> detect_edges(const std::string& image_path, int low_threshold, int high_threshold, int blur_kernel_size = 5);
    
    // Core logic for memory buffers (useful for testing and specialized loaders)
    // Assumes grayscale or RGB data. If channels >= 3, converts to grayscale.
    static std::vector<Point> detect_edges_from_memory(const unsigned char* data, int width, int height, int channels, int low_threshold, int high_threshold, int blur_kernel_size = 5);

    // Native bilinear resizer
    static std::vector<uint8_t> resize(const unsigned char* data, int width, int height, int channels, int new_width, int new_height);

private:
    static std::vector<uint8_t> grayscale(const unsigned char* data, int width, int height, int channels);
    static std::vector<uint8_t> gaussian_blur(const std::vector<uint8_t>& image, int width, int height, int kernel_size);
    static void sobel(const std::vector<uint8_t>& image, int width, int height, std::vector<float>& magnitude, std::vector<float>& angle);
    static std::vector<uint8_t> non_max_suppression(const std::vector<float>& magnitude, const std::vector<float>& angle, int width, int height);
    static std::vector<Point> hysteresis(const std::vector<uint8_t>& image, int width, int height, int low, int high);
    static std::vector<Point> bridge_gaps(const std::vector<Point>& edges, int width, int height, int max_gap = 10);
};