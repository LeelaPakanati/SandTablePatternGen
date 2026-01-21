#include "doctest.h"
#include "../src/EdgeDetector.h"
#include <vector>
#include <numeric>

TEST_CASE("EdgeDetector::detect_edges_from_memory") {
    // Create a 30x30 image (larger to accommodate 5px border mask)
    int width = 30;
    int height = 30;
    int channels = 1;
    std::vector<unsigned char> data(width * height, 0);

    // Draw 10x10 square at (10,10)
    for (int y = 10; y < 20; ++y) {
        for (int x = 10; x < 20; ++x) {
            data[y * width + x] = 255;
        }
    }

    // Border masking check: ensure borders are 0 first (they are init to 0)
    // Actually, let's put white pixels at the very border to test if they get ignored
    data[0] = 255;
    data[width - 1] = 255;
    
    // Using low thresholds to ensure detection of the square
    auto edges = EdgeDetector::detect_edges_from_memory(data.data(), width, height, channels, 50, 150, 3);

    // Should find points. 
    // The square is solid, so edges should be at the transition 2-3 and 6-7.
    // Given 3x3 Sobel and blur, exact pixel locations might shift slightly, 
    // but we expect *some* edges.
    
    CHECK(edges.size() > 0);

    // Check that NO edges are at the frame boundary (0 or width-1/height-1)
    for (const auto& p : edges) {
        CHECK(p.x > 0);
        CHECK(p.x < width - 1);
        CHECK(p.y > 0);
        CHECK(p.y < height - 1);
    }
}
