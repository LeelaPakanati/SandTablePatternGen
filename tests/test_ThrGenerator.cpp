#include "doctest.h"
#include "../src/ThrGenerator.h"
#include <vector>
#include <cmath>

TEST_CASE("ThrGenerator::generate_thr") {
    int width = 100;
    int height = 100;
    // Center 50,50
    
    // Max radius will be distance to 100,50 -> 50.
    std::vector<Point> points = {
        {100, 50}, // Right edge -> Theta 0 (or near), Rho 1.0
        {50, 0}    // Top edge -> Y is 0 (inverted?) In image coords 0 is top.
                   // dx = 0, dy = -50. atan2(50, 0) = PI/2.
                   // Logic in code: dy = p.y - center.y = -50. atan2(-dy, dx) = atan2(50, 0) = PI/2.
    };

    auto thr = ThrGenerator::generate_thr(points, width, height);

    // Should include start/end zero points + 2 points
    REQUIRE(thr.size() == 4);

    // Start point (center)
    CHECK(thr[0].rho == 0.0);

    // First point {100, 50}
    // dx=50, dy=0. atan2(0, 50) = 0.
    CHECK(thr[1].theta == doctest::Approx(0.0));
    CHECK(thr[1].rho == doctest::Approx(1.0));

    // Second point {50, 0}
    // dx=0, dy=-50. atan2(dy, dx) = atan2(-50, 0) = -PI/2 = -1.5707...
    CHECK(thr[2].theta == doctest::Approx(-1.570796).epsilon(0.001));
    CHECK(thr[2].rho == doctest::Approx(1.0));

    // End point (center)
    CHECK(thr[3].rho == 0.0);
}
