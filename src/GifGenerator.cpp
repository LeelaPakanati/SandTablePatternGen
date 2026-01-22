#include "GifGenerator.h"
#include "gif.h"
#include "stb_image_write.h"
#include "Utils.h"
#include <cmath>
#include <algorithm>
#include <iostream>

void GifGenerator::generate_gif(const std::vector<Point>& points, int width, int height, const std::string& output_filename) {
    if (points.empty()) return;

    // Dynamic resolution, capped at 1024 for performance/size
    int max_dim = std::max(width, height);
    int gif_size = std::min(max_dim, 1024); 
    
    double scale = (double)gif_size / max_dim;
    
    // Create writer
    GifWriter g;
    GifBegin(&g, output_filename.c_str(), gif_size, gif_size, 5); // 5ms delay

    std::vector<uint8_t> frame(gif_size * gif_size * 4);
    
    // Initialize background to Dark Sand Color (Parallel)
    Utils::parallel_for(0, gif_size * gif_size, [&](int i) {
        frame[i * 4] = 50;      // R
        frame[i * 4 + 1] = 40;  // G
        frame[i * 4 + 2] = 30;  // B
        frame[i * 4 + 3] = 255; // A
    });

    int total_points = points.size();
    int points_per_frame = std::max(1, total_points / 100); // 100 frames total
    
    Point prev = {
        static_cast<int>(points[0].x * scale), 
        static_cast<int>(points[0].y * scale)
    };

    int offset_x = (gif_size - (int)(width * scale)) / 2;
    int offset_y = (gif_size - (int)(height * scale)) / 2;
    
    prev.x += offset_x;
    prev.y += offset_y;

    // Track color (Light Sand)
    uint8_t tr = 230, tg = 220, tb = 200;

    for (int i = 1; i < total_points; ++i) {
        Point curr = {
            static_cast<int>(points[i].x * scale) + offset_x, 
            static_cast<int>(points[i].y * scale) + offset_y
        };

        draw_line(frame, gif_size, gif_size, prev, curr, tr, tg, tb);
        prev = curr;

        if (i % points_per_frame == 0 || i == total_points - 1) {
            // Create a temp buffer for this frame to draw the ball
            std::vector<uint8_t> display_frame = frame;
            
            // Draw Ball (Steel/Silver circle) at 'curr'
            // Scaled radius, min 3
            int br = std::max(3, static_cast<int>(4.0 * scale));
            
            // Parallelize ball drawing (simple for small br, but good for consistency)
            for (int dy = -br; dy <= br; ++dy) {
                for (int dx = -br; dx <= br; ++dx) {
                    if (dx*dx + dy*dy <= br*br) {
                        int bx = curr.x + dx;
                        int by = curr.y + dy;
                        if (bx >= 0 && bx < gif_size && by >= 0 && by < gif_size) {
                            int idx = (by * gif_size + bx) * 4;
                            display_frame[idx] = 192;   // R
                            display_frame[idx+1] = 192; // G
                            display_frame[idx+2] = 192; // B
                        }
                    }
                }
            }

            GifWriteFrame(&g, display_frame.data(), gif_size, gif_size, 5); 
        }
    }

    GifEnd(&g);
}

void GifGenerator::draw_line(std::vector<uint8_t>& image, int width, int height, Point p0, Point p1, uint8_t r, uint8_t g, uint8_t b) {
    int x0 = p0.x;
    int y0 = p0.y;
    int x1 = p1.x;
    int y1 = p1.y;

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true) {
        if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height) {
            int idx = (y0 * width + x0) * 4;
            image[idx] = r; image[idx+1] = g; image[idx+2] = b; image[idx+3] = 255;
        }
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void GifGenerator::generate_png(const std::vector<Point>& points, int width, int height, const std::string& output_filename) {
    if (points.empty()) return;

    // Match SisyphusTable viewer: 800x800 canvas, center at 400,400, radius 380
    const int png_size = 800;
    const int center = 400;
    const int radius = 380;

    // Source image center and max radius (inscribed circle)
    double src_cx = width / 2.0;
    double src_cy = height / 2.0;
    double src_max_r = std::min(width, height) / 2.0;

    // Initialize background to Transparent
    std::vector<uint8_t> image(png_size * png_size * 4, 0);

    // Convert source point to canvas coordinates (matching viewer's mapping)
    auto to_canvas = [&](const Point& p) -> Point {
        // Normalize to [-1, 1] based on inscribed circle
        double norm_x = (p.x - src_cx) / src_max_r;
        double norm_y = (p.y - src_cy) / src_max_r;
        // Map to canvas: center + normalized * radius
        return {
            static_cast<int>(center + norm_x * radius),
            static_cast<int>(center + norm_y * radius)
        };
    };

    Point prev = to_canvas(points[0]);

    // White lines
    uint8_t tr = 255, tg = 255, tb = 255;

    for (size_t i = 1; i < points.size(); ++i) {
        Point curr = to_canvas(points[i]);
        draw_line(image, png_size, png_size, prev, curr, tr, tg, tb);
        prev = curr;
    }

    stbi_write_png(output_filename.c_str(), png_size, png_size, 4, image.data(), png_size * 4);
}

