#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

#include "EdgeDetector.h"
#include "PathPlanner.h"
#include "ThrGenerator.h"
#include "GifGenerator.h"
#include "stb_image.h"
#include "stb_image_write.h"

// Helper to parse THR file
std::vector<Point> parse_thr(const std::string& filename, int width, int height) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open THR file: " << filename << std::endl;
        return {};
    }

    std::vector<Point> points;
    std::string line;
    double max_radius = std::min(width, height) / 2.0;
    double cx = width / 2.0;
    double cy = height / 2.0;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        double theta, rho;
        if (ss >> theta >> rho) {
            // Convert back to Cartesian for visualization
            // x = cx + rho * max_r * cos(theta)
            // y = cy - rho * max_r * sin(theta) (inverted Y)
            int x = static_cast<int>(cx + rho * max_radius * std::cos(theta));
            int y = static_cast<int>(cy - rho * max_radius * std::sin(theta));
            points.push_back({x, y});
        }
    }
    return points;
}

// Helper to draw points to PNG (solid white, transparent background)
void write_points_png(const std::vector<Point>& points, int width, int height, const std::string& out_path) {
    std::vector<uint8_t> image(width * height * 4, 0); // RGBA, transparent background

    for (const auto& p : points) {
        if (p.x >= 0 && p.x < width && p.y >= 0 && p.y < height) {
            int idx = (p.y * width + p.x) * 4;
            image[idx] = 255;   // R
            image[idx+1] = 255; // G
            image[idx+2] = 255; // B
            image[idx+3] = 255; // A
        }
    }

    stbi_write_png(out_path.c_str(), width, height, 4, image.data(), width * 4);
}

void print_usage(const char* name) {
    std::cout << "Usage:\n";
    std::cout << "  Generate THR from Image:\n";
    std::cout << "    " << name << " gen <input_image> <output_thr> [options]\n";
    std::cout << "    Options:\n";
    std::cout << "      --low <val>    Low threshold (default 50)\n";
    std::cout << "      --high <val>   High threshold (default 150)\n";
    std::cout << "      --blur <val>   Blur kernel size (default 5)\n";
    std::cout << "      --gif <file>   Generate visualization GIF\n";
    std::cout << "      --png <file>   Generate static path image (white on transparent)\n";
    std::cout << "      --edges <file> Generate static edge image (white on transparent)\n\n";
    
    std::cout << "  Visualize THR file:\n";
    std::cout << "    " << name << " vis <input_thr> [options]\n";
    std::cout << "    Options:\n";
    std::cout << "      --gif <file>   Output GIF filename\n";
    std::cout << "      --png <file>   Output PNG filename (white on transparent)\n";
    std::cout << "      --edges <file> Output PNG filename (edge points)\n";
    std::cout << "      --size <int>   Resolution (default 1024)\n";
}

int main(int argc, char** argv) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    std::string mode = argv[1];
    std::string input_file = argv[2];

    if (mode == "gen") {
        if (argc < 4) {
            std::cerr << "Error: Output THR filename required.\n";
            return 1;
        }
        std::string output_thr = argv[3];
        int low = 50, high = 150, blur = 5;
        std::string gif_out, png_out, edges_out;

        for (int i = 4; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--low" && i+1 < argc) low = std::stoi(argv[++i]);
            else if (arg == "--high" && i+1 < argc) high = std::stoi(argv[++i]);
            else if (arg == "--blur" && i+1 < argc) blur = std::stoi(argv[++i]);
            else if (arg == "--gif" && i+1 < argc) gif_out = argv[++i];
            else if (arg == "--png" && i+1 < argc) png_out = argv[++i];
            else if (arg == "--edges" && i+1 < argc) edges_out = argv[++i];
        }

        // Load Image
        int width, height, channels;
        if (stbi_info(input_file.c_str(), &width, &height, &channels) == 0) {
            // Try magick conversion if stbi fails
             std::string temp = "cli_temp.png";
             std::string cmd = "magick " + input_file + " -strip " + temp;
             if (std::system(cmd.c_str()) == 0) {
                 input_file = temp;
             } else {
                 std::cerr << "Error: Could not load image " << input_file << std::endl;
                 return 1;
             }
        }

        auto edges = EdgeDetector::detect_edges(input_file, low, high, blur);
        if (edges.empty()) {
            std::cerr << "Warning: No edges detected.\n";
        }

        if (!edges_out.empty()) {
            write_points_png(edges, width, height, edges_out);
            std::cout << "Wrote Edges PNG to " << edges_out << std::endl;
        }
        
        auto path = PathPlanner::plan_path(edges, width, height);
        auto thr = ThrGenerator::generate_thr(path, width, height);
        
        std::ofstream out(output_thr);
        out << ThrGenerator::to_string(thr);
        out.close();
        std::cout << "Wrote THR to " << output_thr << std::endl;

        if (!gif_out.empty()) {
            GifGenerator::generate_gif(path, width, height, gif_out);
            std::cout << "Wrote GIF to " << gif_out << std::endl;
        }
        if (!png_out.empty()) {
            GifGenerator::generate_png(path, width, height, png_out);
            std::cout << "Wrote PNG to " << png_out << std::endl;
        }

    } else if (mode == "vis") {
        std::string gif_out, png_out, edges_out;
        int size = 1024;

        for (int i = 3; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--gif" && i+1 < argc) gif_out = argv[++i];
            else if (arg == "--png" && i+1 < argc) png_out = argv[++i];
            else if (arg == "--edges" && i+1 < argc) edges_out = argv[++i];
            else if (arg == "--size" && i+1 < argc) size = std::stoi(argv[++i]);
        }

        if (gif_out.empty() && png_out.empty() && edges_out.empty()) {
            std::cerr << "Error: Specify at least one output format (--gif, --png, or --edges)\n";
            return 1;
        }

        auto path = parse_thr(input_file, size, size);
        std::cout << "Parsed " << path.size() << " points from THR.\n";

        if (!gif_out.empty()) {
            GifGenerator::generate_gif(path, size, size, gif_out);
            std::cout << "Wrote GIF to " << gif_out << std::endl;
        }
        if (!png_out.empty()) {
            GifGenerator::generate_png(path, size, size, png_out);
            std::cout << "Wrote PNG to " << png_out << std::endl;
        }
        if (!edges_out.empty()) {
            write_points_png(path, size, size, edges_out);
            std::cout << "Wrote Edges PNG to " << edges_out << std::endl;
        }

    } else {
        print_usage(argv[0]);
        return 1;
    }

    return 0;
}