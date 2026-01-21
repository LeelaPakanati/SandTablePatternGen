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

void print_usage(const char* name) {
    std::cout << "Usage:\n";
    std::cout << "  Generate THR from Image:\n";
    std::cout << "    " << name << " gen <input_image> <output_thr> [options]\n";
    std::cout << "    Options:\n";
    std::cout << "      --low <val>    Low threshold (default 50)\n";
    std::cout << "      --high <val>   High threshold (default 150)\n";
    std::cout << "      --blur <val>   Blur kernel size (default 5)\n";
    std::cout << "      --gif <file>   Generate visualization GIF\n";
    std::cout << "      --png <file>   Generate static path image (sand style)\n";
    std::cout << "      --grad <file>  Generate static path image (gradient style)\n\n";
    
    std::cout << "  Visualize THR file:\n";
    std::cout << "    " << name << " vis <input_thr> [options]\n";
    std::cout << "    Options:\n";
    std::cout << "      --gif <file>   Output GIF filename\n";
    std::cout << "      --png <file>   Output PNG filename (sand style)\n";
    std::cout << "      --grad <file>  Output PNG filename (gradient style)\n";
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
        std::string gif_out, png_out, grad_out;

        for (int i = 4; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--low" && i+1 < argc) low = std::stoi(argv[++i]);
            else if (arg == "--high" && i+1 < argc) high = std::stoi(argv[++i]);
            else if (arg == "--blur" && i+1 < argc) blur = std::stoi(argv[++i]);
            else if (arg == "--gif" && i+1 < argc) gif_out = argv[++i];
            else if (arg == "--png" && i+1 < argc) png_out = argv[++i];
            else if (arg == "--grad" && i+1 < argc) grad_out = argv[++i];
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
        if (!grad_out.empty()) {
            GifGenerator::generate_gradient_png(path, width, height, grad_out);
            std::cout << "Wrote Gradient PNG to " << grad_out << std::endl;
        }

    } else if (mode == "vis") {
        std::string gif_out, png_out, grad_out;
        int size = 1024;

        for (int i = 3; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--gif" && i+1 < argc) gif_out = argv[++i];
            else if (arg == "--png" && i+1 < argc) png_out = argv[++i];
            else if (arg == "--grad" && i+1 < argc) grad_out = argv[++i];
            else if (arg == "--size" && i+1 < argc) size = std::stoi(argv[++i]);
        }

        if (gif_out.empty() && png_out.empty() && grad_out.empty()) {
            std::cerr << "Error: Specify at least one output format (--gif, --png, or --grad)\n";
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
        if (!grad_out.empty()) {
            GifGenerator::generate_gradient_png(path, size, size, grad_out);
            std::cout << "Wrote Gradient PNG to " << grad_out << std::endl;
        }

    } else {
        print_usage(argv[0]);
        return 1;
    }

    return 0;
}
