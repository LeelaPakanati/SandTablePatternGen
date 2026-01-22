#include "httplib.h"
#include "json.hpp"
#include "EdgeDetector.h"
#include "PathPlanner.h"
#include "ThrGenerator.h"
#include "GifGenerator.h"
#include "Utils.h"
#include "stb_image.h"
#include "stb_image_write.h"

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <ctime>

using json = nlohmann::json;

int main() {
    httplib::Server svr;

    // Serve static files - check root or build directory
    std::string static_path = "./static";
    if (!std::ifstream("static/index.html").good()) {
        if (std::ifstream("../static/index.html").good()) {
            static_path = "../static";
        }
    }
    
    std::cout << "Mounting static files from: " << static_path << std::endl;
    svr.set_mount_point("/", static_path);

    // Default route for / to serve index.html explicitly
    svr.Get("/", [static_path](const httplib::Request&, httplib::Response& res) {
        std::ifstream ifs(static_path + "/index.html");
        if (ifs.good()) {
            std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
            res.set_content(content, "text/html");
        } else {
            res.status = 404;
            res.set_content("Index file not found", "text/plain");
        }
    });

    // Process endpoint
    svr.Post("/process", [](const httplib::Request& req, httplib::Response& res) {
        if (!req.form.has_file("image")) {
            res.status = 400;
            res.set_content("No image file provided", "text/plain");
            return;
        }

        const auto& file = req.form.get_file("image");
        int low = std::stoi(req.form.get_field("low_threshold"));
        int high = std::stoi(req.form.get_field("high_threshold"));
        int blur = std::stoi(req.form.get_field("blur"));

        // Save original file
        std::string upload_filename = "temp_upload"; 
        {
            std::ofstream ofs(upload_filename, std::ios::binary);
            ofs.write(file.content.data(), file.content.size());
        }

        std::string target_file = upload_filename;
        std::string converted_filename = "temp_processed.png";
        
        int width, height, channels;
        bool needs_conversion = false;

        // Check if stbi can load it and check dimensions
        if (stbi_info(upload_filename.c_str(), &width, &height, &channels) == 1) {
            if (width > 2048 || height > 2048) {
                std::cout << "Image too large (" << width << "x" << height << "), resizing..." << std::endl;
                needs_conversion = true;
            }
        } else {
            std::cout << "stbi load failed (unsupported format), converting..." << std::endl;
            needs_conversion = true;
        }

        if (needs_conversion) {
            // Native C++ resizing
            unsigned char* raw_data = stbi_load(upload_filename.c_str(), &width, &height, &channels, 0);
            if (raw_data) {
                int new_width = width, new_height = height;
                if (width > 2048 || height > 2048) {
                    float ratio = std::min(2048.0f / width, 2048.0f / height);
                    new_width = (int)(width * ratio);
                    new_height = (int)(height * ratio);
                }
                
                auto resized = EdgeDetector::resize(raw_data, width, height, channels, new_width, new_height);
                stbi_image_free(raw_data);
                
                // Save to temp_processed.png for the next steps (or refactor to use buffer directly)
                stbi_write_png(converted_filename.c_str(), new_width, new_height, channels, resized.data(), new_width * channels);
                target_file = converted_filename;
            } else {
                // Fallback to magick if stbi still can't load it (e.g. specialized WebP/HEIC)
                std::string cmd = "magick " + upload_filename + " -resize '2048x2048>' -strip " + converted_filename;
                if (std::system(cmd.c_str()) == 0) {
                    target_file = converted_filename;
                } else {
                    res.status = 400;
                    res.set_content("Image conversion/resizing failed", "text/plain");
                    return;
                }
            }
        }

        // Get final dimensions (reload info to be sure of new size)
        if (stbi_info(target_file.c_str(), &width, &height, &channels) == 0) {
             res.status = 400;
             res.set_content("Invalid image format or conversion failed", "text/plain");
             return;
        }

        std::cout << "Processing image: " << width << "x" << height << std::endl;

        // Clear timing report for this request
        Utils::clear_timing_report();

        // Processing with timing
        std::vector<Point> edges;
        {
            Utils::Timer t("edge_detection", true, true);
            edges = EdgeDetector::detect_edges(target_file, low, high, blur);
        }

        std::vector<Point> path;
        {
            Utils::Timer t("path_planning", true, true);
            path = PathPlanner::plan_path(edges, width, height);
        }

        std::vector<ThrPoint> thr;
        std::string thr_content;
        {
            Utils::Timer t("thr_generation", true, true);
            thr = ThrGenerator::generate_thr(path, width, height);
            thr_content = ThrGenerator::to_string(thr);
        }

        // Generate GIF
        std::time_t t = std::time(nullptr);
        std::string base_name = "sim_" + std::to_string(t);
        std::string gif_filename = "static/" + base_name + ".gif";
        {
            Utils::Timer timer("gif_generation", true, true);
            GifGenerator::generate_gif(path, width, height, gif_filename);
        }
        std::cout << "Generated GIF: " << gif_filename << std::endl;

        // Generate PNG (white path on transparent background for table overlay)
        std::string png_name = base_name + ".png";
        std::string png_filename = "static/" + png_name;
        {
            Utils::Timer timer("png_generation", true, true);
            GifGenerator::generate_png(path, width, height, png_filename);
        }
        std::cout << "Generated PNG: " << png_filename << std::endl;

        // Print timing report
        Utils::get_timing_report().print();

        // JSON response
        json response;
        response["thr"] = thr_content;
        response["gif_url"] = "/" + base_name + ".gif"; 
        response["png_url"] = "/" + base_name + ".png";
        
        // Raw edges
        std::vector<std::vector<int>> edges_preview;
        for(const auto& p : edges) {
            edges_preview.push_back({p.x, p.y});
        }
        response["edges"] = edges_preview;

        // Path
        std::vector<std::vector<int>> preview_path;
        for(const auto& p : path) {
            preview_path.push_back({p.x, p.y});
        }
        response["preview"] = preview_path;
        response["width"] = width;
        response["height"] = height;

        // Add timing data to response
        json timing;
        const auto& report = Utils::get_timing_report();
        for (const auto& p : report.stages) {
            timing[p.first] = p.second;
        }
        timing["total_ms"] = report.total_ms();
        response["timing"] = timing;

        res.set_content(response.dump(), "application/json");
        
        // Cleanup
        std::remove(upload_filename.c_str());
        std::remove(converted_filename.c_str());
    });

    // Process THR file endpoint - regenerate visualization from existing .thr
    svr.Post("/process_thr", [](const httplib::Request& req, httplib::Response& res) {
        if (!req.form.has_file("thr")) {
            res.status = 400;
            res.set_content("No THR file provided", "text/plain");
            return;
        }

        const auto& file = req.form.get_file("thr");
        std::string thr_content(file.content.data(), file.content.size());

        // Parse the .thr file
        auto thr_points = ThrGenerator::parse(thr_content);
        if (thr_points.empty()) {
            res.status = 400;
            res.set_content("Invalid or empty THR file", "text/plain");
            return;
        }

        // Use default dimensions (square canvas)
        int width = 1024;
        int height = 1024;

        // Convert polar to Cartesian for visualization
        auto path = ThrGenerator::to_cartesian(thr_points, width, height);

        std::cout << "Processing THR file: " << thr_points.size() << " points" << std::endl;

        // Generate GIF
        std::time_t t = std::time(nullptr);
        std::string base_name = "sim_" + std::to_string(t);
        std::string gif_filename = "static/" + base_name + ".gif";
        GifGenerator::generate_gif(path, width, height, gif_filename);
        std::cout << "Generated GIF: " << gif_filename << std::endl;

        // Generate PNG
        std::string png_name = base_name + ".png";
        std::string png_filename = "static/" + png_name;
        GifGenerator::generate_png(path, width, height, png_filename);
        std::cout << "Generated PNG: " << png_filename << std::endl;

        // JSON response
        json response;
        response["thr"] = thr_content;
        response["gif_url"] = "/" + base_name + ".gif";
        response["png_url"] = "/" + base_name + ".png";

        // Path for preview
        std::vector<std::vector<int>> preview_path;
        for (const auto& p : path) {
            preview_path.push_back({p.x, p.y});
        }
        response["preview"] = preview_path;
        response["edges"] = json::array(); // No edges for THR import
        response["width"] = width;
        response["height"] = height;

        res.set_content(response.dump(), "application/json");
    });

    std::cout << "Server started at http://localhost:8080" << std::endl;
    svr.listen("0.0.0.0", 8080);

    return 0;
}