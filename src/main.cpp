#include "httplib.h"
#include "json.hpp"
#include "EdgeDetector.h"
#include "PathPlanner.h"
#include "ThrGenerator.h"
#include "GifGenerator.h"
#include "stb_image.h"

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <ctime>

using json = nlohmann::json;

int main() {
    httplib::Server svr;

    // Serve static files
    svr.set_mount_point("/", "./static");

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
            // Resize to max 2048x2048, maintaining aspect ratio. 
            // The '>' flag means "only shrink if larger than dimensions".
            std::string cmd = "magick " + upload_filename + " -resize '2048x2048>' -strip " + converted_filename;
            int ret = std::system(cmd.c_str());
            
            if (ret == 0) {
                target_file = converted_filename;
            } else {
                res.status = 400;
                res.set_content("Image conversion/resizing failed", "text/plain");
                return;
            }
        }

        // Get final dimensions (reload info to be sure of new size)
        if (stbi_info(target_file.c_str(), &width, &height, &channels) == 0) {
             res.status = 400;
             res.set_content("Invalid image format or conversion failed", "text/plain");
             return;
        }

        std::cout << "Processing image: " << width << "x" << height << std::endl;

        // Processing
        auto edges = EdgeDetector::detect_edges(target_file, low, high, blur);
        auto path = PathPlanner::plan_path(edges, width, height);
        auto thr = ThrGenerator::generate_thr(path, width, height);
        std::string thr_content = ThrGenerator::to_string(thr);

        // Generate GIF
        std::time_t t = std::time(nullptr);
        std::string gif_name = "sim_" + std::to_string(t) + ".gif";
        std::string gif_filename = "static/" + gif_name;
        GifGenerator::generate_gif(path, width, height, gif_filename);
        std::cout << "Generated GIF: " << gif_filename << std::endl;

        // JSON response
        json response;
        response["thr"] = thr_content;
        response["gif_url"] = "/" + gif_name; 
        
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

        res.set_content(response.dump(), "application/json");
        
        // Cleanup
        std::remove(upload_filename.c_str());
        std::remove(converted_filename.c_str());
    });

    std::cout << "Server started at http://localhost:8080" << std::endl;
    svr.listen("0.0.0.0", 8080);

    return 0;
}