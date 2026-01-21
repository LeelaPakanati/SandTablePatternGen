#include "EdgeDetector.h"
#include "stb_image.h"
#include "Utils.h"
#include <cmath>
#include <stack>
#include <iostream>
#include <vector>
#include <algorithm>

const double PI = 3.14159265358979323846;

std::vector<Point> EdgeDetector::detect_edges(const std::string& image_path, int low_threshold, int high_threshold, int blur_kernel_size) {
    int width, height, channels;
    unsigned char* data = stbi_load(image_path.c_str(), &width, &height, &channels, 0);
    if (!data) {
        std::cerr << "Error loading image: " << image_path << std::endl;
        return {};
    }

    std::vector<Point> edges = detect_edges_from_memory(data, width, height, channels, low_threshold, high_threshold, blur_kernel_size);
    stbi_image_free(data);
    return edges;
}

std::vector<Point> EdgeDetector::detect_edges_from_memory(const unsigned char* data, int width, int height, int channels, int low_threshold, int high_threshold, int blur_kernel_size) {
    std::vector<uint8_t> gray = grayscale(data, width, height, channels);
    std::vector<uint8_t> blurred = gaussian_blur(gray, width, height, blur_kernel_size);

    std::vector<float> magnitude(width * height);
    std::vector<float> angle(width * height);
    sobel(blurred, width, height, magnitude, angle);

    std::vector<uint8_t> suppressed = non_max_suppression(magnitude, angle, width, height);

    // Mask borders to prevent frame detection
    int margin = 5;
    for (int i = 0; i < margin; ++i) {
        for (int x = 0; x < width; ++x) {
            suppressed[i * width + x] = 0; // Top
            suppressed[(height - 1 - i) * width + x] = 0; // Bottom
        }
        for (int y = 0; y < height; ++y) {
            suppressed[y * width + i] = 0; // Left
            suppressed[y * width + (width - 1 - i)] = 0; // Right
        }
    }

    return hysteresis(suppressed, width, height, low_threshold, high_threshold);
}

std::vector<uint8_t> EdgeDetector::grayscale(const unsigned char* data, int width, int height, int channels) {
    std::vector<uint8_t> gray(width * height);
    // Parallelize grayscale conversion
    Utils::parallel_for(0, width * height, [&](int i) {
        if (channels >= 3) {
            // Standard luminance weights
            gray[i] = static_cast<uint8_t>(0.299 * data[i * channels] + 0.587 * data[i * channels + 1] + 0.114 * data[i * channels + 2]);
        } else {
            gray[i] = data[i * channels];
        }
    });
    return gray;
}

std::vector<uint8_t> EdgeDetector::gaussian_blur(const std::vector<uint8_t>& image, int width, int height, int kernel_size) {
    if (kernel_size % 2 == 0) kernel_size++; 
    int r = kernel_size / 2;
    double sigma = std::max(1.0, kernel_size / 6.0); // Rough approximation
    
    std::vector<double> kernel(kernel_size * kernel_size);
    double sum = 0.0;

    for (int y = -r; y <= r; ++y) {
        for (int x = -r; x <= r; ++x) {
            double val = (1.0 / (2.0 * PI * sigma * sigma)) * exp(-(x * x + y * y) / (2.0 * sigma * sigma));
            kernel[(y + r) * kernel_size + (x + r)] = val;
            sum += val;
        }
    }

    // Normalize
    for (double& k : kernel) k /= sum;

    std::vector<uint8_t> output(width * height);

    // Parallelize blur (iterate over rows)
    Utils::parallel_for(r, height - r, [&](int y) {
        for (int x = r; x < width - r; ++x) {
            double val = 0.0;
            for (int ky = -r; ky <= r; ++ky) {
                for (int kx = -r; kx <= r; ++kx) {
                    val += image[(y + ky) * width + (x + kx)] * kernel[(ky + r) * kernel_size + (kx + r)];
                }
            }
            output[y * width + x] = static_cast<uint8_t>(std::min(255.0, std::max(0.0, val)));
        }
    });
    return output;
}

void EdgeDetector::sobel(const std::vector<uint8_t>& image, int width, int height, std::vector<float>& magnitude, std::vector<float>& angle) {
    // Standard Sobel kernels
    int Gx[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    int Gy[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};

    // Parallelize sobel (iterate over rows)
    Utils::parallel_for(1, height - 1, [&](int y) {
        for (int x = 1; x < width - 1; ++x) {
            float sumX = 0;
            float sumY = 0;

            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    int p = image[(y + i) * width + (x + j)];
                    sumX += p * Gx[i + 1][j + 1];
                    sumY += p * Gy[i + 1][j + 1];
                }
            }

            magnitude[y * width + x] = std::sqrt(sumX * sumX + sumY * sumY);
            angle[y * width + x] = std::atan2(sumY, sumX);
        }
    });
}

std::vector<uint8_t> EdgeDetector::non_max_suppression(const std::vector<float>& magnitude, const std::vector<float>& angle, int width, int height) {
    std::vector<uint8_t> suppressed(width * height, 0);

    // Parallelize suppression (iterate over rows)
    Utils::parallel_for(1, height - 1, [&](int y) {
        for (int x = 1; x < width - 1; ++x) {
            float ang = angle[y * width + x] * 180.0 / PI;
            if (ang < 0) ang += 180;

            float q = 255;
            float r = 255;

            // angle 0
            if ((0 <= ang && ang < 22.5) || (157.5 <= ang && ang <= 180)) {
                q = magnitude[y * width + (x + 1)];
                r = magnitude[y * width + (x - 1)];
            }
            // angle 45
            else if (22.5 <= ang && ang < 67.5) {
                q = magnitude[(y + 1) * width + (x - 1)];
                r = magnitude[(y - 1) * width + (x + 1)];
            }
            // angle 90
            else if (67.5 <= ang && ang < 112.5) {
                q = magnitude[(y + 1) * width + x];
                r = magnitude[(y - 1) * width + x];
            }
            // angle 135
            else if (112.5 <= ang && ang < 157.5) {
                q = magnitude[(y - 1) * width + (x - 1)];
                r = magnitude[(y + 1) * width + (x + 1)];
            }

            float mag = magnitude[y * width + x];
            if (mag >= q && mag >= r) {
                suppressed[y * width + x] = static_cast<uint8_t>(std::min(255.0f, mag));
            } else {
                suppressed[y * width + x] = 0;
            }
        }
    });
    return suppressed;
}

std::vector<Point> EdgeDetector::hysteresis(const std::vector<uint8_t>& image, int width, int height, int low, int high) {
    std::vector<Point> edges;
    std::vector<bool> visited(width * height, false);
    std::stack<Point> stack;

    // Identify strong edges and push to stack
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            int idx = y * width + x;
            if (image[idx] >= high && !visited[idx]) {
                stack.push({x, y});
                visited[idx] = true;
                edges.push_back({x, y});
            }
        }
    }

    // Connect weak edges
    while (!stack.empty()) {
        Point p = stack.top();
        stack.pop();

        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;

                int nx = p.x + dx;
                int ny = p.y + dy;

                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    int n_idx = ny * width + nx;
                    if (!visited[n_idx] && image[n_idx] >= low) {
                        visited[n_idx] = true;
                        stack.push({nx, ny});
                        edges.push_back({nx, ny});
                    }
                }
            }
        }
    }

    return edges;
}