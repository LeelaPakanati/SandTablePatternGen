#pragma once

#include <vector>
#include <thread>
#include <future>
#include <algorithm>
#include <functional>

#include <iostream>
#include <chrono>

namespace Utils {

    class Timer {
    public:
        Timer(const std::string& name) : name_(name), start_(std::chrono::high_resolution_clock::now()) {}
        ~Timer() {
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> diff = end - start_;
            std::cout << "[TIMER] " << name_ << ": " << diff.count() << "s" << std::endl;
        }
    private:
        std::string name_;
        std::chrono::time_point<std::chrono::high_resolution_clock> start_;
    };

    inline bool thread_count_printed = false;

    template <typename Index, typename Func>
    void parallel_for(Index start, Index end, Func&& f) {
        unsigned int num_threads = std::thread::hardware_concurrency();
        
        if (num_threads == 0) {
            if (!thread_count_printed) {
                std::cerr << "Warning: std::thread::hardware_concurrency() failed to detect cores. Falling back to 2 threads." << std::endl;
                thread_count_printed = true;
            }
            num_threads = 2;
        } else if (!thread_count_printed) {
            std::cout << "Parallelizing across " << num_threads << " logical cores." << std::endl;
            thread_count_printed = true;
        }

        Index range = end - start;
        if (range == 0) return;

        if (range < static_cast<Index>(num_threads)) {
            num_threads = static_cast<unsigned int>(range);
        }

        std::vector<std::future<void>> futures;
        Index chunk_size = range / num_threads;
        Index remainder = range % num_threads;

        Index current_start = start;

        for (unsigned int i = 0; i < num_threads; ++i) {
            Index current_chunk = chunk_size + (static_cast<Index>(i) < remainder ? 1 : 0);
            Index current_end = current_start + current_chunk;

            futures.push_back(std::async(std::launch::async, [current_start, current_end, &f]() {
                for (Index j = current_start; j < current_end; ++j) {
                    f(j);
                }
            }));

            current_start = current_end;
        }

        for (auto& fut : futures) {
            if (fut.valid()) {
                fut.get();
            }
        }
    }

}
