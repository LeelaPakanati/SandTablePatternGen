#pragma once

#include <vector>
#include <thread>
#include <future>
#include <algorithm>
#include <functional>

namespace Utils {

    template <typename Index, typename Func>
    void parallel_for(Index start, Index end, Func&& f) {
        unsigned int num_threads = std::thread::hardware_concurrency();
        if (num_threads == 0) num_threads = 2;

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
