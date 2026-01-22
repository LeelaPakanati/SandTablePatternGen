# ThrGen C++ Optimization Task List

This document tracks planned and completed performance optimizations for the ThrGen engine.

## 1. Path Planner: Efficient Global Search
- [x] Implement a Spatial Index (Grid-of-Buckets) for unvisited points to replace $O(N)$ BFS search.
- [x] Optimize "trace-back" pathfinding by using a simplified graph of junction points rather than pixel-by-pixel BFS. (Optimized with reused buffers and sparse grid exploration).
- [x] Reduce redundant distance calculations in the jumping logic. (Implemented Bounding Box Pruning and Component-based Sampling).

## 2. Parallelize Component Labeling & Hysteresis
- [x] Replace stack-based BFS component labeling with a Parallel Disjoint Set Union (DSU) algorithm.
- [x] Parallelize the seed-finding phase of Hysteresis thresholding.

## 3. Imaging Pipeline Efficiency
- [x] Implement native C++ bilinear downsampling to remove `ImageMagick` dependency for resizing.
- [x] Optimize `GifGenerator` background initialization.
- [x] Optimize `Bridge Gaps` using spatial grid and parallelization.
- [ ] Add support for "Fast Mode" which simplifies the path during generation.

## 4. Hardware-Level Optimizations (SIMD)
- [x] Implement SIMD-friendly loops for Gaussian Blur.
- [x] Implement SIMD-friendly loops for Sobel Operator calculations.
- [ ] Use `std::vector<bool>` alternatives (like `std::vector<uint8_t>`) more strategically to avoid bit-packing overhead in parallel sections. (Partially done in EdgeDetector).

## 5. Benchmarking & Profiling
- [ ] Add detailed timing instrumentation for each stage of the pipeline (Edge Detect, Plan, Generate, GIF).
- [ ] Create a regression script to ensure optimizations do not degrade output quality.
