# ThrGen C++ Architecture Documentation

## Overview

ThrGen C++ is a high-performance tool designed to convert raster images into kinetic art tracks (`.thr` files) for Sisyphus tables. It is built with C++17 for speed and minimal dependencies, using a single-page web interface for interaction.

## System Architecture

The application follows a monolithic architecture with a clear separation between the C++ backend and the HTML/JS frontend.

```mermaid
graph TD
    User[User Browser] <-->|HTTP/JSON| Server[C++ Server (httplib)]
    Server --> EdgeDetector[Edge Detector]
    Server --> PathPlanner[Path Planner]
    Server --> ThrGenerator[THR Generator]
    Server --> GifGenerator[GIF Generator]
    Server --> ImgIO[Image I/O (stb/magick)]
```

### Components

1.  **Web Server (`main.cpp`)**:
    -   Uses `cpp-httplib` to serve static files and handle the `/process` API endpoint.
    -   Orchestrates the pipeline: Upload $\to$ Resize/Convert $\to$ Edge Detect $\to$ Plan Path $\to$ Generate THR $\to$ Generate GIF.
    -   Handles image format normalization using `imagemagick` (fallback for WebP/HEIC) and `stb_image`.

2.  **Edge Detector (`EdgeDetector.cpp`)**:
    -   **Input**: Raw image data.
    -   **Output**: Vector of edge points.
    -   **Algorithm**: Custom Canny Edge Detection implementation.
        -   **Grayscale**: Luminance conversion (Parallelized).
        -   **Gaussian Blur**: Noise reduction (Parallelized).
        -   **Sobel Operator**: Gradient magnitude and direction calculation (Parallelized).
        -   **Non-Max Suppression**: Thinning edges (Parallelized).
        -   **Hysteresis Thresholding**: Connecting strong/weak edges (Sequential).
    -   **Optimization**: Border pixels (5px margin) are explicitly masked to prevent frame detection artifacts.

3.  **Path Planner (`PathPlanner.cpp`)**:
    -   **Goal**: Convert a cloud of edge points into a single continuous path.
    -   **Stage 1: Component Labeling**: Uses BFS to group connected pixels into components. Filters out small noise (< 15 px).
    -   **Stage 2: Connectivity (MST)**: Connects disjoint components into a single graph.
        -   Uses a greedy Minimum Spanning Tree (MST) approach.
        -   Calculates distances between all component pairs (Optimized with strided sampling and Parallelization).
        -   **Perimeter Routing**: If components are far apart, the bridge path is routed along the table's perimeter (Rho=1.0) to hide the travel line.
    -   **Stage 3: Traversal (DFS)**: Performs a Depth-First Search on the connected graph.
        -   Implements backtracking: When the "pen" hits a dead end, it records the path back to the junction, ensuring the ball never lifts.

4.  **THR Generator (`ThrGenerator.cpp`)**:
    -   **Input**: Ordered Cartesian path.
    -   **Output**: Theta-Rho coordinates.
    -   **Logic**:
        -   Converts $(x, y)$ to $(\theta, \rho)$.
        -   Unwraps $\theta$ to ensure continuous rotation (avoiding jumps from $\pi$ to $-\pi$).
        -   Normalizes $\rho$ to $[0, 1]$.

5.  **GIF Generator (`GifGenerator.cpp`)**:
    -   Rasterizes the vector path into a pixel grid.
    -   Uses `gif.h` to write frames.
    -   **Simulation**: Draws a persistent "track" (cumulative frames) and a temporary "ball" overlay to simulate the Sisyphus table effect.

## Key Optimizations

-   **Multi-threading**: `Utils::parallel_for` distributes pixel-wise operations (Blur, Sobel) and component distance calculations across available CPU cores.
-   **Strided Sampling**: Instead of $O(N^2)$ pixel comparisons for component connection, the planner samples every $K$-th point, drastically reducing complexity for large images.
-   **Intelligent Resizing**: Images > 2048px are downscaled before processing to keep runtime interactive.

## Data Flow

1.  **Upload**: User drops an image.
2.  **Preprocessing**: If WebP or >2k resolution, `magick` converts/resizes it to PNG.
3.  **Loading**: `stb_image` loads raw bytes.
4.  **Detection**: Edges extracted into `std::vector<Point>`.
5.  **Planning**: Points reordered into a valid continuous path (graph traversal).
6.  **Generation**:
    -   Path converted to `.thr` string.
    -   Path rendered to `.gif` animation.
7.  **Response**: JSON containing the `.thr` text, GIF URL, and preview points sent back to browser.
