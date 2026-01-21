# ThrGen C++

A high-performance C++ tool for generating Sisyphus kinetic sand table tracks from images. This application processes input images (PNG, JPG, WebP) to detect outlines and optimizes them into a continuous polar coordinate path (`.thr` file).

## Features

-   **High Performance**: Built with C++17 for fast image processing and path optimization.
-   **No OpenCV Dependency**: Uses a custom, optimized Canny Edge Detector.
-   **Intelligent Path Planning**:
    -   **Component Connectivity**: Connects disjoint parts of an image using Minimum Spanning Tree logic.
    -   **Perimeter Routing**: Routes travel moves along the table's hidden perimeter to avoid drawing unwanted lines across the sand.
    -   **DFS Backtracking**: Ensures a continuous, valid path by doubling back over existing lines when necessary.
    -   **Noise Filtering**: Automatically removes small artifacts to ensure a clean result.
-   **Modern Web Interface**:
    -   **Dark Mode UI**: A professional, dark-themed interface.
    -   **Drag-and-Drop**: Easy file upload support.
    -   **Live Visualization**: View detected edges, the optimized path (gradient-colored for direction), and a simulated animation.
    -   **Zen Garden Simulation**: Generates an animated GIF showing a steel ball tracing the path on a sand bed.
-   **Robustness**:
    -   Handles various image formats (WebP/HEIC) via ImageMagick.
    -   Automatically resizes extremely large images to manageable dimensions while preserving aspect ratio.
    -   Masks image borders to prevent "frame" detection artifacts.

## Build Instructions

### Prerequisites
-   `cmake` (version 3.10+)
-   `g++` (or any C++17 compatible compiler)
-   `make`
-   `imagemagick` (runtime dependency for format conversion)

### Compilation
```bash
mkdir build
cd build
cmake ..
make
```

## Usage

### Web Server
1.  Run the server:
    ```bash
    ./ThrGenCpp
    ```
2.  Open your browser to: `http://localhost:8080`
3.  Upload an image and click "Generate Path".

### CLI Usage

The project includes a command-line interface `ThrGenCLI` for batch processing and visualization.

**1. Generate THR from Image**
```bash
./ThrGenCLI gen input.jpg output.thr --low 50 --high 150 --gif preview.gif
```
Options:
- `--low <val>`: Low threshold (default 50)
- `--high <val>`: High threshold (default 150)
- `--blur <val>`: Blur kernel size (default 5)
- `--gif <file>`: Generate an animated GIF visualization
- `--png <file>`: Generate a static path image

**2. Visualize Existing THR File**
```bash
./ThrGenCLI vis output.thr --gif animation.gif --size 512
```
Options:
- `--gif <file>`: Output GIF filename
- `--png <file>`: Output PNG filename
- `--size <int>`: Resolution for the output (default 1024)

## SD Card Organization

You can use the provided Python script to organize a Sisyphus SD card. This scans for `.thr` files, moves them into a structured `patterns/` directory, and generates static and animated previews for each.

```bash
python3 organize_sd.py /path/to/sd_card --cli ./build/ThrGenCLI
```

Structure created:
```
/path/to/sd_card/
└── patterns/
    ├── pattern1/
    │   ├── pattern1.thr
    │   ├── pattern1.png
    │   └── pattern1.gif
    └── ...
```

## Testing

The project includes a comprehensive test suite covering edge detection, path planning, and end-to-end integration.

```bash
# Run all tests
./ThrGenCpp_Tests
```

## Documentation

For a deep dive into the system design, algorithms, and data flow, please refer to the [Architecture Documentation](docs/ARCHITECTURE.md).

## Architecture

-   **Frontend**: HTML5/CSS3/JavaScript (Single Page Application).
-   **Backend**: C++ (`cpp-httplib` for server, `stb_image` for I/O, `nlohmann/json` for data).
-   **Core Logic**:
    -   `EdgeDetector`: Custom implementation of Canny edge detection.
    -   `PathPlanner`: Graph-based connectivity and traversal optimization.
    -   `ThrGenerator`: Cartesian-to-Polar conversion for Sisyphus format.
    -   `GifGenerator`: Rasterizes the path into an animated preview.