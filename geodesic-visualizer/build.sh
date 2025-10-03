#!/bin/bash

# Geodesic Flow Visualizer - Build Script
# Compiles C++ code to WebAssembly using Emscripten

echo "=========================================="
echo "Geodesic Flow Visualizer - Build Script"
echo "=========================================="
echo ""

# Check if Emscripten is available
if ! command -v em++ &> /dev/null
then
    echo "ERROR: Emscripten (em++) not found!"
    echo "Please install and activate Emscripten SDK:"
    echo "  git clone https://github.com/emscripten-core/emsdk.git"
    echo "  cd emsdk"
    echo "  ./emsdk install latest"
    echo "  ./emsdk activate latest"
    echo "  source ./emsdk_env.sh"
    exit 1
fi

echo "Emscripten found: $(em++ --version | head -n 1)"
echo ""

# Create build directory if it doesn't exist
if [ ! -d "build" ]; then
    echo "Creating build directory..."
    mkdir -p build
fi

echo "Compiling C++ to WebAssembly..."
echo ""

# Compile with Emscripten
em++ src/main.cpp src/geometry_core.cpp \
  -I./include \
  -o build/geodesic.js \
  -s WASM=1 \
  -s EXPORTED_FUNCTIONS='["_malloc","_free"]' \
  -s EXPORTED_RUNTIME_METHODS='["ccall","cwrap"]' \
  -s ALLOW_MEMORY_GROWTH=1 \
  -s MODULARIZE=1 \
  -s EXPORT_NAME='createGeodesicModule' \
  -s TOTAL_MEMORY=67108864 \
  -s INITIAL_MEMORY=33554432 \
  -O3 \
  -std=c++17 \
  --bind \
  -s DISABLE_EXCEPTION_CATCHING=0 \
  -s ASSERTIONS=1

# Check if compilation was successful
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "BUILD SUCCESSFUL!"
    echo "=========================================="
    echo ""
    echo "Generated files:"
    echo "  - build/geodesic.js"
    echo "  - build/geodesic.wasm"
    echo ""
    echo "File sizes:"
    ls -lh build/geodesic.js build/geodesic.wasm | awk '{print "  " $9 ": " $5}'
    echo ""
    echo "To run the application:"
    echo "  1. Start the development server: ./serve.sh"
    echo "  2. Open browser: http://localhost:8000/web/index.html"
    echo ""
else
    echo ""
    echo "=========================================="
    echo "BUILD FAILED!"
    echo "=========================================="
    echo ""
    echo "Please check the error messages above."
    exit 1
fi