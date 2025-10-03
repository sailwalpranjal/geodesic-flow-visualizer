#!/bin/bash

# Geodesic Flow Visualizer - Development Server Script
# Starts a local HTTP server to serve the application

echo "=========================================="
echo "Geodesic Flow Visualizer - Dev Server"
echo "=========================================="
echo ""

# Check if build files exist
if [ ! -f "build/geodesic.js" ] || [ ! -f "build/geodesic.wasm" ]; then
    echo "ERROR: Build files not found!"
    echo "Please run ./build.sh first to compile the project."
    echo ""
    exit 1
fi

# Check if Python 3 is available
if command -v python3 &> /dev/null
then
    PYTHON_CMD="python3"
elif command -v python &> /dev/null
then
    PYTHON_VERSION=$(python --version 2>&1 | awk '{print $2}' | cut -d. -f1)
    if [ "$PYTHON_VERSION" = "3" ]; then
        PYTHON_CMD="python"
    else
        echo "ERROR: Python 3 not found!"
        echo "Please install Python 3 to run the development server."
        exit 1
    fi
else
    echo "ERROR: Python not found!"
    echo "Please install Python 3 to run the development server."
    exit 1
fi

echo "Starting development server..."
echo ""
echo "Server Information:"
echo "  URL: http://localhost:8000/web/index.html"
echo "  Port: 8000"
echo ""
echo "Press Ctrl+C to stop the server"
echo "=========================================="
echo ""

# Start the server
$PYTHON_CMD -m http.server 8000