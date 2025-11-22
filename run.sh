
#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$ROOT_DIR/build"
BIN_DIR="$BUILD_DIR/src"

# If cache is broken or missing, clean build folder
if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]]; then
    echo "No valid CMake cache found. Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

# Configure
cmake -S "$ROOT_DIR" -B "$BUILD_DIR"

# Build targets
cmake --build "$BUILD_DIR" --target master bb_server drone input obstacles targets

# Sanity check
if [[ ! -x "$BIN_DIR/master" ]]; then
    echo "Error: $BIN_DIR/master not found or not executable."
    echo "Contents of $BIN_DIR:"
    ls -l "$BIN_DIR"
    exit 1
fi

cd "$BIN_DIR"
exec ./master
