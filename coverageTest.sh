#!/bin/bash
# Helper program to build everything with one command.
# Usage: ./build.sh [Debug|Release]

# Default build type is Release
BUILD_TYPE=Debug
BLDDIR=bldCoverage
CMAKE_OPTIONS=-DBUILD_WITH_COVERAGE=ON

if [ $# -gt 0 ]; then
    # Override build type
    BUILD_TYPE=$1
fi
rm -rf CMakeCache.txt
cmake -DBUILD_TYPE=$BUILD_TYPE $CMAKE_OPTIONS . || exit 1
make
make lcov || exit 1

echo "Coverage analysis completed."
