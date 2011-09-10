#/bin/sh
rm CMakeCache.txt
rm -rf CMakeFiles
cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug .
