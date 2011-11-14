#/bin/sh
if [ -e "CMakeCache.txt" ]
then
	rm CMakeCache.txt
fi
if [ -d "CMakeFiles" ]
then
	rm -rf CMakeFiles
fi
cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug .
