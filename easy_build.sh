#!/bin/bash

# build GJK
g++ -Wall -fPIC -fopenmp -shared `python3 -m pybind11 --includes` -I ./src/local_planner/planner/include -I /usr/include/eigen3 ./src/local_planner/planner/src/pyopenGJK.cpp ./src/local_planner/planner/src/openGJK.c -o ./src/local_planner/planner/scripts/opengjkc.so
echo "GJK built successfully"

# build Homotopy-aware Path Planning algorithm
opencv_include_dir=$(pkg-config --cflags opencv4)
opencv_library_dir=$(pkg-config --libs-only-L opencv4)
opencv_libraries=$(pkg-config --libs-only-l opencv4)
pybind11_includes=$(python3 -m pybind11 --includes)
ros_includes_dir=$(pkg-config --cflags roscpp nav_msgs)
ros_libraries=$(pkg-config --libs roscpp nav_msgs)
custom_service_include_dir="-I$(pwd)/devel/include"
custom_service_library_dir="-L$(pwd)/devel/lib"
g++ -O3 -Wall -shared -std=c++11 -fPIC $pybind11_includes $opencv_include_dir $ros_includes_dir $custom_service_include_dir ./src/global_planner/copath/src/generate_homotopic_path_binding.cpp -o ./src/global_planner/copath/scripts/generate_homotopic_path.so $opencv_library_dir $(echo $opencv_libraries $ros_libraries $custom_service_library_dir | sed 's/-l/-l/g') -Wno-sign-compare -Wno-unused-but-set-variable -Wno-unused-variable -Wno-reorder
echo "Homotopy-aware Path Planning algorithm built successfully"