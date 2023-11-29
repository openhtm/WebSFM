#!/bin/bash

echo ""
echo "Building fBow lib!"
echo ""

cd 3rd/fbow

mkdir build
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-march=native" -DCMAKE_INSTALL_PREFIX="../install/"
make -j4
cd ../../..


### uncomment the following lines if you want to build ceres-solver and Sophus

# echo ""
# echo "Building Sophus lib!"
# echo ""

# cd 3rd/Sophus

# mkdir build
# mkdir install
# cd build/
# cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-march=native" -DCMAKE_INSTALL_PREFIX="../install/" 
# make -j4 install
# cd ../../..

# echo ""
# echo "Building Ceres lib!"
# echo ""

# cd 3rd/ceres-solver
# mkdir build
# mkdir install
# cd build/
# cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 -DCMAKE_CXX_FLAGS="-march=native" -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
# make -j4 install
# cd ../../..

 