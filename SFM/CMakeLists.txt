cmake_minimum_required(VERSION 3.0)
project(pysfm)

##########################
# cmake standard
##########################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Debug")


##########################
# cmake warnings
##########################
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-result -Wno-ignored-attributes")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}-Wno-unused-result -Wno-ignored-attributes")


##########################
# cmake optimize
##########################
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Og -ffast-math -mtune=native -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Og -ffast-math -mtune=native -march=native")


##########################
# set 3rd directory
##########################
set(fbow_DIR ${PROJECT_SOURCE_DIR}/3rd/fbow/install/share/fbow)
### set ceres and sophus if not installed globally
# set(Ceres_DIR ${PROJECT_SOURCE_DIR}/3rd/ceres-soler/install/lib/cmake/Ceres)
# set(Sophus_DIR ${PROJECT_SOURCE_DIR}/3rd/Sophus/install/lib/cmake/Sophus)


##########################
# find package
##########################
find_package(fbow REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Protobuf REQUIRED)


##########################
# pybind11
##########################
add_subdirectory(${PROJECT_SOURCE_DIR}/3rd/pybind11)


##########################
# build pybind module
##########################
# set output
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

pybind11_add_module(${PROJECT_NAME}
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/MapDrawer.cc
  src/CeresOptimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Initializer.cc
  src/Viewer.cc
  src/MatEigenConverter.cc
  src/Osmap.cc
  src/osmap.pb.cc
  src/interface/Pybind.cc
  src/interface/SceneMVS.cc
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/3rd/spdlog/include
    ${PROJECT_SOURCE_DIR}/3rd/pybind11/include/
    /usr/include/python3.8/
)

target_link_libraries(${PROJECT_NAME} 
  PUBLIC
    Eigen3::Eigen
    Sophus::Sophus
  PRIVATE
    Ceres::ceres
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PROTOBUF_LIBRARIES}
    ${PCL_LIBRARIES}
    ${fbow_LIBS}
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "pysfm")
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".so")