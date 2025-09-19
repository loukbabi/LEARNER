cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM3 VERSION 1.0.0)



IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")

set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -fopenmp -march=native -pthread")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -O3")
set(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS} -g")
# Check C++11 or C++0x support


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#set(OpenCV_DIR "~/Workspaces/PhD_ws/src/Prerequisites/OpenCV34/install/lib/cmake/opencv3")

unset(OpenCV_FOUND CACHE)
unset(OpenCV_INCLUDE_DIRS CACHE)
unset(OpenCV_LIBS CACHE)

# Use default system OpenCV
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
find_package(OpenCV 4.2 REQUIRED)


message(STATUS "Using OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include path: ${OpenCV_INCLUDE_DIRS}")




MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)
# find_package(realsense2)
#find_package(Torch REQUIRED)
# --- Manually Define the ONNX Runtime IMPORTED Library Target ---
# This is the modern, robust way to handle pre-built libraries without CMake support.
add_library(onnxruntime::onnxruntime INTERFACE IMPORTED GLOBAL)

set_property(TARGET onnxruntime::onnxruntime PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES "/home/anast/Requirements/onnxruntime-linux-x64-gpu-1.16.3/include"
)

set_property(TARGET onnxruntime::onnxruntime PROPERTY
  INTERFACE_LINK_LIBRARIES "/home/anast/Requirements/onnxruntime-linux-x64-gpu-1.16.3/lib/libonnxruntime.so"
)
# --- End of ONNX Runtime Definition ---
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}

${CUDA_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${YAML_CPP_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${GFLAGS_INCLUDE_DIRS}
${GLOG_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
        ${TORCH_INCLUDE_DIRS}

)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED

src/Random.cpp
src/Timestamp.cpp
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
src/Extractors/SPextractor.cc
src/Extractors/superpoint_onnx.cc
src/Matchers/SPmatcher.cc
src/Matchers/lightglue_onnx.cpp
src/Matchers/transform.cpp
include/Timestamp.h
include/Random.h
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Extractors/SPextractor.h
include/Extractors/superpoint_onnx.h
include/Matchers/SPmatcher.h
include/Matchers/lightglue_onnx.h
include/Matchers/Configuration.h
include/Matchers/transform.h
include/Settings.h
#testyolo.cpp
src/YoloDetection.cpp
include/YoloDetection.h

)



add_library(ORB_SLAM3::ORB_SLAM3 ALIAS ${PROJECT_NAME}) 

add_subdirectory(Thirdparty/g2o)




target_link_libraries(${PROJECT_NAME}
  ${CUDA_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW3/lib/libDBoW3.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${TORCH_LIBRARIES}
  onnxruntime::onnxruntime  
  -lboost_serialization
  -lcrypto
)



# Build examples



#Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)





add_executable(mono_tum
        Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
        Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
        Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(mono_tum_vi
        Examples/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi ${PROJECT_NAME})


#Monocular inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

add_executable(mono_inertial_euroc
        Examples/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

add_executable(mono_inertial_tum_vi
        Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})



set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

add_executable(stereo_inertial_euroc
        Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})







# Install the main library target and create an "export set"
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

# Install the headers from your 'include' directory
install(DIRECTORY include/
    DESTINATION include)

# THIS IS THE KEY FIX: Install the headers for your third-party libraries too.
# This ensures that when our ROS 2 node includes "System.h", it can also
# find the relative includes like "Thirdparty/DBoW3/src/DBoW3.h".
install(DIRECTORY Thirdparty/
    DESTINATION include/Thirdparty
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Generate and install the CMake configuration files
include(CMakePackageConfigHelpers)
install(EXPORT ${PROJECT_NAME}Targets
  FILE
    ${PROJECT_NAME}Targets.cmake
  NAMESPACE
    ${PROJECT_NAME}::
  DESTINATION
    lib/cmake/${PROJECT_NAME}
)

configure_package_config_file(${PROJECT_NAME}Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${${PROJECT_NAME}_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "lib/cmake/${PROJECT_NAME}"
)




