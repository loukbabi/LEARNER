cmake_minimum_required(VERSION 3.10)

#set(OpenCV_DIR "/home/anast/Workspaces/PhD_ws/src/Prerequisites/OpenCV34/install/lib/cmake/opencv3" CACHE PATH "Path to OpenCV3" FORCE)
#set(CMAKE_PREFIX_PATH "/home/anast/Workspaces/PhD_ws/src/Prerequisites/OpenCV34/install" CACHE PATH "Prefix path" FORCE)

unset(OpenCV_FOUND CACHE)
unset(OpenCV_INCLUDE_DIRS CACHE)
unset(OpenCV_LIBS CACHE)

project(DBoW3)

set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

find_package(OpenCV 4.2 REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall  -O3 -march=native")

include_directories(${OpenCV_INCLUDE_DIRS})

set(HDRS_DBOW3
  src/BowVector.h
  src/Database.h
  src/DBoW3.h
  src/DescManip.h
  src/exports.h
  src/QueryResults.h
  src/quicklz.h
  src/timers.h
  src/FeatureVector.h
  src/ScoringObject.h   
  src/Vocabulary.h)

set(SRCS_DBOW3
  src/BowVector.cpp
  src/Database.cpp
  src/DescManip.cpp
  src/QueryResults.cpp
  src/quicklz.c
  src/FeatureVector.cpp
  src/ScoringObject.cpp   
  src/Vocabulary.cpp)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(DBoW3 SHARED ${SRCS_DBOW3})
target_link_libraries(DBoW3 ${OpenCV_LIBS})
add_executable(voc_to_bin converter.cpp)
target_link_libraries(voc_to_bin DBoW3 ${OpenCV_LIBS})
