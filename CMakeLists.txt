cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(FACE_FEATURE_MATCHING)

find_package(PCL 1.8 REQUIRED)

set (CMAKE_CXX_STANDARD 14)
set (EXE_FEATURE_EXTRACTION feature_extraction)
set (EXE_FEATURE_MATCHING feature_matching)

include_directories(${PCL_INCLUDE_DIRS}  )
link_directories(${PCL_LIBRARY_DIRS} ./local_include/)
add_definitions(${PCL_DEFINITIONS})

# first executable (extraction)
add_executable(${EXE_FEATURE_EXTRACTION} feature_extraction.cpp)
target_link_libraries (${EXE_FEATURE_EXTRACTION} ${PCL_LIBRARIES})

# second executable (matching features)
add_executable(${EXE_FEATURE_MATCHING} feature_matching.cpp)
target_link_libraries (${EXE_FEATURE_MATCHING} ${PCL_LIBRARIES})

