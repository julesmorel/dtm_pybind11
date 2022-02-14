cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(dtm)

find_package(PCL 1.11 REQUIRED)
find_package(GSL REQUIRED)

set(CMAKE_BUILD_TYPE Release)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FILE(GLOB SRCFILES *.cpp)
FILE(GLOB CORESRCFILES core/*.cpp)
FILE(GLOB PATCHSRCFILES core/localCell/*.cpp)
FILE(GLOB PLOTSRCFILES plot/*.cpp)
FILE(GLOB UTILSSRCFILES utils/*.cpp)
add_executable (dtm ${SRCFILES} ${CORESRCFILES} ${PLOTSRCFILES} ${PATCHSRCFILES} ${UTILSSRCFILES})
target_link_libraries (dtm pthread ${PCL_LIBRARIES} -lgsl)
