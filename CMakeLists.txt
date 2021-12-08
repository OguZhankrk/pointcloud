cmake_minimum_required(VERSION 3.0.0)
project(createRangeImage VERSION 0.1.0)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)



include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIB_DIR})
add_definitions(${PCL_DEFINITIONS} )


add_executable(${PROJECT_NAME} src/main.cpp src/data_type.h )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES}) 

