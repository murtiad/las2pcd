cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(las2pcd)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})

include_directories(/usr/include)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(las2pcd las2pcd.cpp)

target_link_libraries(las2pcd 
${PCL_COMMON_LIBRARIES}
${PCL_IO_LIBRARIES}
/usr/lib/liblas.so
/usr/lib/liblas_c.so
)
