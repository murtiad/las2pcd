cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(las2pcd)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})

include_directories(E:/These_Murtiyoso/2017/03_C++/MSVC_2010/Dep/liblas/include)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(las2pcd las2pcd.cpp)

target_link_libraries(las2pcd 
${PCL_COMMON_LIBRARIES}
${PCL_IO_LIBRARIES}
E:/These_Murtiyoso/2017/03_C++/MSVC_2010/Dep/liblas/lib/liblas.lib
E:/These_Murtiyoso/2017/03_C++/MSVC_2010/Dep/liblas/lib/liblas_c.lib
)
