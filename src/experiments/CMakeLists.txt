cmake_minimum_required( VERSION 2.8 )
project( solver_test_bench )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

add_executable( solver_test_bench solver_tb.cpp )
# 与Ceres和OpenCV链接
target_link_libraries( solver_test_bench ${CERES_LIBRARIES} ${OpenCV_LIBS} )