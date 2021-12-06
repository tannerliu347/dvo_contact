project(experiment)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )
#message(${PROJECT_SOURCE_DIR}/include)
#message(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable( solver_test_bench solver_tb.cpp )
# 与Ceres和OpenCV链接
target_link_libraries( solver_test_bench PUBLIC ${PROJECT_NAME}_rgbd ${CERES_LIBRARIES} ${OpenCV_LIBS} )
