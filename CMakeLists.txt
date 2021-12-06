set(PROJECT_NAME Dvo)

cmake_minimum_required(VERSION 3.0)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) 
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "-std=c++14 -O3" )
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME}_rgbd SHARED ${PROJECT_SOURCE_DIR}/src/rgbd_image.cpp)
target_link_libraries(${PROJECT_NAME}_rgbd PUBLIC ${OpenCV_LIBS})

add_library(${PROJECT_NAME}_intrinsic SHARED ${PROJECT_SOURCE_DIR}/src/intrinsic.cpp)
target_link_libraries(${PROJECT_NAME}_intrinsic PUBLIC Eigen3::Eigen)

add_executable(rgbd_test ${PROJECT_SOURCE_DIR}/src/experiments/rgbd_test.cpp)
target_link_libraries(rgbd_test PUBLIC ${PROJECT_NAME}_rgbd ${OpenCV_LIBS})
                                        
add_executable( solver_test_bench ${PROJECT_SOURCE_DIR}/src/experiments/solver_tb.cpp)
target_link_libraries( solver_test_bench PUBLIC ${PROJECT_NAME}_rgbd ${PROJECT_NAME}_intrinsic ${CERES_LIBRARIES} ${OpenCV_LIBS} )                                    

# add_executable(frontend_solver ${PROJECT_SOURCE_DIR}/src/FrontendSolver.cpp)
# target_link_libraries( frontend_solver PUBLIC Eigen3::Eigen ${PROJECT_NAME}_rgbd ${PROJECT_NAME}_intrinsic ${CERES_LIBRARIES} ${OpenCV_LIBS} )                                    