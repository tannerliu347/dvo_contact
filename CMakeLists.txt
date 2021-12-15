set(PROJECT_NAME Dvo)

cmake_minimum_required(VERSION 3.0)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) 
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
# find_package(Pangolin 0.4 REQUIRED)
find_package(Ceres REQUIRED )
# find_package(Pangolin 0.4 REQUIRED)
# find_package(G2O REQUIRED)

# G2O
#set( G2O_LIBS 
#    g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Yaml:
find_library(Yaml_cpp_LIBRARY yaml-cpp /usr/local/include/yaml-cpp/)
message("YAML_CPP_INCLUDE_DIR: " ${Yaml_cpp_LIBRARY})

if(YAMLCPP_STATIC_LIBRARY)
    set(YAMLCPP_STATIC libyaml-cpp.a)
endif()

# find the yaml-cpp include directory
find_path(YAMLCPP_INCLUDE_DIR yaml-cpp/yaml.h
          PATH_SUFFIXES include
          PATHS
          ~/Library/Frameworks/yaml-cpp/include/
          /Library/Frameworks/yaml-cpp/include/
          /usr/local/include/
          /usr/include/
          /sw/yaml-cpp/         # Fink
          /opt/local/yaml-cpp/  # DarwinPorts
          /opt/csw/yaml-cpp/    # Blastwave
          /opt/yaml-cpp/
          ${YAMLCPP_DIR}/include/)

# find the yaml-cpp library
find_library(YAMLCPP_LIBRARY
             NAMES ${YAMLCPP_STATIC} yaml-cpp
             PATH_SUFFIXES lib64 lib
             PATHS ~/Library/Frameworks
                    /Library/Frameworks
                    /usr/local
                    /usr
                    /sw
                    /opt/local
                    /opt/csw
                    /opt
                    ${YAMLCPP_DIR}/lib)

# handle the QUIETLY and REQUIRED arguments and set YAMLCPP_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(YAMLCPP DEFAULT_MSG YAMLCPP_INCLUDE_DIR YAMLCPP_LIBRARY)
mark_as_advanced(YAMLCPP_INCLUDE_DIR YAMLCPP_LIBRARY)


include_directories(${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/include/experiments
                    # ${Pangolin_INCLUDE_DIRS}
                    # ${G2O_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_rgbd SHARED ${PROJECT_SOURCE_DIR}/src/rgbd_image.cpp
                                        ${PROJECT_SOURCE_DIR}/src/intrinsic.cpp
                                        ${PROJECT_SOURCE_DIR}/src/interpolation.cpp
                                        ${PROJECT_SOURCE_DIR}/src/point_selection.cpp)
target_link_libraries(${PROJECT_NAME}_rgbd PUBLIC ${OpenCV_LIBS} Eigen3::Eigen )

# add_library(${PROJECT_NAME}_solver SHARED ${PROJECT_SOURCE_DIR}/src/solver.cpp)
# target_link_libraries(${PROJECT_NAME}_solver PUBLIC ${OpenCV_LIBS}
                                                    # ${PROJECT_NAME}_rgbd
                                                    # ${G2O_LIBS})

add_library(${PROJECT_NAME}_TUM_loader SHARED ${PROJECT_SOURCE_DIR}/src/experiments/TUM_loader.cpp)
target_link_libraries(${PROJECT_NAME}_TUM_loader PUBLIC ${PROJECT_NAME}_rgbd
                                                        ${OpenCV_LIBS}) 

add_library(${PROJECT_NAME}_frontend_solver SHARED ${PROJECT_SOURCE_DIR}/src/FrontendSolver.cpp)
target_link_libraries( ${PROJECT_NAME}_frontend_solver PUBLIC Eigen3::Eigen 
                                                        ${PROJECT_NAME}_rgbd  
                                                        ${CERES_LIBRARIES} 
                                                        ${OpenCV_LIBS} )   

# add_executable(rgbd_test ${PROJECT_SOURCE_DIR}/src/experiments/rgbd_test.cpp)                
                              
# target_link_libraries(rgbd_test PUBLIC ${PROJECT_NAME}_rgbd
                                       # ${PROJECT_NAME}_TUM_loader
                                       # ${PROJECT_NAME}_point_selection
                                       # ${OpenCV_LIBS}
                                       # ${Yaml_cpp_LIBRARY})
                                       # ${Pangolin_LIBRARIES})
                                       # pango_display
                                       # pango_scene
                                       # /usr/local/lib/libpango_windowing.so)

add_executable( solver_test_bench ${PROJECT_SOURCE_DIR}/src/experiments/solver_tb.cpp)
target_link_libraries( solver_test_bench PUBLIC ${PROJECT_NAME}_rgbd ${OpenCV_LIBS} ${PROJECT_NAME}_frontend_solver ${CERES_LIBRARIES} ${Yaml_cpp_LIBRARY})  
add_executable( solver_wrapper ${PROJECT_SOURCE_DIR}/src/solver_wrapper.cpp)
target_link_libraries(solver_wrapper  ${PROJECT_NAME}_rgbd ${OpenCV_LIBS} ${PROJECT_NAME}_frontend_solver ${CERES_LIBRARIES}
                                    ${PROJECT_NAME}_TUM_loader ${Yaml_cpp_LIBRARY} )
#add_executable( solver_test ${PROJECT_SOURCE_DIR}/src/experiments/solver_test.cpp)
<<<<<<< HEAD
#target_link_libraries( solver_test PUBLIC ${PROJECT_NAME}_frontend_solver ${PROJECT_NAME}_rgbd )    

#find_package( G2O )
#include_directories( ${G2O_INCLUDE_DIRS} ) 

#include_directories( "/usr/include/eigen3" )

#set( G2O_LIBS 
#    g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension 
#)

#add_executable( direct_sparse ${PROJECT_SOURCE_DIR}/src/slambook_utils/direct_sparse.cpp )
#target_link_libraries( direct_sparse ${OpenCV_LIBS} ${G2O_LIBS} )
=======
#target_link_libraries( solver_test PUBLIC ${PROJECT_NAME}_frontend_solver ${PROJECT_NAME}_rgbd )    
>>>>>>> 35d494edc453a9a217cc3d6934928ebf825b0e57
