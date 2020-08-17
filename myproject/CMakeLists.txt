
#1. 设定
cmake_minimum_required( VERSION 2.8 )
project( vo1 )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 2. 找到包
find_package( OpenCV  REQUIRED )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )
# Sophus
find_package( Sophus REQUIRED )
include_directories(
    ${OpenCV_INCLUDE_DIRS} 
    ${G2O_INCLUDE_DIRS}
#    ${CSPARSE_INCLUDE_DIR}
    "/usr/include/eigen3/"
        ${Sophus_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
        test.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
        ${CSPARSE_LIBRARY}
        ${Sophus_LIBRARIES}

        )

add_executable(test test_main.cpp)
target_link_libraries(test
        ${PROJECT_NAME})
add_executable(slam slam_main.cpp)
target_link_libraries(slam
        ${PROJECT_NAME})