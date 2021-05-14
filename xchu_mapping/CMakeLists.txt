cmake_minimum_required(VERSION 2.8.3)
project(xchu_mapping)

#add_compile_options(-std=c++17 -O0)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenMP REQUIRED) #链接ndt_cpu库 get_max_thread_num
if (OPENMP_FOUND)
    message(STATUS "OPENMP FOUND")
    set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
    set(OpenMP_LIBS gomp)
endif ()

find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        nav_msgs
        pcl_conversions
        sensor_msgs
        tf
        std_msgs
        geometry_msgs
        message_generation
        )

add_message_files(FILES
        FloorCoeffs.msg
        LoopInfo.msg
        )

link_directories(lib ${PCL_LIBRARY_DIRS}  ${GTSAM_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS nav_msgs roscpp tf sensor_msgs std_msgs geometry_msgs pcl_conversions
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}

)
message("PCL Libraries include: ${PCL_LIBRARIES}")
message("PCL version: ${PCL_VERSION}")

add_executable(odom_node src/odom_node.cpp)
target_link_libraries(odom_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBS} ndt_cpu ndt_omp)

add_executable(filter_node
        src/filter_node.cpp
        )
target_link_libraries(filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pgo_node
        src/pgo_node.cpp
        include/scancontext/Scancontext.cpp
        )
target_compile_options(pgo_node
        PRIVATE ${OpenMP_CXX_FLAGS}
        )
target_link_libraries(pgo_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenMP_CXX_FLAGS}
        ${OpenCV_LIBS}
        gtsam
        )
