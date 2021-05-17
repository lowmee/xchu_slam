cmake_minimum_required(VERSION 2.8.3)
project(xchu_mapping)

message("Compiler version is: ${CMAKE_SYSTEM_VERSION}")
message("System version is: ${CMAKE_SYSTEM_NAME}")
message("Architecture is: ${CMAKE_SYSTEM_PROCESSOR}")


if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Darwin")
    set(CMAKE_CXX_COMPILER clang++)
    set(CMAKE_C_COMPILER clang)
    set(MACOSX TRUE)
endif ()

#add_compile_options(-std=c++17 -O0)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if (NOT MACOSX)
    find_package(OpenMP REQUIRED) #链接ndt_cpu库 get_max_thread_num
    if (OPENMP_FOUND)
        message(STATUS "OPENMP FOUND")
        set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
        set(OpenMP_LIBS gomp)
        set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
    endif ()
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -ffast-math -Xclang -fopenmp -pthread")
    if (EXISTS /usr/local/lib/libomp.dylib)
        set(OMP_LIB /usr/local/lib/libomp.dylib)
    elseif (EXISTS /usr/local/lib/libomp.a)
        set(OMP_LIB /usr/local/lib/libomp.a)
    elseif (EXISTS /opt/local/lib/libomp.a)
        set(OMP_LIB /opt/local/lib/libomp.a)
        include_directories(/opt/local/include /opt/local/include/libomp)
    elseif (EXISTS /opt/local/lib/libomp.dylib)
        set(OMP_LIB /opt/local/lib/libomp.dylib)
        include_directories(/opt/local/include /opt/local/include/libomp)
    elseif (EXISTS /usr/lib/libomp.a)
        set(OMP_LIB /usr/lib/libomp.a)
    elseif (EXISTS /usr/lib/libomp.dylib)
        set(OMP_LIB /usr/lib/libomp.dylib)
    else ()
        message(FATAL_ERROR "Mac OS Detected but No OpenMP Library Found!")
    endif ()
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
        cv_bridge
        image_transport
        #        ndt_cpu
        #        ndt_omp
        )

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

add_message_files(FILES
        FloorCoeffs.msg
        LoopInfo.msg
        )

link_directories(lib ${PCL_LIBRARY_DIRS} ${GTSAM_LIBRARY_DIRS})
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
target_link_libraries(odom_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBS} ${OMP_LIB} ndt_cpu ndt_omp)

add_executable(filter_node
        src/filter_node.cpp
        )
target_link_libraries(filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pgo_node
        src/pgo_node.cpp
        include/scancontext/Scancontext.cpp
        include/isc/ISCGeneration.cpp
        )
if (NOT MACOSX)
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
else ()
    target_link_libraries(pgo_node
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${OMP_LIB}
            ndt_cpu
            ndt_omp
            gtsam
            )
endif ()

# g2o
add_executable(pgo_g2o_node
        src/pgo_g2o_node.cpp
#        src/g2o/graph_slam.cpp
#        src/g2o/robust_kernel_io.cpp
        include/scancontext/Scancontext.cpp
        )
target_link_libraries(pgo_g2o_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${G2O_TYPES_DATA}
        ${G2O_CORE_LIBRARY}
#        ${G2O_STUFF_LIBRARY}
#        ${G2O_SOLVER_PCG}
#        ${G2O_SOLVER_CSPARSE}
#        ${G2O_SOLVER_CHOLMOD}
#        ${G2O_TYPES_SLAM3D}
#        ${G2O_TYPES_SLAM3D_ADDONS}
        ${OpenCV_LIBS}
        #        ${CERES_LIBRARIES}
#        ${OpenCV_LIBRARIES}
        )