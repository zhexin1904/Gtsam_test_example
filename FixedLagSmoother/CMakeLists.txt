cmake_minimum_required(VERSION 3.10)

project(FixedLagSmootherExample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)


# set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu/") 
# find_package(Boost REQUIRED filesystem graph serialization thread date_time regex timer chrono)
# if(Boost_FOUND)
# message(STATUS "Found Boost: ${Boost_INCLUDE_DIRS} ${Boost_LIBRARIES}")
# endif()

set(GTSAM_DIR /home/jason/catkin_ws/ThirdParty/gtsam/build)
set(GTSAM_UNSTABLE_DIR /home/jason/catkin_ws/ThirdParty/gtsam/build)

find_package(GTSAM REQUIRED)
if(GTSAM_FOUND)
message(STATUS "Found GTSAM library: ${GTSAM_INCLUDE_DIR}\n")
endif()

find_package(GTSAM_UNSTABLE REQUIRED)
if(GTSAM_UNSTABLE_FOUND)
message(STATUS "Found GTSAM unstable library: ${GTSAM_UNSTABLE_INCLUDE_DIR}\n")
endif()

# find_package(Eigen3 REQUIRED)
# if(EIGEN3_FOUND)
# message(STATUS "Found Eigen3 library: ${EIGEN3_INCLUDE_DIR}\n")
# endif()
# include_directories("usr/include/eigen3")

include_directories(${CMAKE_SOURCE_DIR}/include
                    ${Boost_INCLUDE_DIRS}    
                    ${GTSAM_INCLUDE_DIR}
                    # ${EIGEN3_INCLUDE_DIR}
                    )



add_executable(FLSVIO_new_test_LM src/FLSVIO_new_test_LM.cpp)
target_link_libraries(FLSVIO_new_test_LM ${Boost_LIBRARIES} gtsam gtsam_unstable)

add_executable(FLSVIO_new_batch_test src/FLSVIO_new_batch_test.cpp)
target_link_libraries(FLSVIO_new_batch_test ${Boost_LIBRARIES} gtsam gtsam_unstable)

add_executable(FLSVIO_new_backup_1 src/FLSVIO_new_backup_1.cpp)
target_link_libraries(FLSVIO_new_backup_1 ${Boost_LIBRARIES} gtsam gtsam_unstable)

add_executable(FLSVIO_new_backup_2 src/FLSVIO_new_backup_2.cpp)
target_link_libraries(FLSVIO_new_backup_2 ${Boost_LIBRARIES} gtsam gtsam_unstable)

add_executable(FLSVIO_new_backup_3 src/FLSVIO_new_backup_3.cpp)
target_link_libraries(FLSVIO_new_backup_3 ${Boost_LIBRARIES} gtsam gtsam_unstable)

add_executable(FLSVIO_new_backup_4 src/FLSVIO_new_backup_4.cpp)
target_link_libraries(FLSVIO_new_backup_4 ${Boost_LIBRARIES} gtsam gtsam_unstable)

# add_executable(FLSVIO_new_backup_5 src/FLSVIO_new_backup_5.cpp)
# target_link_libraries(FLSVIO_new_backup_5 ${Boost_LIBRARIES} gtsam gtsam_unstable)

add_executable(FLSVIO_new_backup_6 src/FLSVIO_new_backup_6.cpp)
target_link_libraries(FLSVIO_new_backup_6 ${Boost_LIBRARIES} gtsam gtsam_unstable)

add_executable(FLSVIO_new_backup_7 src/FLSVIO_new_backup_7.cpp)
target_link_libraries(FLSVIO_new_backup_7 ${Boost_LIBRARIES} gtsam gtsam_unstable)