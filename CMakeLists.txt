cmake_minimum_required(VERSION 2.8.3)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(orp)

set(GCC_COVERAGE_COMPILE_FLAGS "-Wno-enum-compare -Wno-deprecated -Wformat -Wformat-extra-args")
#set(GCC_COVERAGE_COMPILE_FLAGS "-Wno-enum-compare -pg")
set(GCC_COVERAGE_LINK_FLAGS    "")
#set(GCC_COVERAGE_LINK_FLAGS    "-pg")

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS pcl_ros roscpp sensor_msgs std_msgs geometry_msgs message_generation tf tf_conversions eigen_conversions interactive_markers)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

########################################################
## Declare ROS messages, services, and configurations ##
########################################################

add_message_files(
    FILES
    WorldObject.msg
    WorldObjects.msg
    ClassificationResult.msg
    DetectionSet.msg
)

add_service_files(
    FILES
    Recognition.srv
    RunData.srv
    DataCollect.srv
    SaveCloud.srv
    GetObjectPose.srv
    ChooseShelf.srv
    ReloadParams.srv
    Segmentation.srv
)

generate_dynamic_reconfigure_options(
    cfg/Recognizer.cfg
    cfg/Segmentation.cfg
    cfg/HistogramSaver.cfg
)

generate_messages(
    DEPENDENCIES
    sensor_msgs std_msgs geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime pcl_ros roscpp sensor_msgs std_msgs
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_library(orp
    src/world_object.cpp
    src/world_object_manager.cpp
    src/classifier.cpp
    src/orp_utils.cpp
)
####################################################################################################

add_executable(recognizer src/recognizer.cpp)
add_executable(segmentation src/segmentation.cpp)
add_executable(point_cloud_batch_processor src/point_cloud_processor.cpp)

add_executable(pan_data_collector src/pan_data_collector.cpp)
add_executable(apc_data_collector src/apc_data_collector.cpp)
add_executable(histogram_saver src/histogram_saver.cpp)
add_executable(virtual_train src/virtual_train.cpp)

add_executable(cph_classifier src/cph_classifier.cpp)
#add_executable(vfh_classifier src/vfh_classifier.cpp)
#add_executable(cvfh_classifier src/cvfh_classifier.cpp)
#add_executable(icp_classifier src/icp_classifier.cpp)
add_executable(sixdof_classifier src/sixdof_classifier.cpp)
add_executable(rgb_classifier src/rgb_classifier.cpp)
add_executable(vision_simulator src/vision_simulator.cpp)

####################################################################################################


add_dependencies(recognizer ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
add_dependencies(segmentation ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)

add_dependencies(pan_data_collector ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
add_dependencies(apc_data_collector ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
add_dependencies(point_cloud_batch_processor ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
add_dependencies(histogram_saver ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
add_dependencies(virtual_train ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)

add_dependencies(cph_classifier ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
#add_dependencies(vfh_classifier ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
#add_dependencies(cvfh_classifier ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
#add_dependencies(icp_classifier ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
add_dependencies(sixdof_classifier ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
add_dependencies(rgb_classifier ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
add_dependencies(vision_simulator ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
add_dependencies(orp ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)

####################################################################################################


target_link_libraries(orp ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkIO.so libvtkFiltering.so)

target_link_libraries(recognizer ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )
target_link_libraries(segmentation ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )

target_link_libraries(pan_data_collector ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )
target_link_libraries(apc_data_collector ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )
target_link_libraries(point_cloud_batch_processor ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )
target_link_libraries(histogram_saver ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )
target_link_libraries(virtual_train ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp ${PCL_LIBRARIES} libvtkIO.so libvtkFiltering.so)

target_link_libraries(cph_classifier ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )
#target_link_libraries(vfh_classifier ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )
#target_link_libraries(cvfh_classifier ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )
#target_link_libraries(icp_classifier ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )
target_link_libraries(sixdof_classifier ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )
target_link_libraries(rgb_classifier ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp ${OpenCV_LIBS})
target_link_libraries(vision_simulator ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )