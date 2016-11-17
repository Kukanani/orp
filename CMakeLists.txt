cmake_minimum_required(VERSION 2.8.3)
project(orp)

set(GCC_COVERAGE_COMPILE_FLAGS "-Wno-enum-compare -ggdb")
#set(GCC_COVERAGE_COMPILE_FLAGS "-Wno-enum-compare -pg -ggdb")
set(GCC_COVERAGE_LINK_FLAGS    "")
#set(GCC_COVERAGE_LINK_FLAGS    "-pg -ggdb")

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    geometry_msgs
    message_generation
    tf
    tf_conversions
    eigen_conversions
    pcl_conversions
    interactive_markers
    cv_bridge
    obj_interface
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

########################################################
## Declare ROS messages, services, and configurations ##
########################################################

add_message_files(
    FILES
    ClassificationResult.msg
    WorldObjects.msg
)

add_service_files(
    FILES
    DataCollect.srv
    GetObjectPose.srv
    GetObjects.srv
    Recognition.srv
    RunData.srv
    SaveCloud.srv
    Segmentation.srv
)

generate_dynamic_reconfigure_options(
    cfg/CylinderClassifier.cfg
    cfg/HistogramSaver.cfg
    cfg/Recognizer.cfg
    cfg/Segmentation.cfg
)

generate_messages(
    DEPENDENCIES
    sensor_msgs std_msgs geometry_msgs obj_interface
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES orp
    CATKIN_DEPENDS message_runtime pcl_ros roscpp sensor_msgs std_msgs obj_interface
    DEPENDS pcl
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

####################################################################################################

add_library(orp
    src/classifier.cpp
    src/orp_utils.cpp
)
add_dependencies(orp ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp obj_interface_generate_messages)
target_link_libraries(orp ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkIO.so libvtkFiltering.so)

####################################################################################################

add_executable(recognizer src/recognizer.cpp)
add_dependencies(recognizer ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
target_link_libraries(recognizer ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp)

add_executable(segmentation src/segmentation.cpp)
add_dependencies(segmentation ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
target_link_libraries(segmentation ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )

####################################################################################################

add_executable(cph_classifier src/cph_classifier.cpp)
add_dependencies(cph_classifier ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
target_link_libraries(cph_classifier ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )

add_executable(sixdof_classifier src/sixdof_classifier.cpp)
add_dependencies(sixdof_classifier ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
target_link_libraries(sixdof_classifier ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )

add_executable(rgb_classifier src/rgb_classifier.cpp)
add_dependencies(rgb_classifier ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
target_link_libraries(rgb_classifier ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp ${OpenCV_LIBS})

add_executable(cylinder_classifier src/cylinder_classifier.cpp)
add_dependencies(cylinder_classifier ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
target_link_libraries(cylinder_classifier ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp ${OpenCV_LIBS})

add_executable(linemod_train src/linemod_train.cpp)
add_dependencies(linemod_train ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
target_link_libraries(linemod_train ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )

add_executable(train_linemod_template src/train_linemod_template.cpp)
add_dependencies(train_linemod_template ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
target_link_libraries(train_linemod_template ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )

####################################################################################################

add_executable(pan_data_collector src/pan_data_collector.cpp)
add_dependencies(pan_data_collector ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
target_link_libraries(pan_data_collector ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )

#add_executable(apc_data_collector src/apc_data_collector.cpp)
#add_dependencies(apc_data_collector ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
#target_link_libraries(apc_data_collector ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )

add_executable(histogram_saver src/histogram_saver.cpp)
add_dependencies(histogram_saver ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
target_link_libraries(histogram_saver ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )

#add_executable(virtual_train src/virtual_train.cpp)
#add_dependencies(virtual_train ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
#target_link_libraries(virtual_train ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp ${PCL_LIBRARIES} libvtkIO.so libvtkFiltering.so)

add_executable(vision_simulator src/vision_simulator.cpp)
add_dependencies(vision_simulator ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
target_link_libraries(vision_simulator ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )

# add_executable(point_cloud_batch_processor src/point_cloud_processor.cpp)
# add_dependencies(point_cloud_batch_processor ${orp_EXPORTED_TARGETS} orp_generate_messages_cpp)
# target_link_libraries(point_cloud_batch_processor ${Boost_LIBRARIES} ${catkin_LIBRARIES} orp )
