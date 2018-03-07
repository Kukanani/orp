cmake_minimum_required(VERSION 2.8.3)
project(orp)

# Use C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    eigen_conversions
    geometry_msgs
    image_transport
    interactive_markers
    message_generation
    message_runtime
    opencv_apps
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    vision_msgs
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

########################################################
## Declare ROS messages, services, and configurations ##
########################################################

add_message_files(
    FILES
    ClassificationResult.msg
    WorldObject.msg
    WorldObjects.msg
    Region.msg
)

add_service_files(
    FILES
    DataCollect.srv
    GetObjectPose.srv
    GetObjects.srv
    Recognition.srv
  	#RunData.srv
    SaveCloud.srv
    Segmentation.srv
    Monitor.srv
)

generate_dynamic_reconfigure_options(
    cfg/CylinderClassifier.cfg
    cfg/HistogramSaver.cfg
    cfg/Recognizer.cfg
    cfg/Segmentation.cfg
)

generate_messages(
    DEPENDENCIES
    sensor_msgs std_msgs geometry_msgs vision_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES orp
    CATKIN_DEPENDS pcl_ros sensor_msgs std_msgs vision_msgs
    # DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

####################################################################################################

add_library(orp
    src/classifier.cpp
    src/nn_classifier.cpp
    src/classifier2d.cpp
    src/classifier3d.cpp
    src/orp_utils.cpp
    src/world_object.cpp
    src/world_object_manager.cpp
    src/grasp_generator.cpp
)
add_dependencies(orp ${orp_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(orp ${catkin_LIBRARIES} ${PCL_LIBRARIES})

####################################################################################################

add_executable(recognizer src/recognizer.cpp)
add_dependencies(recognizer ${orp_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(recognizer ${catkin_LIBRARIES} orp)

add_executable(segmentation src/segmentation.cpp)
add_dependencies(segmentation ${orp_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(segmentation ${catkin_LIBRARIES} orp )

####################################################################################################

add_executable(sixdof_classifier src/sixdof_classifier.cpp)
add_dependencies(sixdof_classifier ${orp_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(sixdof_classifier ${catkin_LIBRARIES} orp )

add_executable(rgb_classifier src/rgb_classifier.cpp)
add_dependencies(rgb_classifier ${orp_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(rgb_classifier ${catkin_LIBRARIES} orp ${OpenCV_LIBS})

add_executable(basic_classifier src/basic_classifier.cpp)
add_dependencies(basic_classifier ${orp_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(basic_classifier ${catkin_LIBRARIES} orp ${OpenCV_LIBS})

####################################################################################################

add_executable(vision_simulator src/vision_simulator.cpp)
add_dependencies(vision_simulator ${orp_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(vision_simulator ${catkin_LIBRARIES} orp )
