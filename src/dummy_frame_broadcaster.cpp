#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

tf::Transform transform;
tf::Quaternion q;
std::string source;
std::string destination;

void poseCallback(const ros::TimerEvent& msg){
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform, msg.current_real, source, destination));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "dummy_frame_broadcaster");
  if (argc != 9) {
    ROS_ERROR("syntax: dummy_frame_broadcaster source destination x y z roll pitch yaw");
    return -1;
  };

  source = argv[1];
  destination = argv[2];

  transform.setOrigin( tf::Vector3(atof(argv[3]), atof(argv[4]), atof(argv[5])));

  q.setRPY(atof(argv[6]), atof(argv[7]), atof(argv[8]));
  transform.setRotation(q);

  ros::NodeHandle node;
  ros::Timer timer = node.createTimer(ros::Duration(0.01), poseCallback);

  ros::spin();
  return 0;
};