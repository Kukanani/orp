#include "core/world_object.h"
#include "core/world_object_manager.h"

#include <tf/tf.h>
#include <X11/Xlib.h>

#include <eigen_conversions/eigen_msg.h>

int WorldObject::nextValidID = 0;

WorldObject::WorldObject(float colocationDist, WorldObjectManager* manager_, std::string type_, std::string frame, Eigen::Affine3d pose_, float probability_):
  type(manager_->getTypeByName(type_)),
  lastUpdated(ros::Time::now()),
  id(nextValidID++),
  stale(false),
  manager(manager_),
  colocationDistance(colocationDist),
  pose(pose_),
  probability(probability_),
  objectMarker(type.stub)
{
  covariance.setIdentity();
  Q.setIdentity();
  Q *= 0.001; // 1cm measurement covariance
  
  objectMarker.header.frame_id = frame;
  objectMarker.header.stamp       = ros::Time::now();
  objectMarker.id                 = id;
  objectMarker.action             = visualization_msgs::Marker::ADD;
  tf::poseTFToMsg(getPoseTf(), objectMarker.pose);
  objectMarker.color.a = 0.5;
  
  labelMarker.header.frame_id    = frame;
  labelMarker.header.stamp       = ros::Time::now();
  labelMarker.type               = visualization_msgs::Marker::TEXT_VIEW_FACING;
  labelMarker.action             = visualization_msgs::Marker::ADD;
  labelMarker.id                 = id;
  labelMarker.text               = type.name;
  tf::poseTFToMsg(getPoseTf(), labelMarker.pose);
  labelMarker.pose.position.z += 0.03f;
  labelMarker.scale.z = 0.1f;
  labelMarker.color.r = 1.0f;
  labelMarker.color.g = 1.0f;
  labelMarker.color.b = 1.0f;
  labelMarker.color.a = 1.0f;
  
   tf::Pose stubAdjustmentPose = tf::Pose(tf::createQuaternionFromRPY(
    type.offset.roll,
    type.offset.pitch,
    type.offset.yaw
  ));
  
  Eigen::Affine3d eigStubAdjustment;
  tf::poseTFToEigen(stubAdjustmentPose, eigStubAdjustment);
  
  pose = pose * eigStubAdjustment;
} //WorldObject constructor

bool WorldObject::merge(WorldObjectPtr other)
{
  colocationDistance = other->getColocationDist();
  type = other->getType();
  id = other->getID();
  stale = false;
  manager = other->manager;
  setPose(other->getPose()); //perform Kalman Filtering
  probability = other->getProbability(); //this is where we would perform Bayesian filtering on type probabilities
  setLastUpdated(ros::Time::now());
  
  return true;
} //merge

bool WorldObject::isColocatedWith(WorldObjectPtr other) {
  return ((pose.translation()-other->getPose().translation()).norm() <= colocationDistance);
  return false;
}

void WorldObject::setStale(bool s) {
  stale = s;
  if(s) {
    objectMarker.action = visualization_msgs::Marker::DELETE;
    labelMarker.action = visualization_msgs::Marker::DELETE;
  } else {
    objectMarker.action = visualization_msgs::Marker::ADD;
    labelMarker.action = visualization_msgs::Marker::ADD;
  }
}
  
void WorldObject::refresh()
{
  setLastUpdated(ros::Time::now());
  stale = false;
} //refresh

void WorldObject::debugPrint()
{
  //debug print
  /*for(TypeMap::iterator it = types.begin(); it != types.end(); ++it) {
    Eigen::Matrix3d rotMat = it->second.pose.linear();
    Eigen::Quaterniond rotQ; rotQ = rotMat;
    ROS_INFO("world object: (%f) has %s position %f, %f, %f", it->second.prob, it->first.name.c_str(), it->second.pose(0,3), it->second.pose(1,3), it->second.pose(3,3));
       ROS_INFO("pose for %s has quat %f, %f, %f, %f", it->first.name.c_str(), rotQ.x(), rotQ.y(), rotQ.z(), rotQ.w());
  }*/
} //getInfoString

void WorldObject::setLastUpdated(ros::Time time)
{
  lastUpdated = time;
} //refresh

// void WorldObject::setPose(tf::Pose pos)
// {
//   tf::poseTFToEigen(pos, pose);
// } //setPose

// void WorldObject::setPose(geometry_msgs::Pose pos)
// {
//   tf::Pose post;
//   post.setOrigin(tf::Vector3(pos.position.x, pos.position.y, pos.position.z));
//   post.setRotation(tf::Quaternion(pos.orientation.x, pos.orientation.y, pos.orientation.z,
//     pos.orientation.w));
//   setPose(post);
// } //setPose

// void WorldObject::setPosition(tf::Vector3 pos)
// {
//   Eigen::Affine3d toSet = getPose();
//   toSet(0,3) = (double)pos.x();
//   toSet(1,3) = (double)pos.y();
//   toSet(2,3) = (double)pos.z();
// } //setPosition

void WorldObject::setProbability(float probability_) {
  probability = probability_;
}

void WorldObject::setPose( Eigen::Affine3d pos, bool hard) {  
  Eigen::Quaterniond origRotQ(pose.rotation());
  Eigen::Quaterniond newRotQ(pos.rotation());
  
  //KALMAN
  Eigen::Matrix<double, KALMAN_SIZE,KALMAN_SIZE> K = covariance * ( (covariance + Q).inverse() );
  
  Eigen::Matrix<double, KALMAN_SIZE,1> difference;
  for(int i=0; i<3; ++i) {
    difference(i) = pos(i,3)-pose(i,3);
  }
  //difference(3,3) = newRotQ.x()-origRotQ.x();
  //difference(4,4) = newRotQ.y()-origRotQ.y();
  //difference(5,5) = newRotQ.z()-origRotQ.z();
  //difference(6,6) = newRotQ.w()-origRotQ.w();
  Eigen::Matrix<double, KALMAN_SIZE,1> result = K*difference;
  
  //ROS_INFO_STREAM("old pose: " <<std::endl<< pos.matrix());
  ROS_INFO_STREAM("difference: " <<std::endl<< difference);
  ROS_INFO_STREAM("covariance: " <<std::endl<< covariance);
  ROS_INFO_STREAM("k: " <<std::endl<< K);
  //ROS_INFO_STREAM("result: " <<std::endl<< result);
 
  
  //Eigen::Quaterniond resultRotQ = Eigen::Quaterniond(result(3,3), result(4,4), result(5,5), result(6,6)).normalized();
  //Eigen::Matrix3d resultRotMat; resultRotMat = resultRotQ;
  
  std::cout << pose(0,3) << " " << pose(1,3) << " " << pose(2,3) << std::endl;
  pose(0,3) += result(0);
  pose(1,3) += result(1);
  pose(2,3) += result(2);
  //pose.linear() += resultRotMat;
  
  //ROS_INFO_STREAM("new pose: " <<std::endl<< pos.matrix());
  
  std::cout << pose(0,3) << " " << pose(1,3) << " " << pose(2,3) << std::endl;
  Eigen::Matrix<double, KALMAN_SIZE,KALMAN_SIZE> I;
  I.setIdentity();
  covariance = (I-K)*covariance;
  
  setLastUpdated(ros::Time::now());
  
  //pose = pos; //no kalman!
  
  
  tf::poseEigenToMsg(pose, objectMarker.pose);
}

Eigen::Affine3d WorldObject::getPose() {
  return pose;
}

float WorldObject::getProbability()
{
  return probability;
} //getProbability

tf::Pose WorldObject::getPoseTf() {
  tf::Pose tPose;
  tf::poseEigenToTF(getPose(), tPose);
  return tPose;
}
