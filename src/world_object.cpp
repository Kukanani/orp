#include "core/world_object.h"
#include "core/world_object_manager.h"

int WorldObject::nextValidID = 0;

WorldObject::WorldObject(float colocationDist, WorldObjectManager* manager_, std::string type_, Eigen::Affine3d pose_, float probability_):
  type(manager_->getTypeByName(type_)),
  lastUpdated(ros::Time::now()),
  id(nextValidID++),
  stale(false),
  manager(manager_),
  colocationDistance(colocationDist),
  pose(pose_),
  probability(probability_)
{
} //WorldObject constructor

bool WorldObject::merge(WorldObject* other)
{
  return false;
} //merge

void WorldObject::refresh()
{
  setLastUpdated(ros::Time::now());
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

void WorldObject::setPose(tf::Pose pos)
{
  tf::poseTFToEigen(pos, pose);
} //setPose

void WorldObject::setPose(geometry_msgs::Pose pos)
{
  tf::Pose post;
  post.setOrigin(tf::Vector3(pos.position.x, pos.position.y, pos.position.z));
  post.setRotation(tf::Quaternion(pos.orientation.x, pos.orientation.y, pos.orientation.z,
    pos.orientation.w));
  setPose(post);
} //setPose

void WorldObject::setPosition(tf::Vector3 pos)
{
  Eigen::Affine3d toSet = getPose();
  toSet(0,3) = (double)pos.x();
  toSet(1,3) = (double)pos.y();
  toSet(2,3) = (double)pos.z();
} //setPosition

void WorldObject::setProbability(float probability_) {
  probability = probability_;
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
