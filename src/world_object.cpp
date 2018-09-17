// Copyright (c) 2016, Adam Allevato
// Copyright (c) 2017, The University of Texas at Austin
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "orp/core/world_object.h"

#include <tf/tf.h>

#include <eigen_conversions/eigen_msg.h>

#include "orp/core/world_object_manager.h"
#include "orp/core/grasp_generator.h"

int WorldObject::nextValidID = 0;
int WorldObjectType::nextValidID = 0;

const float WorldObject::MEASUREMENT_COVARIANCE = 0.001f; //1mm measurement covariance

WorldObject WorldObject::createFromMessage(
  WorldObjectManager* manager_, orp::WorldObject message)
{
  Eigen::Affine3d eigenPose;
  tf::poseMsgToEigen(message.pose.pose, eigenPose);
  WorldObject obj(message.colocationDist, manager_, message.label,
    message.pose.header.frame_id, eigenPose, message.probability);
  obj.setCloud(message.cloud);
  return obj;
}

WorldObject::WorldObject(float colocationDist, WorldObjectManager* manager_,
                         std::string type_, std::string _frame,
                         const Eigen::Affine3d& pose_, float probability_):
  type(manager_->getTypeByName(type_)),
  lastUpdated(ros::Time::now()),
  id(nextValidID),
  stale(false),
  frame(_frame),
  manager(manager_),
  colocationDistance(colocationDist),
  pose(pose_),
  probability(probability_),
  showGrasps(false)
{
  nextValidID += MARKERS_PER_OBJECT;

  covariance.setIdentity();
  Q.setIdentity();
  Q *= MEASUREMENT_COVARIANCE;

  setupObjectMarker();
  setupLabelMarker();

  updateMarkers();

  tf::Pose stubAdjustmentPose = type.generateOffsetPose();

  Eigen::Affine3d eigStubAdjustment;
  tf::poseTFToEigen(stubAdjustmentPose, eigStubAdjustment);

  pose = pose * eigStubAdjustment;
} //WorldObject constructor

std::string WorldObject::getDebugRepresentation()
{
  std::string staleString = "";
  if(isStale())
  {
    staleString = " [STALE] ";
  }
  return "WorldObject(" + staleString + "#" + std::to_string(id) + ", " +
         getType().getName() + ")";
}

bool WorldObject::merge(WorldObjectPtr other)
{
  // ROS_DEBUG_STREAM("my type is " << getType().getName() << ", my probability is " << getProbability() <<
  //          "their type is " << other->getType().getName() << ", their probability is " << other->getProbability());
  if(probability > other->getProbability())
  {
    // I'm better. Stay as I am
    return false;
  }
  else {
    // the other one is better, steal its properties
    colocationDistance = other->getColocationDistance();
    setType(other->getType());
    id = id;
    stale = false;
    manager = other->manager;
    //perform Kalman Filtering
    setPose(other->getPose());
    //this is where we would perform Bayesian filtering on type probabilities
    probability = other->getProbability();
    setLastUpdated(ros::Time::now());
    setCloud(other->getCloud());

    // request yourself to be destroyed in the future
    // merge successful
    return true;
  }
} //merge


void WorldObject::calculateGrasps() {
  GraspGenerator g(ROBOTIQ_S_MODEL); //TODO magic
  grasps = g.createGrasps(*this, APPROACH_DIST);
}

bool WorldObject::isColocatedWith(WorldObjectPtr other) {
  return distanceTo(other) <=
    colocationDistance;
  return false;
}

float WorldObject::distanceTo(WorldObjectPtr other) {
  return (getPose().translation()-other->getPose().translation()).norm();
}

void WorldObject::setType(WorldObjectType newType) {
  type = newType;
  setupObjectMarker();
  setupLabelMarker();
}

void WorldObject::setupObjectMarker() {
  objectMarker = type.getStub();
  objectMarker.header.frame_id = frame;
  objectMarker.header.stamp = ros::Time::now();
  objectMarker.id = id;
  objectMarker.action = visualization_msgs::Marker::ADD;
  objectMarker.color.a = 0.5;
}

void WorldObject::setupLabelMarker() {
  labelMarker.header.frame_id = frame;
  labelMarker.header.stamp = ros::Time::now();
  labelMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  labelMarker.action = visualization_msgs::Marker::ADD;
  // 1 id is reserved for the object marker
  labelMarker.id = id + 1;
  labelMarker.text = type.getName();
  labelMarker.scale.z = 0.03f;
  labelMarker.color.r = 1.0f;
  labelMarker.color.g = 1.0f;
  labelMarker.color.b = 1.0f;
  labelMarker.color.a = 1.0f;
}

void WorldObject::setStale(bool s) {
  stale = s;
}

void WorldObject::refresh()
{
  setLastUpdated(ros::Time::now());
  stale = false;
} //refresh

void WorldObject::setLastUpdated(ros::Time time)
{
  lastUpdated = time;
} //refresh

void WorldObject::setProbability(float probability_) {
  probability = probability_;
}

void WorldObject::setPose(const Eigen::Affine3d& pos, bool hard) {
  if(hard) {
    pose = pos; //no filter!
  }
  else {
    setPoseKalman(pos);
  }

  //tPose.setRotation(tPose.getRotation().normalize());
  //update the marker
  tf::poseEigenToMsg(Eigen::Affine3d(pose), objectMarker.pose);
  tf::poseEigenToMsg(Eigen::Affine3d(pose), labelMarker.pose);
  updateMarkers();
}

void WorldObject::setPoseKalman(const Eigen::Affine3d& pos) {
  Eigen::Matrix<double, KALMAN_SIZE,KALMAN_SIZE> K =
    covariance * (covariance + Q).inverse();

  Eigen::Matrix<double, KALMAN_SIZE,1> difference;
  for(int i=0; i<3; ++i) {
    difference(i) = pos(i,3)-pose(i,3);
  }
  Eigen::Matrix<double, KALMAN_SIZE,1> result = K*difference;

  pose(0,3) += result(0);
  pose(1,3) += result(1);
  pose(2,3) += result(2);

  pose.linear() = pos.linear();

  Eigen::Matrix<double, KALMAN_SIZE,KALMAN_SIZE> I;
  I.setIdentity();
  covariance = (I-K)*covariance;

  setLastUpdated(ros::Time::now());
}

void WorldObject::updateMarkers() {
  tf::poseTFToMsg(getPoseTf(), objectMarker.pose);
  tf::poseTFToMsg(getPoseTf(), labelMarker.pose);
  labelMarker.pose.position.z += 0.03f;
}

const Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign>& WorldObject::getPose() {
  return pose;
}

WorldObjectType WorldObject::getType() {
  //ensure that the frame is correct for the marker before passing along
  type.setFrame(this->getFrame());
  return type;
}

std::vector<visualization_msgs::Marker> WorldObject::getMarkers() {
  std::vector<visualization_msgs::Marker> markers;
  if(isStale()) {
    //DELETE all the markers alloted to this object
    for(int i=0; i < MARKERS_PER_OBJECT; ++i) {
      visualization_msgs::Marker newMarker;
      newMarker.id = getID() + i;
      newMarker.action = visualization_msgs::Marker::DELETE;
      markers.push_back(newMarker);
    }
  }
  else {
    markers.push_back(objectMarker);
    markers.push_back(labelMarker);

    if(getShowGrasps()) {
      calculateGrasps(); /// setup for the arrows
      visualization_msgs::Marker arrow;
      arrow.id = getID() + NON_GRASP_MARKER_COUNT;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.color.a = 1.0;
      arrow.color.r = 0.5;
      arrow.header.frame_id = getFrame();
      arrow.scale.x = 0.01; arrow.scale.y = 0.02; arrow.scale.z=0.03;

      for(auto grasp = grasps.begin(); grasp != grasps.end(); ++grasp) {
        arrow.points.clear();
        geometry_msgs::Point point;
        tf::pointTFToMsg(grasp->approachPose.getOrigin(), point);
        arrow.points.push_back(point);
        tf::pointTFToMsg(grasp->graspPose.getOrigin(), point);
        arrow.points.push_back(point);
        markers.push_back(arrow);
        arrow.id++;
      }
    }
  }

  return markers;
}

float WorldObject::getProbability()
{
  return probability;
} //getProbability

tf::Pose WorldObject::getPoseTf() {
  tf::poseEigenToTF(Eigen::Affine3d(getPose()), tPose);
  return tPose;
}

tf::Stamped<tf::Pose> WorldObject::getPoseTfStamped() {
  return tf::Stamped<tf::Pose>(getPoseTf(), ros::Time::now(), frame);
}
