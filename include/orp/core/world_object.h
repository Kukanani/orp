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

#ifndef WORLD_OBJECT_H
#define WORLD_OBJECT_H

//Core includes
#include <string>
#include <algorithm>
#include <map>
#include <cstdlib>
#include <memory>

//3rd party libraries
#include <boost/shared_ptr.hpp>

//ROS includes
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

//ROS Messages
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include <orp/WorldObject.h>

#include "grasp.h"

//helpful typedefs and forward declarations
class WorldObject;
struct WorldObjectType;
class WorldObjectManager;
typedef boost::shared_ptr< WorldObject > WorldObjectPtr;
typedef std::list< WorldObjectPtr > WorldObjectList;

/**
 * roll, pitch and yaw (orientation) struct
 */
struct RPY {
  double roll; //x axis
  double pitch; //y axis
  double yaw; //z axis

  RPY() : roll(0), pitch(0), yaw(0) {};
};

enum ObjectShape {
  BOX,
  CYLINDER,
  BLOB,
  FLAT,
  OTHER
};

/**
 * @brief Represents a type of object that can be found in the world.
 *
 * Objects that are of the same WorldObjectType are functionally
 * identical in the real world (same color, size, weight, etc).
 *
 * If objects need to be distinguished from each other, they need to be
 * of different WorldObjectType.
 */
struct WorldObjectType {
protected:
  std::string name; ///No two WorldObjectTypes should share a name.
  visualization_msgs::Marker stub;
  RPY offset;
  ObjectShape shape;

  void setupStub() {
    stub.action = visualization_msgs::Marker::ADD;
    stub.color.a = 0.5; //make it visible
  };
public:
  WorldObjectType(std::string _name): shape(OTHER), name(_name), stub(), offset() {
    setupStub();
    if(name == "unknown") {
      setShape(BLOB);
      stub.scale.x = 0.1;
      stub.scale.y = 0.1;
      stub.scale.z = 0.1;

      offset.roll = 0.0;
      offset.pitch = 0.0;
      offset.yaw = 0.0;
    }
  };


  void setSize(float x, float y, float z) {
    stub.scale.x = x; stub.scale.y = y; stub.scale.z = z;
  };
  void setColor(float r, float g, float b) {
    stub.color.r = r; stub.color.g = g; stub.color.b = b;
  };
  void setOffset(float roll, float pitch, float yaw) {
    offset.roll = roll; offset.pitch = pitch; offset.yaw = yaw;
  };
  void setShape(ObjectShape _shape) {
    shape = _shape;
    switch(shape) {
      case BOX:
        stub.type = visualization_msgs::Marker::CUBE;
        break;
      case CYLINDER:
        stub.type = visualization_msgs::Marker::CYLINDER;
        break;
      case BLOB:
        stub.type = visualization_msgs::Marker::SPHERE;
        break;
      case FLAT:
        stub.type = visualization_msgs::Marker::CUBE;
        break;
      case OTHER:
      default:
        stub.type = visualization_msgs::Marker::SPHERE;
        break;
    }
  };

  tf::Pose generateOffsetPose() {
    return tf::Pose(tf::createQuaternionFromRPY(
      offset.roll, offset.pitch, offset.yaw
    ));
  };

  void setFrame(std::string frameName) {
    stub.header.frame_id = frameName;
  };

  std::string getName() { return name; };
  visualization_msgs::Marker getStub() { return stub; };
  ObjectShape getShape() { return shape; };

  inline bool operator <(const WorldObjectType& wot1) const {
    //ROS_INFO("%s vs %s", name.c_str(), wot1.name.c_str());
    return name < wot1.name;
  };

  inline bool operator ==(const WorldObjectType& wot1) const {
    //ROS_INFO("%s vs %s", name.c_str(), wot1.name.c_str());
    return name == wot1.name;
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; //WorldObjectType

/**
 * @brief Represents an object in the world as a probabilistic model.
 *
 * A WorldObject has a chance of being any of a number of different WorldObjectType.
 * It stores probabilities that it is various types.
 *
 * A WorldObject does not have to have a probability for each type. If no probability
 * exists, it is assumed to be 0.
 */

const float APPROACH_DIST = 0.04f;

class WorldObject {
private:
  static int nextValidID;     /// ncremented on new object creation.

  static const int MARKERS_PER_OBJECT = 10;
  static const int NON_GRASP_MARKER_COUNT = 2; //start grasp arrow marker IDs after this many. One reserved for object, one for name
  static const int KALMAN_SIZE = 3;
  static const float MEASUREMENT_COVARIANCE; //set in cpp file

  void setType(WorldObjectType newType);
  void setupObjectMarker();
  void setupLabelMarker();

protected:
  int id;                     /// This object's unique ID. This is mainly used for ROS markers.
  float probability;          /// Certainty of classification/pose
  std::string frame;          /// The TF frame name that this object lives in.
  Eigen::Affine3d pose;       /// position and orientation of this object (center of object)

  ros::Time lastUpdated;      /// When this object was last updated by a recognition event
  bool stale;                 /// If the object is stale, that means it needs to be deleted.
  float colocationDistance;   /// objects closer than this distance (in meters) are assumed to be the same one.

  WorldObjectManager* manager;/// Manages global list of types.
  WorldObjectType type;       /// The type of this object

  bool showGrasps;            /// If true, return grasp markers in getMarkers()
  std::vector<Grasp> grasps;  /// Possible grasps for this object.
  sensor_msgs::PointCloud2 cloud;


  std::vector<std::string> fullSensorModel; //list of all detectable items
  std::vector<std::string> subSensorModel;  //list of items to be detected

  visualization_msgs::Marker objectMarker; //visual representation of this
  visualization_msgs::Marker labelMarker; //the object's name

  Eigen::Matrix<double, KALMAN_SIZE,KALMAN_SIZE> covariance; //for Kalman filter
  Eigen::Matrix<double, KALMAN_SIZE,KALMAN_SIZE> Q; // for Kalman filter
  void setPoseKalman(Eigen::Affine3d pos);
public:
  ///Create a new WorldObject from a WorldObject message.
  static WorldObject createFromMessage(WorldObjectManager* manager_, orp::WorldObject message);

  /**
   * Basic constructor.
   */
  WorldObject(float colocationDist, WorldObjectManager* manager_, std::string type_, std::string frame, Eigen::Affine3d pose_, float probability_);

  /**
   * Merge this object's data with another object. It's assumed that the other is newer than this object.
   *
   * @arg other the WorldObject to merge with
   */
  virtual bool merge(WorldObjectPtr other);

  /**
   * Refreshes this object's last updated time to be ros::Time::now().
   *
   * Note that calling this repeatedly will invoke a lot of calls to ros::Time::now().
   * Consider using setLastUpdated() instead.
   */
  void refresh();
  void updateMarkers(); /// Set the position of the associated markers to match this object's pose.
  void calculateGrasps(); /// Use GraspGenerator to get a set of grasps for this object.

  /**
   * Set the object's pose.
   * If the "hard" argument is true, then no filtering will be performed and the object's pose will
   * exactly match the "pos" argument after this function ends.
   * If "hard" is false, then the function will call setPoseKalman() to update the pose statistically.
   */
  void setPose( Eigen::Affine3d pos, bool hard = false);
  void setProbability(float probability_);
  void setStale(bool s); ///set this object as stale to kill it on the next update
  void setLastUpdated(ros::Time time); ///Set the object's last updated time
  void setShowGrasps(bool show) { showGrasps = show; };

  void setCloud(sensor_msgs::PointCloud2 clou) {
    cloud = clou;
  }
  sensor_msgs::PointCloud2 getCloud() { return cloud; };

  std::vector<visualization_msgs::Marker> getMarkers(); /// Get whatever markers are appropriate for the object in it's current state.
  float getProbability(); ///Get the classification certainty.
  tf::Pose getPoseTf();      ///Get the object pose
  tf::Stamped<tf::Pose>  getPoseTfStamped(); /// Get the object pose including frame information
  Eigen::Affine3d getPose(); ///Get the object pose
  WorldObjectType getType();
  inline ros::Time getLastUpdated()                   { return lastUpdated;        }; /// @return the ROS time when this object was last updated/recognized.
  inline int getID()                                  { return id;                 }; ///Get the object's autogenerated ID number (unique).
  inline std::string getFrame()                       { return frame;              }; ///Get the name of the TF frame that this object's poses are relative to.
  inline float getX()                                 { return getPose()(0,3);     }; /// Get the X-component of pose position
  inline float getY()                                 { return getPose()(1,3);     }; /// Get the Y-component of pose position
  inline float getZ()                                 { return getPose()(2,3);     }; /// Get the Z-component of pose position
  inline float getColocationDistance()                { return colocationDistance; }; /// Objects closer than this distance should be merged with this object.
  inline std::vector<Grasp> getGrasps()               { return grasps;             };

  inline bool isStale()                               { return stale;              }; /// If true, this object should be removed.
  bool isColocatedWith(WorldObjectPtr other);
  inline bool getShowGrasps()                         { return showGrasps;         }; /// If true, grasp markers will be returned in getMarkers().

  //required for using fixed-size vectorizable Eigen types.
  //see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; //WorldObject

#endif //_WORLD_OBJECT_H_
