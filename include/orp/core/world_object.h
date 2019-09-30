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
#include <algorithm>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>

//3rd party libraries
#include <boost/shared_ptr.hpp>

//ROS includes
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

#include <orp/WorldObject.h>

#include "orp/core/grasp.h"

//helpful typedefs and forward declarations
class WorldObject;
struct WorldObjectType;
class WorldObjectManager;
typedef boost::shared_ptr< WorldObject > WorldObjectPtr;
typedef std::list< WorldObjectPtr > WorldObjectList;

/// Distance to approach pose
/// TODO(Kukanani): this is a hacktastic feature, kill it!
const float APPROACH_DIST = 0.04f;

/**
 * roll, pitch and yaw (orientation) struct
 */
struct RPY {
  /// Rotation in radians around the X axis
  double roll;
  /// Rotation in radians around the Y axis
  double pitch;
  /// Rotation in radians around the Z axis
  double yaw;

  RPY() : roll(0), pitch(0), yaw(0) {};
};

/**
 * Possible shapes an object can have
 */
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
  /// No two WorldObjectTypes should share a name.
  std::string name;

  /// Unique identifier, used to identify the object in a more compact
  /// fashion than using the whole name.
  int id;

  /// This object's shape.
  ObjectShape shape;

  /// A marker with its appearance already prepared for this object type.
  visualization_msgs::Marker stub;

  /// Offset rotation of the appearance from default.
  RPY offset;

  /**
   * Do the barebones setup for this type's marker stub.
   */
  void setupStub() {
    stub.action = visualization_msgs::Marker::ADD;
    stub.color.a = 0.5; //make it visible
  };
public:
  /**
   * Create a new WorldObjectType. This constructor is usually only called
   * by the WorldObjectManager.
   * TODO(Kukanani): use friend classes to make this constructor protected,
   * while still being accessible from the world object manager.
   */
  WorldObjectType(int _id, std::string _name):
      id(_id),
      name(_name),
      shape(OTHER),
      stub(),
      offset()
  {
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

  /**
   * Set this object's size. Dimensions are in meters.
   * @param x size in the X-direction.
   * @param y size in the Y-direction.
   * @param z size in the Z-direction.
   */
  void setSize(float x, float y, float z) {
    stub.scale.x = x;
    stub.scale.y = y;
    stub.scale.z = z;
  };

  /**
   * Set this object's color. Values should be in the range [0.0,1.0].
   * @param r red
   * @param g green
   * @param b channel
   */
  void setColor(float r, float g, float b) {
    stub.color.r = r;
    stub.color.g = g;
    stub.color.b = b;
  };
  /**
   * Set the rotation of this object type's marker.
   * @param roll  rotation (radians) about X axis
   * @param pitch rotation (radians) about Y axis
   * @param yaw   rotation (radians) about Z axis
   */
  void setOffset(float roll, float pitch, float yaw) {
    offset.roll = roll; offset.pitch = pitch; offset.yaw = yaw;
  };

  /**
   * Set the shape of this object
   * @param _shape the new shape
   */
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

  /**
   * Create a tf Pose from this object's pose.
   * @return a pose with this object's RPY rotation from identity
   */
  tf::Pose generateOffsetPose() {
    return tf::Pose(tf::createQuaternionFromRPY(
      offset.roll, offset.pitch, offset.yaw
    ));
  };

  /**
   * Set the tf frame that this object lives in.
   * @param frameName frame to use for this object's marker.
   */
  void setFrame(std::string frameName) {
    stub.header.frame_id = frameName;
  };

  /**
   * Get the name of this type (not this specific object)
   * @return the object type's name
   */
  std::string getName() { return name; };

  /**
   * Get the ID of this object's type
   * @return the unique identifier for this object type.
   */
  int getID() { return id; };

  /**
   * Get a marker stub for this object type
   * @return a visualization marker that already looks the way this
   *         object should look.
   */
  visualization_msgs::Marker getStub() { return stub; };

  /**
   * Get this type's shape
   * @return the object type's shape
   */
  ObjectShape getShape() { return shape; };

  inline bool operator <(const WorldObjectType& wot1) const {
    return name < wot1.name;
  };

  inline bool operator ==(const WorldObjectType& wot1) const {
    return name == wot1.name;
  };

  // required for using fixed-size vectorizable Eigen types. See
  // http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
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

class WorldObject {
private:
  static int nextValidID;     /// ncremented on new object creation.

///////////////////////////////////////////////////////////////////////////////
// Yer a wizard, ORPy. Be sure to buy these magic constants before you
// first day of class! TODO(Kukanani): get rid of these one way or another.
///////////////////////////////////////////////////////////////////////////////

  /// Used to offset marker IDs and avoid collisions
  static const int MARKERS_PER_OBJECT = 10;
  //start grasp arrow marker IDs after this offset. One reserved for object, one for name
  static const int NON_GRASP_MARKER_COUNT = 2;

  /// Size of the Kalman filter matrix.
  /// TODO(Kukanani): This should never not be 3, so I think
  /// moving it inline should be fine
  static const int KALMAN_SIZE = 3;

  /// Default covariance of the measurement. Defined in .cpp file, because
  /// it's a float.
  /// TODO(Kukanani): this variable is tragically underused. The covariance
  /// should be a useful variable instead of a hacky constant.
  static const float MEASUREMENT_COVARIANCE;

///////////////////////////////////////////////////////////////////////////////
// Back to the muggle world
///////////////////////////////////////////////////////////////////////////////

  /// Set the type of this object
  void setType(WorldObjectType newType);

  /// Create the object marker
  void setupObjectMarker();

  /// Create the text marker
  void setupLabelMarker();

protected:
  /// This object's unique ID. This is mainly used for ROS markers.
  int id;
  /// Certainty of classification/pose
  float probability;
  /// The TF frame name that this object lives in.
  std::string frame;
  /// position and orientation of this object (center of object)
  Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> pose;
  /// TF version of the pose
  tf::Pose tPose;

  /// When this object was last updated by a recognition event
  ros::Time lastUpdated;
  /// If the object is stale, that means it needs to be deleted.
  bool stale;
  /// objects closer than this distance (in meters) are assumed to be the same one.
  float colocationDistance;

  /// Manages global list of types.
  WorldObjectManager* manager;
  /// The type of this object
  WorldObjectType type;

  /// If true, return grasp markers in getMarkers()
  bool showGrasps;
  /// Possible grasps for this object.
  std::vector<Grasp> grasps;
  /// Point cloud representing this object. Usually will be the point cloud
  /// used to detect the object in the first place.
  sensor_msgs::PointCloud2 cloud;

  //list of all detectable items
  std::vector<std::string> fullSensorModel;
  //list of items to be detected
  std::vector<std::string> subSensorModel;

  //visual representation of this
  visualization_msgs::Marker objectMarker;
  //the object's name
  visualization_msgs::Marker labelMarker;

  //for Kalman filter
  Eigen::Matrix<double, KALMAN_SIZE,KALMAN_SIZE> covariance;
  // for Kalman filter
  Eigen::Matrix<double, KALMAN_SIZE,KALMAN_SIZE> Q;

  /// Use a pseudo-Kalman statistical filter to update the object's position.
  void setPoseKalman(const Eigen::Affine3d& pos);
public:
  ///Create a new WorldObject from a WorldObject message.
  static WorldObject createFromMessage(WorldObjectManager* manager_,
      orp::WorldObject message);

  /**
   * Constructor. Unfortunately, a many arguments are required, which
   * makes building a WorldObject a bit of pain.
   */
  WorldObject(float colocationDist, WorldObjectManager* manager_,
    std::string type_, std::string frame, const Eigen::Affine3d& pose_,
    float probability_);

  /**
   * Return a human-friendly representation string of this object, including
   * the ID and type name.
   */
  std::string getDebugRepresentation();

  /**
   * Merge this object's data with another object. It's assumed that the other
   * is newer than this object.
   *
   * @arg other the WorldObject to merge with
   */
  virtual bool merge(WorldObjectPtr other);

  /**
   * Refreshes this object's last updated time to be ros::Time::now().
   *
   * Note that calling this repeatedly will invoke a lot of calls to
   * ros::Time::now(). Consider using setLastUpdated() instead.
   */
  void refresh();

  /// Set the position of the associated markers to match this object's pose.
  void updateMarkers();

  /// Use GraspGenerator to get a set of grasps for this object.
  void calculateGrasps();

  /**
   * Set the object's pose.
   * If the "hard" argument is true, then no filtering will be performed and
   * the object's pose will exactly match the "pos" argument after this
   * function ends. If "hard" is false, then the function will call
   * setPoseKalman() to update the pose statistically.
   */
  void setPose(const Eigen::Affine3d& pos, bool hard = false);

  /// Set the probability of this object's class
  void setProbability(float probability_);

  /// Set this object as stale to kill it on the next update
  void setStale(bool s);

  /// Set the object's last updated time
  void setLastUpdated(ros::Time time);

  /// Set the flag to show or hide grasps
  void setShowGrasps(bool show) { showGrasps = show; };

  /// Set this object's point cloud representation
  void setCloud(sensor_msgs::PointCloud2 _cloud) {
    cloud = _cloud;
  }

  /// @return the point cloud associated with this object.
  sensor_msgs::PointCloud2 getCloud() { return cloud; };

  /// Get whatever markers are appropriate for the object in it's current
  /// state.
  std::vector<visualization_msgs::Marker> getMarkers();

  /// Get the classification certainty.
  float getProbability();

  /// Get the object pose
  tf::Pose getPoseTf();

  /// Get the object pose including frame information
  tf::Stamped<tf::Pose>  getPoseTfStamped();

  /// Get the object pose
  const Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign>& getPose();

  /// Get this object's type.
  WorldObjectType getType();

  /// @return the ROS time when this object was last updated/recognized.
  inline ros::Time getLastUpdated() {
    return lastUpdated;
  };

  /// Get the object's autogenerated ID number (unique).
  inline int getID() {
    return id;
  };

  /// Get the name of the TF frame that this object's poses are relative to.
  inline std::string getFrame() {
    return frame;
  };

  /// Get the X-component of pose position
  inline float getX() {
    return getPose()(0,3);
  };

  /// Get the Y-component of pose position
  inline float getY() {
    return getPose()(1,3);
  };

  /// Get the Z-component of pose position
  inline float getZ() {
    return getPose()(2,3);
  };

  /// Objects closer than this distance should be merged with this object.
  inline float getColocationDistance() {
    return colocationDistance;
  };

  /// Get all the grasps for this object
  inline std::vector<Grasp> getGrasps() {
    return grasps;
  };

  /// If true, this object should be removed.
  inline bool isStale() {
    return stale;
  };
  /// If true, then this object is close enough to the other object
  /// to be considered the same object, and a merge is imminent.
  bool isColocatedWith(WorldObjectPtr other);

  /// Return the distance from this object to the other.
  float distanceTo(WorldObjectPtr other);
  /// If true, grasp markers will be returned in getMarkers().
  inline bool getShowGrasps() {
    return showGrasps;
  };

  // required for using fixed-size vectorizable Eigen types. See
  // http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
