#ifndef _WORLD_OBJECT_H_
#define _WORLD_OBJECT_H_

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
#include <orp/ClassificationResult.h>
#include <visualization_msgs/Marker.h>

#include "core/orp_utils.h"

//helpful typedefs and forward declarations
class WorldObject;
struct WorldObjectType;
class WorldObjectManager;
typedef boost::shared_ptr< WorldObject > WorldObjectPtr;
typedef std::list< WorldObjectPtr > WorldObjectList;

/**
 * @brief Represents a type of object that can be found in the world.
 * 
 * Objects that are of the same WorldObjectType are functionally
 * identical in the real world (same color, size, weight, etc).
 *
 * If objects need to be distinguished from each other, they need to be
 * of different WorldObjectType.
 *
 * @version 1.0
 * @ingroup objectrecognition
 * @ingroup apc
 * 
 * @author    Adam Allevato <allevato@utexas.edu>
 * @copyright BSD 3-paragraph
 * @date      1/29/2015
 */
struct WorldObjectType {
  //TODO:
  // point cloud info?
  // possible grasps
  // weight
  // color information
  // size?
  std::string name; ///No two WorldObjectTypes should share a name.

  inline bool operator <(const WorldObjectType& wot1) const {
    //ROS_INFO("%s vs %s", name.c_str(), wot1.name.c_str());
    return name < wot1.name;
  };

  inline bool operator ==(const WorldObjectType& wot1) const {
    //ROS_INFO("%s vs %s", name.c_str(), wot1.name.c_str());
    return name == wot1.name;
  };
  WorldObjectType(): name("notset") {};
  WorldObjectType(std::string n): name(n) {};

  visualization_msgs::Marker stub;
  RPY offset;
  
public:
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
 *
 * @version 1.0
 * @ingroup objectrecognition
 * @ingroup apc
 * 
 * @author    Adam Allevato <allevato@utexas.edu>
 * @copyright BSD 3-paragraph
 * @date      1/29/2015
 */
class WorldObject {
private:
  static int nextValidID;     /// ncremented on new object creation.
  
  static const int KALMAN_SIZE = 3;
protected:
  ros::Time lastUpdated;      /// When this object was last updated by a recognition event
  int id;                     /// This object's unique ID. This is mainly used for ROS markers.
  WorldObjectType type;       /// The type of this object
  float probability;          /// Certainty of classification/pose
  Eigen::Affine3d pose;       /// position and orientation of this object (center of object)
  bool stale;                 ///If the object is stale, that means it needs to be deleted.
  WorldObjectManager* manager;///Manages global list of types.

  float colocationDistance; ///objects closer than this distance (in meters) are assumed to be the same one.

  std::vector<std::string> fullSensorModel; //list of all detectable items
  std::vector<std::string> subSensorModel;  //list of items to be detected
  
  visualization_msgs::Marker objectMarker; //visual representation of this
  visualization_msgs::Marker labelMarker; //the object's name

  
  Eigen::Matrix<double, KALMAN_SIZE,KALMAN_SIZE> covariance; //for Kalman filter
  Eigen::Matrix<double, KALMAN_SIZE,KALMAN_SIZE> Q; // for Kalman filter
public:
  /**
   * Constructor with more specifications, to be created from a classifier.
   */
  WorldObject(float colocationDist, WorldObjectManager* manager_, std::string type_, std::string frame, Eigen::Affine3d pose_, float probability_);

  /**
   * Should we merge with the specified object? For internal use
   * @param  other the world object proposed for merging
   * @return       true if should continue with merge process
   */
  virtual bool shouldMergeWith(WorldObject* other) { return false; };

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

  ///Outputs debug information about this object to ROS_INFO.
  virtual void debugPrint();

  ///Set the object's last updated time
  void setLastUpdated(ros::Time time);

  /**
   * Get this item's certainty.
   */
  float getProbability();

  /// @return the ROS time when this object was last updated/recognized.
  ros::Time getLastUpdated() { return lastUpdated; };

  void setProbability(float probability_);
  
  ///Get the object's autogenerated ID number
  int getID() { return id; };

  WorldObjectType getType() { return type; };
  
  visualization_msgs::Marker getObjectMarker() { return objectMarker; };
  
  visualization_msgs::Marker getLabelMarker() { return labelMarker; };

  //set the object pose
  void setPose( Eigen::Affine3d pos, bool hard = false);

  ///Get the object pose
  tf::Pose getPoseTf();
  ///Get the object pose
  Eigen::Affine3d getPose();
  /**
   * convienience method: set the translation portion of the object pose
   * @param pos The position to set the object pose's origin to 
   */
  //void setPosition(tf::Vector3 pos);
  ///Get the pose position
  //tf::Vector3 getPosition() { return getPoseTf().getOrigin(); }; ///Get the pose position

  //get the X-component of pose position
  float getX() { return getPose()(0,3); };
  ///Get the Y-component of pose position
  float getY() { return getPose()(1,3); };
  ///Get the Z-component of pose position
  float getZ() { return getPose()(2,3); };

  ///Set the X-component of pose position
  //float setX(float x) { pose(0,3) = x; };
  ///Set the Y-component of pose position
  //float setY(float y) { pose(1,3) = y; };
  ///Set the Z-component of pose position
  //float setZ(float z) { pose(2,3) = z; };

  //should this object be removed?
  bool isStale() { return stale; };
  //set this object as stale to kill it on the next update
  void setStale(bool s);
  
  float getColocationDist() { return colocationDistance; };

  bool isColocatedWith(WorldObjectPtr other);
  
  //required for using fixed-size vectorizable Eigen types. 
  //see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; //WorldObject

// /**
//  * @brief A special type of WorldObject that merges results based on Bayesian filtering.
//  *
//  *
//  * Stores a Bayesian ground
//  * truth confusion matrix (the sensor model). See the sensorModel variable for more info.
//  * 
//  * @version 1.0
//  * @ingroup objectrecognition
//  * @ingroup apc
//  * 
//  * @author    Adam Allevato <allevato@utexas.edu>
//  * @copyright BSD 3-paragraph
//  * @date      1/29/2015
//  */
// class WorldObjectBayes : public WorldObject {
// private:
//   float uniformLevel;       ///Used to generate uniform Bayesian priors on-the-fly.

//   /**
//    * Does the dirty work of looking up a conditional probability from the sensor model.
//    * If values aren't found, errors are printed and default values returned.
//    * The only exception is for the "unknown" type; this function doesn't
//    * create error messages if unknown can't be found in the sensor model.
//    * @param  row    The recognition result 
//    * @param  column The object class to look up
//    * @return        The conditional probability of "column" given recognition result "row"
//    */
//   float sensorModelAt(std::string row, std::string column);

//   /**
//    * Stores each row of the confusion matrix associated with this world sensor model.
//    * This encodes the conditional probability, p(z|C_j), or the probability of 
//    * getting result z given class C_j. z is the key of this map (std::string),
//    * and C_j is this class itself (with name WorldObjectType::name, an std::string).
//    */
//   FloatLookupTable sensorModel;

// public:

//   /**
//    * Default constructor; calls createUniformPrior
//    * @arg model the sensor model to use for Bayesian merging
//    * @arg colocationDist the max separation distance between objects
//    * @arg manage the object with the global list of types.
//    */
//   WorldObjectBayes(FloatLookupTable model, float colocationDist, WorldObjectManager* manage);
//   WorldObjectBayes(FloatLookupTable model, float colocationDist, WorldObjectManager* manage, 
//                     orp::WorldObject classRes);
//   WorldObjectBayes(FloatLookupTable model, float colocationDist, WorldObjectManager* manage,
//                     orp::WorldObject classRes, TypeMap probDistr);
//   /**
//    * Initializes the probabilities with a uniform prior.
//    * Set a uniform prior.
//    * This is a Bad Idea for a number of reasons, and I'm trying to come up with
//    * other ways of dealing with it. But for now, it's what Brian used, and I'm
//    * trying to recreate his methods before moving on.
//    */
//   void createUniformPrior();

//   /**
//    * VALIDATE and set the Bayesian sensor model for this world. This may have to
//    * be augmented in the future, when we have multisensory methods, but
//    * for now, it works.
//    * @param model the new sensor model.
//    */
//   void setSensorModel(FloatLookupTable model);

//   FloatLookupTable getSensorModel() { return sensorModel; };

//   /**
//    * Should we merge an object into this one?
//    * @param  wo proposed merge.
//    * @return    true if within the colocation distance, and this object has the information
//    * necessary to perform a merge
//    */
//   virtual bool shouldMergeWith(WorldObject* wo);

//   /**
//    * Special Bayesian merge function.
//    *
//    * The object being merged into this one is assumed to be a single reading, and
//    * will be combined with this object's results in a Bayesian fashion.
//    */
//   virtual bool merge(WorldObject* other);

//   ///Overload for more detailed info (if needed in the future)
//   virtual void debugPrint();
  
//   //required for using fixed-size vectorizable Eigen types. 
//   //see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// }; //WorldObjectBayes

/**
 * @brief A special type of WorldObject that merges results based on Bayesian filtering, and uses
 * Kalman filtering for pose/position estimation
 *
 * Stores a Bayesian ground
 * truth confusion matrix (the sensor model). See the sensorModel variable for more info.
 * 
 * @version 1.0
 * @ingroup objectrecognition
 * @ingroup apc
 * 
 * @author    Adam Allevato <allevato@utexas.edu>
 * @copyright BSD 3-paragraph
 * @date      1/29/2015
 */

// class WorldObjectBayesKalman : public WorldObjectBayes {
// protected:
  
//   /**
//    * Sensor noise covariance.
//    */
//   Eigen::Matrix4d sigma;

//   float subtractAngle(float angle_1, float angle_2);
// public:
//   /**
//    * Default constructor; calls createUniformPrior
//    * @arg model the sensor model to use for Bayesian merging
//    * @arg colocationDist the max separation distance between objects
//    * @arg manage the object with the global list of types.
//    */
//   WorldObjectBayesKalman(FloatLookupTable model, float colocationDist, WorldObjectManager* manage);
//   WorldObjectBayesKalman(FloatLookupTable model, float colocationDist, WorldObjectManager* manage, 
//                     orp::WorldObject classRes);
//   WorldObjectBayesKalman(FloatLookupTable model, float colocationDist, WorldObjectManager* manage,
//                     orp::WorldObject classRes, TypeMap probDistr);

//   /**
//    * Special Bayesian merge function.
//    *
//    * The object being merged into this one is assumed to be a single reading, and
//    * will be combined with this object's results in a Bayesian fashion.
//    */
//   virtual bool merge(WorldObject* other);
  
//   //required for using fixed-size vectorizable Eigen types. 
//   //see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

// }; //WorldObjectBayesKalman

#endif //_WORLD_OBJECT_H_
