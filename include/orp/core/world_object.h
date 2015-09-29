#ifndef _WORLD_OBJECT_H_
#define _WORLD_OBJECT_H_

#include <string>
#include <algorithm>
#include <map>
#include <cstdlib>
#include <memory>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <orp/ClassificationResult.h>

//helpful typedefs and forward declarations
class WorldObject;
struct WorldObjectType;
class WorldObjectManager;
struct PoseGuess;
typedef boost::shared_ptr< WorldObject > WorldObjectPtr;
typedef std::list< WorldObjectPtr > WorldObjectList;
typedef std::map< WorldObjectType , PoseGuess,
  std::less<WorldObjectType>,
  Eigen::aligned_allocator<std::pair<const WorldObjectType, PoseGuess > > >
    TypeMap;

#define MIN_PROB 0.001f

struct PoseGuess {
  Eigen::Affine3d pose;
  float prob;

  PoseGuess(float probability = MIN_PROB, Eigen::Affine3d pos = Eigen::Affine3d()) :
    prob(probability), pose(pos)
  {

  };
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * roll, pitch and yaw (orientation) struct
 */
struct RPY {
  double roll; //x axis
  double pitch; //y axis
  double yaw; //z axis

  RPY() : roll(0), pitch(0), yaw(0) {};
};

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
protected:
  ros::Time lastUpdated;      /// When this object was last updated by a recognition event
  int id;                     /// This object's unique ID. This is mainly used for ROS markers.
  TypeMap types;              /// The probability (normalized 0-1) that this object is each type.
  bool stale;                 ///If the object is stale, that means it needs to be deleted.
  WorldObjectManager* manager;///Manages global list of types.

  float colocationDistance; ///objects closer than this distance [m] are assumed to be the same one.


public:

  /**
   * Default constructor
   * @arg manage the object with the global list of types.
   */
  WorldObject(WorldObjectManager* manage, float colocationDist);

  /**
   * Should we merge with the specified object? For internal use
   * @param  other the world object proposed for merging
   * @return       true if should continue with merge process
   */
  virtual bool shouldMergeWith(WorldObject* other) { return false; };

  /**
   * Merge this object's data with another object. It's assumed that the other is newer than this object.
   * TODO: This is where the Bayesian reliability updates can take place. Perhaps this 
   * method should be overloaded in a child class?
   *
   * @arg other the WorldObject to merge with
   */
  virtual bool merge(WorldObject* other);

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
   * Get the best guess as to what type of object this is.
   *
   * If multiple objects are tied for most probable, it will return the
   * label that comes first in alphabetical order
   * 
   * @return the most likely WorldObjectType for this object
   */
  WorldObjectType getBestType();

  /**
   * Get the worst guess as to what type of object this is.
   *
   * If multiple objects are tied for least probable, it will return the
   * label that comes first in alphabetical order
   * 
   * @return the least likely WorldObjectType for this object
   */
  WorldObjectType getWorstType();

  /**
   * Get the best guess as to what type of object this is, subject to a threshold
   *
   * If multiple objects are tied for most probable, it will return the
   * label that comes first in alphabetical order.
   * 
   * @return the most likely WorldObjectType for this object, as long as the likelihood
   * is greater than the supplied threshold value.
   */
  WorldObjectType getType(float threshold);

  /**
   * Give the pose for the best-probability object out of all defined types.
   */
  Eigen::Affine3d& getBestPose();

  /**
   * Give the pose/probability for the best-probability object out of all defined types.
   */
  PoseGuess getBestPoseGuess();

  /**
   * Get the best probability of any types out of all the types defined for this object.This
   * gives no information about what the most likely type is.
   */
  float getBestProbability();

  /**
   * Get the worst probability of any types out of all the types defined for this object. This
   * gives no information about what the most likely type is.
   */
  float getWorstProbability();

  /// @return the ROS time when this object was last updated/recognized.
  ros::Time getLastUpdated() { return lastUpdated; };

  /// Force-set the probability of a given WorldObjectType.
  void setProbabilityOf(WorldObjectType wot, float prob, Eigen::Affine3d pose);

  /**
   * Convenience method
   * @arg wotName the name of the word object type for which to set the probability
   * @arg prob the proability that this object is the given class.
   */
  void setProbabilityOf(std::string wotName, float prob, Eigen::Affine3d pose);

  /// @return the probability that this object is of the specified type.
  float getProbabilityOf(WorldObjectType wot);
  /**
   * Convience method
   * @return the probability that this object is of the type specified by the given name.
   */
  float getProbabilityOf(std::string name);

  ///Get the object's autogenerated ID number
  int getID() { return id; };

  TypeMap getTypes() { return types; };  

  //set the object pose
  void setPose(std::string wotName, tf::Pose pos);

  //set the object pose from the given message
  void setPose(std::string wotName, geometry_msgs::Pose pos);

  //set the pose using the x,y,z coordinates and the rotation about the vertical axis.
  void set4DOFPose(std::string wotName, Eigen::Vector4f thePose);

  //return the x,y,z position and the rotation about the vertical axis.
  Eigen::Vector4f get4DOFPose(std::string wotName);
  ///Get the object pose
  tf::Pose getPoseTf(std::string wotName);
  ///Get the object pose
  Eigen::Affine3d getPose(std::string wotName) { return types[wotName].pose; };
   /**
   * convienience method: set the translation portion of the object pose
   * @param pos The position to set the object pose's origin to 
   */
  void setPosition(std::string wotName, tf::Vector3 pos);
  ///Get the pose position
  tf::Vector3 getPosition(std::string wotName) { return getPoseTf(wotName).getOrigin(); }; ///Get the pose position
  float getBestX() { return getBestPose()(0,3); };
  ///Get the Y-component of pose position
  float getBestY() { return getBestPose()(1,3); };
  ///Get the Z-component of pose position
  float getBestZ() { return getBestPose()(2,3); };

  float getX(std::string wotName) { return types[wotName].pose(0,3); };
  ///Get the Y-component of pose position
  float getY(std::string wotName) { return types[wotName].pose(1,3); };
  ///Get the Z-component of pose position
  float getZ(std::string wotName) { return types[wotName].pose(2,3); };

  ///Set the X-component of pose position
  float setX(std::string wotName, float x) { types[wotName].pose(0,3) = x; };
  ///Set the Y-component of pose position
  float setY(std::string wotName, float y) { types[wotName].pose(1,3) = y; };
  ///Set the Z-component of pose position
  float setZ(std::string wotName, float z) { types[wotName].pose(2,3) = z; };

  //should this object be removed?
  bool isStale() { return stale; };
  //set this object as stale to kill it on the next update
  void setStale(bool s) { stale = s; };
  
  float getColocationDist() { return colocationDistance; };

  //required for using fixed-size vectorizable Eigen types. 
  //see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; //WorldObject

typedef std::map<std::string, float> FloatLookupRow;
typedef std::map< std::string, FloatLookupRow > FloatLookupTable;

/**
 * @brief A special type of WorldObject that merges results based on Bayesian filtering.
 *
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
class WorldObjectBayes : public WorldObject {
private:
  float uniformLevel;       ///Used to generate uniform Bayesian priors on-the-fly.

  /**
   * Does the dirty work of looking up a conditional probability from the sensor model.
   * If values aren't found, errors are printed and default values returned.
   * The only exception is for the "unknown" type; this function doesn't
   * create error messages if unknown can't be found in the sensor model.
   * @param  row    The recognition result 
   * @param  column The object class to look up
   * @return        The conditional probability of "column" given recognition result "row"
   */
  float sensorModelAt(std::string row, std::string column);

  /**
   * Stores each row of the confusion matrix associated with this world sensor model.
   * This encodes the conditional probability, p(z|C_j), or the probability of 
   * getting result z given class C_j. z is the key of this map (std::string),
   * and C_j is this class itself (with name WorldObjectType::name, an std::string).
   */
  FloatLookupTable sensorModel;

public:

  /**
   * Default constructor; calls createUniformPrior
   * @arg model the sensor model to use for Bayesian merging
   * @arg colocationDist the max separation distance between objects
   * @arg manage the object with the global list of types.
   */
  WorldObjectBayes(FloatLookupTable model, float colocationDist, WorldObjectManager* manage);
  WorldObjectBayes(FloatLookupTable model, float colocationDist, WorldObjectManager* manage, 
                    orp::WorldObject classRes);
  WorldObjectBayes(FloatLookupTable model, float colocationDist, WorldObjectManager* manage,
                    orp::WorldObject classRes, TypeMap probDistr);
  /**
   * Initializes the probabilities with a uniform prior.
   * Set a uniform prior.
   * This is a Bad Idea for a number of reasons, and I'm trying to come up with
   * other ways of dealing with it. But for now, it's what Brian used, and I'm
   * trying to recreate his methods before moving on.
   */
  void createUniformPrior();

  /**
   * VALIDATE and set the Bayesian sensor model for this world. This may have to
   * be augmented in the future, when we have multisensory methods, but
   * for now, it works.
   * @param model the new sensor model.
   */
  void setSensorModel(FloatLookupTable model);

  FloatLookupTable getSensorModel() { return sensorModel; };

  /**
   * Should we merge an object into this one?
   * @param  wo proposed merge.
   * @return    true if within the colocation distance, and this object has the information
   * necessary to perform a merge
   */
  virtual bool shouldMergeWith(WorldObject* wo);

  /**
   * Special Bayesian merge function.
   *
   * The object being merged into this one is assumed to be a single reading, and
   * will be combined with this object's results in a Bayesian fashion.
   */
  virtual bool merge(WorldObject* other);

  ///Overload for more detailed info (if needed in the future)
  virtual void debugPrint();
  
  //required for using fixed-size vectorizable Eigen types. 
  //see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; //WorldObjectBayes

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
class WorldObjectBayesKalman : public WorldObjectBayes {
protected:
  
  /**
   * Sensor noise covariance.
   */
  Eigen::Matrix4d sigma;

  float subtractAngle(float angle_1, float angle_2);
public:
  /**
   * Default constructor; calls createUniformPrior
   * @arg model the sensor model to use for Bayesian merging
   * @arg colocationDist the max separation distance between objects
   * @arg manage the object with the global list of types.
   */
  WorldObjectBayesKalman(FloatLookupTable model, float colocationDist, WorldObjectManager* manage);
  WorldObjectBayesKalman(FloatLookupTable model, float colocationDist, WorldObjectManager* manage, 
                    orp::WorldObject classRes);
  WorldObjectBayesKalman(FloatLookupTable model, float colocationDist, WorldObjectManager* manage,
                    orp::WorldObject classRes, TypeMap probDistr);

  /**
   * Special Bayesian merge function.
   *
   * The object being merged into this one is assumed to be a single reading, and
   * will be combined with this object's results in a Bayesian fashion.
   */
  virtual bool merge(WorldObject* other);
  
  //required for using fixed-size vectorizable Eigen types. 
  //see http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; //WorldObjectBayesKalman

#endif //_WORLD_OBJECT_H_
