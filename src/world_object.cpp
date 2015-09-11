#include "orp/core/world_object.h"
#include "orp/core/world_object_manager.h"

int WorldObject::nextValidID = 0;

WorldObject::WorldObject(WorldObjectManager* manage):
  types(),
  lastUpdated(ros::Time::now()),
  id(nextValidID++),
  stale(false),
  manager(manage),
  strength(1)
{
  types.insert(TypeMap::value_type (manager->getUnknownType(), PoseGuess(std::numeric_limits<float>::min())));
  //lowest possible positive value distinguishable from 0
} //WorldObject constructor

bool WorldObject::mergeIn(WorldObject* other)
{
  return false;
} //merge

void WorldObject::refresh()
{
  setLastUpdated(ros::Time::now());
} //refresh

void WorldObject::debugPrint()
{
  ROS_INFO("\t%i\t%s: (%f, %f, %f)", getID(), getBestType().name.c_str(), getBestX(), getBestY(), getBestZ());
  //for(TypeMap::iterator it = types.begin(); it != types.end(); it++) {
    //ROS_DEBUG("\t\t\t%s\t: %f", it->first.name.c_str(), it->second);
  //}
} //getInfoString

void WorldObject::setLastUpdated(ros::Time time)
{
  lastUpdated = time;
} //refresh

WorldObjectType WorldObject::getBestType()
{
  float max = 0.0;
  WorldObjectType best = manager->getUnknownType();
  for(TypeMap::iterator it = types.begin(); it != types.end(); it++)
  {
    //ROS_DEBUG("prob for %s is %f", it->first.name.c_str(), it->second);
    if(it->second.prob > max)
    {
      best = it->first;
      max = it->second.prob;
    }
  }
  return best;
} //getBestType

WorldObjectType WorldObject::getWorstType()
{
  float min = std::numeric_limits<float>::max();
  WorldObjectType worst = manager->getUnknownType();
  for(TypeMap::iterator it = types.begin(); it != types.end(); it++)
  {
    //ROS_DEBUG("prob for %s is %f", it->first.name.c_str(), it->second);
    if(it->second.prob < min)
    {
      worst = it->first;
      min = it->second.prob;
    }
  }
  return worst;
} //getWorstType

PoseGuess WorldObject::getBestPoseGuess()
{
  float max = 0.0;
  PoseGuess best;
  if(types.size() < 1)
  {
    ROS_ERROR("no probabilities in set when trying to get the best probability");
  }
  for(TypeMap::iterator it = types.begin(); it != types.end(); it++)
  {
    //ROS_DEBUG("prob for %s is %f", it->first.name.c_str(), it->second);
    if(it->second.prob > max)
    {
      max = it->second.prob;
      best = it->second;
    }
  }
  return best;
} //getBestPoseGuess

Eigen::Affine3d& WorldObject::getBestPose()
{
  float max = 0.0;
  Eigen::Affine3d* best;
  if(types.size() < 1)
  {
    ROS_ERROR("no probabilities in set when trying to get the best probability");
  }
  for(TypeMap::iterator it = types.begin(); it != types.end(); it++)
  {
    //ROS_DEBUG("prob for %s is %f", it->first.name.c_str(), it->second);
    if(it->second.prob > max)
    {
      max = it->second.prob;
      best = &(it->second.pose);

      Eigen::Matrix3d rotMat = best->linear();
      Eigen::Quaterniond rotQ; rotQ = rotMat;
      //ROS_INFO("New best with quat: %f %f %f %f", rotQ.x(), rotQ.y(), rotQ.z(), rotQ.w());
    }
  }
  return *best;
} //getBestPose

float WorldObject::getBestProbability()
{
  float max = 0.0;
  if(types.size() < 1)
  {
    ROS_ERROR("no probabilities in set when trying to get the best probability");
  }
  for(TypeMap::iterator it = types.begin(); it != types.end(); it++)
  {
    //ROS_DEBUG("prob for %s is %f", it->first.name.c_str(), it->second);
    if(it->second.prob > max)
    {
      max = it->second.prob;
    }
  }
  return max;
} //getBestProbability

float WorldObject::getWorstProbability()
{
  float min = std::numeric_limits<float>::max();
  if(types.size() < 1)
  {
    ROS_ERROR("no probabilities in set when trying to get the worst probability");
  }
  for(TypeMap::iterator it = types.begin(); it != types.end(); it++)
  {
    //ROS_DEBUG("prob for %s is %f", it->first.name.c_str(), it->second);
    if(it->second.prob < min)
    {
      min = it->second.prob;
    }
  }
  return min;
} //getWorstProbability
WorldObjectType WorldObject::getType(float threshold)
{
  if(threshold <= 0 || threshold > 1)
  {
    ROS_ERROR("Threshold supplied to WorldObject::getType should be in the range (0.0,1.0]");
  }

  float max = 0.0;
  WorldObjectType best = manager->getUnknownType();
  for(TypeMap::iterator it = types.begin(); it != types.end(); it++)
  {
    //ROS_DEBUG("prob for %s is %f", it->first.name.c_str(), it->second);
    if(it->second.prob > max && it->second.prob >= threshold)
    {
      best = it->first;
      max = it->second.prob;
    }
  }
  return best;
} //getBestType

void WorldObject::setPose(std::string wotName, tf::Pose pos)
{
  Eigen::Affine3d toSet;
  tf::poseTFToEigen(pos, toSet);
  types[wotName].pose = toSet;
} //setPose

void WorldObject::setPose(std::string wotName, geometry_msgs::Pose pos)
{
  tf::Pose post;
  post.setOrigin(tf::Vector3(pos.position.x, pos.position.y, pos.position.z));
  post.setRotation(tf::Quaternion(pos.orientation.x, pos.orientation.y, pos.orientation.z,
    pos.orientation.w));
  setPose(wotName, post);
} //setPose

void WorldObject::setPosition(std::string wotName, tf::Vector3 pos)
{
  Eigen::Affine3d toSet = getPose(wotName);
  toSet(0,3) = (double)pos.x();
  toSet(1,3) = (double)pos.y();
  toSet(2,3) = (double)pos.z();
} //setPosition

void WorldObject::setProbabilityOf(WorldObjectType wot, float prob, Eigen::Affine3d pose)
{
  //ROS_INFO("setting probability");
  //TODO: verify that probabilities are normalized?
  std::pair<TypeMap::iterator, bool> res = types.insert(TypeMap::value_type(wot, PoseGuess(prob, pose)));
  if(res.second == false)
  { //probability already exists at this key
    types[wot] = PoseGuess(std::max(MIN_PROB, prob), pose);
  } else
  { 
    //inserted a new, nothing to do
  }
} //setProbabilityOf

void WorldObject::setProbabilityOf(std::string wotName, float prob, Eigen::Affine3d pose)
{
  setProbabilityOf(manager->getTypeByName(wotName), prob, pose);
}

float WorldObject::getProbabilityOf(WorldObjectType wot)
{
  ROS_INFO_STREAM("getProbabilityOf(" << wot.name << ") = " << types[wot].prob);
  return types[wot].prob; //returns MIN_PROB if has to create a new value
} //getProbabilityOf

float WorldObject::getProbabilityOf(std::string name)
{
  return getProbabilityOf(manager->getTypeByName(name)); //returns 0 if has to create a new value
} //getProbabilityOf

tf::Pose WorldObject::getPoseTf(std::string wotName) {
  tf::Pose tPose;
  tf::poseEigenToTF(getPose(wotName), tPose);
  return tPose;
}

Eigen::Vector4f WorldObject::get4DOFPose(std::string wotName)
{
  Eigen::Vector4f retVal;
  retVal(0) = getBestX();
  retVal(1) = getBestY();
  retVal(2) = getBestZ();
  retVal(3) = getBestPose().rotation().matrix().eulerAngles(0,2,1)[2];
  return retVal;
}

void WorldObject::set4DOFPose(std::string wotName, Eigen::Vector4f thePose)
{
  getPose(wotName).linear() = (Eigen::AngleAxisd(thePose[3], Eigen::Vector3d::UnitY())).toRotationMatrix();
  setX(wotName, thePose[0]);
  setY(wotName, thePose[1]);
  setZ(wotName, thePose[2]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

WorldObjectBayes::WorldObjectBayes(FloatLookupTable model,
                                    float colocationDist,
                                    WorldObjectManager* manage) :
  WorldObject(manage),
  colocationDistance(colocationDist)
{
  //ROS_INFO("creating new Bayesian world object with no inherent type");
  setSensorModel(model);
  createUniformPrior();
  setStale(false);
  this->setLastUpdated(ros::Time::now());
} //WorldObjectBayes constructor

WorldObjectBayes::WorldObjectBayes(FloatLookupTable model, float colocationDist,
                                   WorldObjectManager* manage, orp::WorldObject classRes) :
  WorldObject(manage),
  colocationDistance(colocationDist)
{
  //ROS_INFO("creating new Bayesian world object of type %s", classRes.label.c_str());
  setSensorModel(model);
  createUniformPrior();
  setStale(false);

  tf::Pose tfPose;
  tf::poseMsgToTF(classRes.pose.pose, tfPose);
  Eigen::Affine3d eigPose;
  tf::poseTFToEigen(tfPose, eigPose);
  setProbabilityOf(classRes.label, 1.0, eigPose);

  this->setLastUpdated(ros::Time::now());
} //WorldObjectBayes constructor


WorldObjectBayes::WorldObjectBayes(FloatLookupTable model,
                float colocationDist,
                WorldObjectManager* manage,
                orp::WorldObject classRes,
                TypeMap probDistr) :
  WorldObject(manage),
  colocationDistance(colocationDist)
{
  types = probDistr;
  setSensorModel(model);
  createUniformPrior();
  setStale(false);

  tf::Pose tfPose;
  tf::poseMsgToTF(classRes.pose.pose, tfPose);
  Eigen::Affine3d eigPose;
  tf::poseTFToEigen(tfPose, eigPose);
  setProbabilityOf(classRes.label, 1.0, eigPose);

  //debug print
  for(TypeMap::iterator it = types.begin(); it != types.end(); ++it) {
    Eigen::Matrix3d rotMat = it->second.pose.linear();
    Eigen::Quaterniond rotQ; rotQ = rotMat;
    //ROS_INFO("world object: new object (%f) has %s position %f, %f, %f", it->second.prob, it->first.name.c_str(), it->second.pose(0,3), it->second.pose(1,3), it->second.pose(3,3));
    //   ROS_INFO("pose for %s has quat %f, %f, %f, %f", it->first.name.c_str(), rotQ.x(), rotQ.y(), rotQ.z(), rotQ.w());
  }

  this->setLastUpdated(ros::Time::now());
}

void WorldObjectBayes::setSensorModel(FloatLookupTable model)
{
  sensorModel = FloatLookupTable(model);

  bool mustRebuild = false;
  float uniform = 0.37; //odd value so you can tell if something's off
  int numRows = sensorModel.size();
  //ROS_INFO("%i rows", numRows);

  //Make sure we have something....WorldObjectBayes::setSensorModel
  if(numRows < 2)
  {
    ROS_ERROR("Not enough rows passed to sensor model. Assuming 2 detectable"
      " object types. Errors will occur.");
    uniform = 0.5;
    mustRebuild = false;
  }

  //Squareness check
  FloatLookupTable::iterator row;
  for(FloatLookupTable::iterator row = sensorModel.begin(); row != sensorModel.end(); ++row)
  {
    if(sensorModel.size() != row->second.size()) {
      ROS_ERROR("Sensor model is not square (expected %i vs. actual %i)!",
        (int) sensorModel.size(), (int) row->second.size());
      uniform = 0.5;
      numRows = 2;
      mustRebuild = true;
    }
  }

  if(mustRebuild)
  {
    ROS_ERROR("rebuilding a custom sensor model with a uniform distribution. This will result in "
      "meaningless recognition results. The table is assumed to have %i x %i elements, which is "
      "the number of rows in the table.", numRows, numRows);

    std::vector<std::string> keys;
    for(row = sensorModel.begin(); row != sensorModel.end(); ++row)
    {
      keys.push_back(row->first);
    }
    for(row = sensorModel.begin(); row != sensorModel.end(); ++row)
    {
      row->second.clear();
      for(std::vector<std::string>::iterator key = keys.begin(); key != keys.end(); ++key)
      {
        row->second[*key] = uniform;
      }
    }
  }
  else
  {
    for(row = sensorModel.begin(); row != sensorModel.end(); ++row)
    {
      // Calculate the normalization sum
      float sum = 0.0;
      FloatLookupRow::iterator column;
      for(column = row->second.begin(); column != row->second.end(); ++column)
      {
        sum += column->second;
      }
      if(sum == 0)
      {
        ROS_ERROR("Sensor model passed to Bayes object type %s summed to 0.", row->first.c_str());
        sum = 1.0;
      }
      if(sum < 0)
      {
        ROS_ERROR("Sensor model passed to Bayes object type %s summed to <0. What the hey...",
          row->first.c_str());
        sum = 1.0;
      }
      if(row->second.size() < 1)
      {
        ROS_ERROR("Empty sensor model passed to Bayes object type %s", row->first.c_str());
        sum = 1.0;
      }
      //ROS_DEBUG("Normalizing Bayesian sensor model by value %f", sum);
      for(column = row->second.begin(); column != row->second.end(); ++column)
      {
        column->second /= sum;
      }
    }
  }
  //ROS_INFO("Bayesian sensor model set (%i rows)", sensorModel.size());
} //setSensorModel

void WorldObjectBayes::createUniformPrior()
{
    //here comes the uniform prior
  float uniformLevel = 1.0/(sensorModel.size()+1);
  //ROS_INFO("creating uniform prior with value = %f (sensorModel size %i)", uniformLevel,
  //  sensorModel.size());
  FloatLookupRow::iterator column;
  for(column = sensorModel.begin()->second.begin(); column != sensorModel.begin()->second.end();
    ++column)
  {
    setProbabilityOf(manager->getTypeByName(column->first), uniformLevel, Eigen::Affine3d());
  }
  types.insert(TypeMap::value_type (manager->getUnknownType(), PoseGuess()));
} //createUniformPrior

bool WorldObjectBayes::shouldMergeWith(WorldObject* other) {
  //ROS_INFO("trying to merge obj %i into obj %i", other->getID(), getID());
  if(other == this) {
    //ROS_INFO("tried to merge with self!");
    return false;
  }
  if(stale)
  {
    //ROS_INFO("object %i stale. Not merging things into here", getID());
    return false;
  }
  if(other->isStale())
  {
    //ROS_INFO("object %i stale. Not merging it into %i", other->getID(), getID());
    return false;
  }
  if(other->getLastUpdated() > getLastUpdated())
  {
    //ROS_INFO("Switcheroo, %i to %i", getID(), other->getID());
    other->mergeIn(this);
    return false;
  }

  //ROS_INFO("actual %f vs threshold %f", tf::tfDistance(this->getPoseTf().getOrigin(),other->getPoseTf().getOrigin()), colocationDistance);
  if(tf::tfDistance(this->getPoseTf(getBestType().name).getOrigin(),other->getPoseTf(getBestType().name).getOrigin()) > colocationDistance)
  {
    return false;
  }
  ////////////////////////////////////////////////////////////////////////////////////
  // Page 57 in Brian's thesis. This is the part of the code that I'm least confident about,
  // because the amount of notation in Bayes' rule trips me up constantly. So I'm going to comment
  // this section exhaustively.
  

  // We're trying to
  // calculate P(C_j|z): the probability that this object IS ACTUALLY class C_j, given that
  // we have the recognition result z.
  
  // STEP 1: some sanity checking. The object being merged in is the recognition result,
  // so it should be really clear what type of object it is. In the future, there will
  // be a possibility for ambiguity, but this is a more generalized form of uncertainty
  // management (Perhaps a Kalman filter?)

  // Now we need to calculate P(C_j|z) for each class.
  // STEP 2:  All of these calculations
  // have the same denominator, which is sum[p(z|C_k)*P(C_k)] over k = all types.
  // This is the denominator, so it CANNOT equal zero at function end. There are 
  // three ways this can equal 0. Here they are, with explanations and checkes.
  // 1. No items in the type list. We have bigger problems if this is happening,
  //    but for now just note it with an error.
  if(types.size() < 1)
  {
    ROS_ERROR("No types in type list");
    return false;
  }

  //If we make it to this point, we're going to actually attempt a merge.
  return true;
} //shouldMergeWith

bool WorldObjectBayes::mergeIn(WorldObject* other)
{
  if(!shouldMergeWith(other)) {
    return false;
  }
  return merge(other);
} //mergeIn

bool WorldObjectBayes::merge(WorldObject* other) {
    //ROS_INFO("Actually performing a merge");
  WorldObjectType newBest = other->getBestType();
  strength += other->getStrength();

  if(other->getProbabilityOf(newBest) < 0.5)
  {
    ROS_ERROR("Incoming measurement is very uncertain (%f)!", other->getProbabilityOf(newBest));
  }

  /*float denominator = 0.0;
  //ROS_INFO("Merging a %s detection (with a %f chance). obj %i => obj %i",
  //  newBest.name.c_str(),
  //  other->getProbabilityOf(newBest),
  //  other->getID(),
  //  id);

  // We have something, so we can loop
  for(TypeMap::iterator k = types.begin(); k != types.end(); k++)
  {

    // 2. The sensor model does not contain some of the probabilities for this class.
    //    If this happens, it means that we've detected this object as possibly 
    //    being an object for which we are unsure of the measurement uncertainty.
    //    Assume that this measurement means nothing, and do not let it contribute
    //    to the posterior. In other words, let zeros be zeros (do nothing).

    // 3. Finally, the current probability of this class can be 0 for this object.
    // 
    //    If the type probability is 0, 
    //    This could be because it's been determined to be 0 by the sensor model,
    //    or because it simply doesn't have a value for the class. At initialization,
    //    The sensor model is pre-determined and converted to an initial uniform
    //    classification guess, and sensor model values cannot begin
    //    at 0, so the logical conclusion is that, prior to this point, this object
    //    has had no knowledge of this object class. Which makes no sense based on how 
    //    this object is initialized. So, just let it go to 0 as well. But let the user know,
    //    because it's possible that they wanted to detect this class, but it's impossible with
    //    a level of 0.
    if(getProbabilityOf(k->first.name) == 0)
    {
      ROS_ERROR("Current probability of %s is 0, and it will stay that way forever", k->first.name.c_str());
    }

    denominator += sensorModelAt(k->first.name, newBest.name) * getProbabilityOf(k->first.name);
  }
  //ROS_INFO("denom is %f", denominator);
  
  if(denominator == 0)
  {
    // If, after all this, the denominator is STILL 0, hard set it to avoid crashes.
    ROS_ERROR("Setting Bayes denominator from 0 to 1. Recognition results will be incorrect.");
    denominator = 1;
  }

  //ROS_INFO("Current probability that it's a %s is %f",
  //  newBest.name.c_str(),
  //  getProbabilityOf(newBest.name));
  ROS_INFO("\tSensor model says that if we detect a %s, it's a %s with %f certainty.",
    newBest.name.c_str(),
    newBest.name.c_str(),
    sensorModelAt(newBest.name, newBest.name));

  // STEP 3: Perform the Bayesian calculations.
  for(TypeMap::iterator j = types.begin(); j != types.end(); j++)
  {
    // ROS_INFO("Current probability that it's a %s is %f",
    //   j->first.name.c_str(),
    //   getProbabilityOf(j->first.name));
    // ROS_INFO("\tSensor model says that if we detect a %s, it's a %s with %f certainty.",
    //   j->first.name.c_str(),
    //   newBest.name.c_str(),
    //   sensorModelAt(j->first.name,
    //     newBest.name));
    // setProbabilityOf(j->first,
    //   sensorModelAt(j->first.name, newBest.name)*getProbabilityOf(j->first.name)/denominator);
    // ROS_INFO("\tfinal probability that it's a %s is %f",
    //   j->first.name.c_str(),
    //   getProbabilityOf(j->first.name));
  }
  //ROS_INFO("\tfinal probability that it's a %s is %f",
  //  getBestType().name.c_str(),
  //  getBestProbability());
  //ROS_INFO("\tfinal probability that it's a %s is %f",
  //  getWorstType().name.c_str(),
  //  getWorstProbability());
  */
  sensorModel = static_cast<WorldObjectBayes*>(other)->getSensorModel();
  other->setStale(true);
  refresh();
  return true;
} //merge

float WorldObjectBayes::sensorModelAt(std::string row, std::string column)
{
  //ROS_INFO("requesting sensor model at %s, %s", row.c_str(), column.c_str());
  if(sensorModel.find(row) == sensorModel.end())
  {
    ROS_ERROR("Row %s not found in global Bayesian sensor model", row.c_str());
  }
  else if(
    sensorModel.find(row)->second.find(column) ==
    sensorModel.find(row)->second.end() )
  {
    ROS_ERROR("Column %s not found in global Bayesian sensor model, row %s",
      column.c_str(),
      row.c_str());
  }

  return sensorModel[row][column];
} //sensorModelAt

void WorldObjectBayes::debugPrint()
{
  for(TypeMap::iterator it = types.begin(); it != types.end(); it++) {
    Eigen::Quaterniond quat;
    quat = it->second.pose.rotation() ;
    ROS_INFO("\t%i\t%s: (%f, %f, %f), [%lf, %lf, %lf, %lf]", getID(), it->first.name.c_str(),
      it->second.pose(0,3), it->second.pose(1,3), it->second.pose(2,3),
      quat.x(), quat.y(), quat.z(), quat.w());
  }
} //getInfoString

////////////////////////////////////////////////////////////////////////////////////////////////////

WorldObjectBayesKalman::WorldObjectBayesKalman(FloatLookupTable model,
                                                float colocationDist,
                                                WorldObjectManager* manage) :
  WorldObjectBayes(model, colocationDist, manage)
{
  sigma.setIdentity();
  sigma(0,0) = 1000; sigma(1,1) = 1000; sigma(2,2) = 1000; sigma(3,3) = 1000;
} //WorldObjectBayesKalman

WorldObjectBayesKalman::WorldObjectBayesKalman(FloatLookupTable model,
                                                float colocationDist,
                                                WorldObjectManager* manage,
                                                orp::WorldObject classRes) :
  WorldObjectBayes(model, colocationDist, manage, classRes)
{
  sigma.setIdentity();
  sigma(0,0) = 1000; sigma(1,1) = 1000; sigma(2,2) = 1000; sigma(3,3) = 1000;
} //WorldObjectBayesKalman

WorldObjectBayesKalman::WorldObjectBayesKalman(FloatLookupTable model,
                                                float colocationDist,
                                                WorldObjectManager* manage,
                                                orp::WorldObject classRes,
                                                TypeMap probDistr) :
  WorldObjectBayes(model, colocationDist, manage, classRes, probDistr)
{
  sigma.setIdentity();
  sigma(0,0) = 1000; sigma(1,1) = 1000; sigma(2,2) = 1000; sigma(3,3) = 1000;
} //WorldObjectBayesKalman

bool WorldObjectBayesKalman::mergeIn(WorldObject* other) {
  if(!WorldObjectBayes::mergeIn(other)) {
    return true;
  }
  this->types = other->getTypes();

  for(TypeMap::iterator it = types.begin(); it != types.end(); ++it) {
    std::string wotName = it->first.name;
    // ROS_INFO_STREAM("ZYX: " << getPose().rotation().matrix().eulerAngles(2,1,0));
    // ROS_INFO_STREAM("XYZ: " << getPose().rotation().matrix().eulerAngles(0,1,2));
    // ROS_INFO_STREAM("YXZ: " << getPose().rotation().matrix().eulerAngles(2,1,0));
    // ROS_INFO_STREAM("Pose begins at " << get4DOFPose());

    //Kalman pose/position update.
     ROS_INFO_STREAM("Error covariance projection sigma = " << sigma);

    //noise covariance vector, R_k in literature
    Eigen::Matrix4d R_k;
    R_k.setIdentity();
    R_k(0,0) = .005f; R_k(1,1) = .005f; R_k(2,2) = .005f, R_k(3,3) = 0.005f;

    //identity matrix, I
    Eigen::Matrix4d I;
    I.setIdentity();

    //compute the Kalman gain, K
    Eigen::Matrix4d K = sigma * ( (sigma + R_k).inverse() );
    // ROS_INFO_STREAM("Kalman gain K = " << K);

    Eigen::Matrix3d rotMat = other->getPose(wotName).linear();
    Eigen::Quaterniond otherQuat; otherQuat = rotMat;
    Eigen::Vector4d otherVec = Eigen::Vector4d(otherQuat.w(), otherQuat.x(), otherQuat.y(), otherQuat.z());

    rotMat = getPose(wotName).linear();
    Eigen::Quaterniond thisQuat; thisQuat = rotMat;
    Eigen::Vector4d thisVec = Eigen::Vector4d(thisQuat.w(), thisQuat.x(), thisQuat.y(), thisQuat.z());

    //get pose delta with correctly-calculated angle difference
    Eigen::Vector4d vecDiff = thisVec-otherVec;

    //Update the pose estimate with a measurement
    Eigen::Vector4d newVec = thisVec;
    newVec += K*(vecDiff);

    Eigen::Quaterniond newQuat = Eigen::Quaterniond(newVec(0), newVec(1), newVec(2), newVec(3));

    it->second.pose *= newQuat;

    //Update the error covariance
    sigma *= I-K;

    // ROS_INFO_STREAM("Set pose to " << simplePose);
    // ROS_INFO_STREAM("Pose ends at " << get4DOFPose());

    // ROS_INFO_STREAM("ZYX: " << getPose().rotation().matrix().eulerAngles(2,1,0));
    // ROS_INFO_STREAM("XYZ: " << getPose().rotation().matrix().eulerAngles(0,1,2));
    // ROS_INFO_STREAM("YXZ: " << getPose().rotation().matrix().eulerAngles(2,1,0));
  }
  return true;
} //mergeIn

float WorldObjectBayesKalman::subtractAngle(float angle_1, float angle_2)
{
  if(angle_1-angle_2 > -M_PI && angle_1-angle_2 < M_PI)
    return(angle_1-angle_2);
  else if(angle_1-angle_2 < -M_PI)
    return(angle_1-angle_2+2*M_PI);
  else
    return(angle_1-angle_2-2*M_PI);
} //subtractAngle