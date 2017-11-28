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

#include "orp/core/world_object_manager.h"
#include "orp/core/world_object.h"

WorldObjectManager::WorldObjectManager() : unknownType("unknown") {
  addType(unknownType);
} //WorldObjectManager

void WorldObjectManager::addType(WorldObjectType wot) {
  // we need this check/case because not using C++17, and WorldObjectType has
  // no default constructor (which [] requires). See
  // http://stackoverflow.com/questions/695645/why-does-the-c-map-type-argument-require-an-empty-constructor-when-using

  if(types.find(wot.getName()) != types.end()) { //already exists
    types.find(wot.getName())->second = wot;
  }
  else {
    types.insert(std::pair<std::string, WorldObjectType>(wot.getName(),wot));
  }
} //addType

WorldObjectType& WorldObjectManager::getTypeByName(std::string name) {
  std::map<std::string, WorldObjectType>::iterator found = types.find(name);
  if(found != types.end()) {
    return found->second;
  }
  throw std::logic_error("did not find object named '" + name + "'");
  return unknownType;
} //getTypeByName

void WorldObjectManager::loadTypesFromParameterServer() {
  ros::NodeHandle n;

  XmlRpc::XmlRpcValue paramMap;
  std::vector<WorldObjectType> objects;
  while(!n.getParam("/orp/items", paramMap)) {
    ROS_INFO_DELAYED_THROTTLE(5.0,
      "Waiting for object type list on parameter server...");
    ros::Duration(1.0).sleep();
  }
  for(auto it = paramMap.begin(); it != paramMap.end(); ++it)
  {
    std::string objName = (*it).first;
    WorldObjectType thisType = WorldObjectType(objName);
    double x, y, z, roll = 0.0, pitch = 0.0, yaw = 0.0;
    double x_off = 0.0, y_off = 0.0, z_off = 0.0;
    float r = 0.5, g = 0.5, b = 0.5;
    ObjectShape shape;

    try {
      std::string geom;
      WorldObjectManager::attemptToReloadStringParam(n,
        "/orp/items/" + objName + "/geometry", geom);
      if(geom == "BOX") {
        shape = BOX;
      } else if(geom == "CYLINDER") {
        shape = CYLINDER;
      } else if(geom == "FLAT") {
        shape = FLAT;
      } else if(geom == "BLOB") {
        shape = BLOB;
      } else {
        ROS_ERROR_STREAM("Did not understand geometry type " <<
          geom << " while creating marker stubs");
        shape = BLOB;
      }
      WorldObjectManager::attemptToReloadDoubleParam(
        n, "/orp/items/" + objName + "/depth", x, true);
      WorldObjectManager::attemptToReloadDoubleParam(
        n, "/orp/items/" + objName + "/width", y, true);
      WorldObjectManager::attemptToReloadDoubleParam(
        n, "/orp/items/" + objName + "/height", z, true);

      WorldObjectManager::attemptToReloadDoubleParam(
        n, "/orp/items/" + objName + "/roll", roll);
      WorldObjectManager::attemptToReloadDoubleParam(
        n, "/orp/items/" + objName + "/pitch", pitch);
      WorldObjectManager::attemptToReloadDoubleParam(
        n, "/orp/items/" + objName + "/yaw", yaw);
      // roll = WorldObjectManager::radFromDeg(roll);
      // pitch = WorldObjectManager::radFromDeg(pitch);
      // yaw = WorldObjectManager::radFromDeg(yaw);

      WorldObjectManager::attemptToReloadFloatParam(
        n, "/orp/items/" + objName + "/red", r);
      WorldObjectManager::attemptToReloadFloatParam(
        n, "/orp/items/" + objName + "/green", g);
      WorldObjectManager::attemptToReloadFloatParam(
        n, "/orp/items/" + objName + "/blue", b);

    } catch(std::exception e) {
      ROS_ERROR_STREAM("error while creating marker stub for world object of "
                << "type '" << e.what() << "': " << (objName).c_str());
      x = 0.1; y = 0.1; z = 0.1;
      shape = BLOB;
    }
    if(x > 1 && y > 1 && z > 1) { //detect sizes in mm instead of m
      ROS_WARN_STREAM("While loading " << objName << ", I found a dimension"
               << "greater than 1 meter. "
               << "Since that's highly unlikely for our tasks, I'm going to "
               << "scale it down by 1000, assuming that the dimension is in "
               << " mm.");
      x /= 1000.0f;
      y /= 1000.0f;
      z /= 1000.0f;
    }
    thisType.setShape(shape);
    thisType.setColor(r,g,b);
    thisType.setSize(x,y,z);
    thisType.setOffset(roll,pitch,yaw);

    addType(thisType);
  }
//   ROS_INFO("The world object loader has generated " << getNumTypes()
//   << " visualization marker stubs.", getNumTypes());
}

double WorldObjectManager::radFromDeg(double deg) {
  return deg * M_PI / 180.0;
}

double WorldObjectManager::degFromRad(double rad) {
  return rad * 180.0 / M_PI;
}

bool WorldObjectManager::attemptToCopyFloatParam(const ros::NodeHandle &node,
                                                 std::string from,
                                                 std::string to)
{
  double temp;
  if(!node.hasParam(from)) {
    ROS_ERROR_STREAM("source param " << from << " not found while attempting "
              << "to copy to " << to);
    throw std::runtime_error("Source param does not exist");
    return false;
  } if(!node.hasParam(to)) {
    ROS_ERROR_STREAM("target param " << to << " not found while attempting "
              << "to copy from " << from);
    throw std::runtime_error("Target param does not exist");
    return false;
  }
  node.getParam(from, temp);
  node.setParam(to, temp);
  return true;
}

bool WorldObjectManager::attemptToSetFloatParam(const ros::NodeHandle &node,
                                                float val, std::string to)
{
  if(!node.hasParam(to)) {
    ROS_ERROR_STREAM("target param " << to << " not found while attempting to "
              << "set value = " << val);
    throw std::runtime_error("Target param does not exist");
    return false;
  }
  node.setParam(to, (double)val);
  return true;
}

bool WorldObjectManager::attemptToReloadFloatParam(const ros::NodeHandle &node,
                                                   std::string paramName,
                                                   float &toFill, bool strict)
{
  double temp;
  if(!node.hasParam(paramName)) {
    if(strict) {
      ROS_ERROR_STREAM("can't find parameter " << paramName.c_str());
      throw std::runtime_error("Source param does not exist");
    }
    return false;
  }
  node.getParam(paramName, temp);
  toFill = temp;
  return true;
}

bool WorldObjectManager::attemptToReloadDoubleParam(
    const ros::NodeHandle &node, std::string paramName, double &toFill,
    bool strict)
{
  if(!node.hasParam(paramName)) {
    if(strict) {
      ROS_ERROR_STREAM("can't find parameter " << paramName.c_str());
      throw std::runtime_error("Source param does not exist");
    }
    return false;
  }
  node.getParam(paramName, toFill);
  return true;
}

bool WorldObjectManager::attemptToReloadStringParam(
    const ros::NodeHandle &node, std::string paramName,std::string &toFill,
    bool strict)
{
  if(!node.hasParam(paramName)) {
    if(strict) {
      ROS_ERROR_STREAM("can't find parameter " << paramName.c_str());
      throw std::runtime_error("Source param does not exist");
    }
    return false;
  }
  node.getParam(paramName, toFill);
  return true;
}