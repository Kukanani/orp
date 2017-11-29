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

#ifndef _WORLD_OBJECT_MANAGER_H_
#define _WORLD_OBJECT_MANAGER_H_

#include <map>
#include <stdexcept>

#include "orp/core/world_object.h"

/**
 * @brief Keeps track of world object types so that their properties remain
 *        constant.
 */
class WorldObjectManager {
private:
  /// Default object type
  WorldObjectType unknownType;
public:
  /// The possible world object types
  std::map<int, WorldObjectType> types;

  /**
    * Create a new world object manager with the specified default type.
    * @arg unknown the world object to fall back on when nothing can be found.
    */
  WorldObjectManager();

  /// Return how many object types this manager is aware of.
  int getNumTypes() { return types.size(); };

  /// Return how many object types this manager is aware of.
  WorldObjectType getUnknownType() { return unknownType; };

  /// Add an object type to the type list.
  void addType(WorldObjectType wot);

  /// Retrieve a type by name, or the "unknown" object type if not found.
  WorldObjectType& getTypeByName(std::string name);

  /// Retrieve a type by ID, or the "unknown" object type if not found.
  WorldObjectType& getTypeByName(int id);

  /// Load a full set of world object types from the parameter server.
  /// This can be called after loading a yaml file (or sqlite database) of
  /// object properties onto the parameter server. Throws no exceptions, but
  /// if all type parameters can't be found it will assume some default values.
  void loadTypesFromParameterServer();
  /// Attempt to copy a float stored in the parameter server to a different
  /// location in the parameter server.
  static bool attemptToCopyFloatParam(const ros::NodeHandle &node,
      std::string from, std::string to);
  /// Attempt to put a float into the parameter server.
  static bool attemptToSetFloatParam(const ros::NodeHandle &node,
      float val, std::string to);
  /// Attempt to put a load a float from the parameter server.
  static bool attemptToReloadFloatParam(const ros::NodeHandle &node,
      std::string paramName, float &toFill, bool strict = false);
  /// Attempt to put a load a double from the parameter server.
  static bool attemptToReloadDoubleParam(const ros::NodeHandle &node,
      std::string paramName, double &toFill, bool strict = false);
  /// Attempt to put a load a string from the parameter server.
  static bool attemptToReloadStringParam(const ros::NodeHandle &node,
    std::string paramName, std::string &toFill, bool strict = false);

  /// Convert degrees to radians.
  static double radFromDeg(double deg);
  /// Convert radians to degrees.
  static double degFromRad(double rad);
};

#endif