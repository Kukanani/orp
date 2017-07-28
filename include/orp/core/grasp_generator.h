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

#ifndef GRASP_GENERATOR_H
#define GRASP_GENERATOR_H

#include <vector>

#include "orp/core/world_object.h"
#include "orp/core/grasp.h"

/**
 * @brief Generates basic grasps for objects.
 *
 * This was part of a now-dormant attempt to have ORP objects be more than
 * just vision objects, and have grasp generation tied closely to the object
 * definition. This doesn't make sense for a number of reasons, the most
 * important of which is that it creates a lot of unnecessary coupling between
 * the vision and grasping code. If there was a package or set of packages
 * that defined an agnostic "objectness" concept, then vision pipelines could
 * produce objects, and grasp generators could act on them. For an attempt
 * at this latter approach, see
 *
 * All that to say, that this class, along with the Grasp struct, could and
 * should be removed. HOWEVER the multiuse_workcell class currently uses the
 * grasp generator in a limited capacity (see sorter.cpp), so that
 * implementation would have to be modified as a result of removing this class.
 *
 * It may be better to save this change for "ORP 2" or something similar.
 *
 * @author Adam Allevato
 * @email allevato@utexas.edu
 * @date 2016-01-12
 * @version 0.1.0
 */

/**
 * Different gripper types that the generator can generate grasps for.
 * (each gripper's geometry is different)
 */
enum GripperType {
  UNDEFINED       = 0,
  ROBOTIQ_S_MODEL = 1
};

/**
 * Given objects, generates grasps.
 */
class GraspGenerator {
protected:
  /// What type of gripper to use
  GripperType gripperType;

  /**
   * Generate grasps from all 8 cartesian axis directions (+/- X/Y/Z).
   */
  std::vector<Grasp> generateAxisGrabs(WorldObject target, float approachDist);
public:
  /**
   * Basic constructor
   *
   * @param g the type of gripper to use for generation
   */
  GraspGenerator(GripperType g = UNDEFINED);

  /**
   * Generate grasps for the object in question. Different object shapes, for
   * example, may require different grasp generation strategies.
   */
  std::vector<Grasp> createGrasps(WorldObject target, float approachDist);
};

#endif