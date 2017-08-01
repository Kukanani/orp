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

#include "orp/core/grasp_generator.h"

#include <tf/tf.h>

GraspGenerator::GraspGenerator(GripperType g) : gripperType(g) {

}

std::vector<Grasp> GraspGenerator::createGrasps(
    WorldObject target, float approachDist) {
  // For now, all of these are the same, but perhaps in the future they could
  // split into different grasp strategies for different object types
  switch(target.getType().getShape()) {
    case BOX:
      return generateAxisGrabs(target, approachDist);
    case FLAT:
      return generateAxisGrabs(target, approachDist);
    case BLOB:
      return generateAxisGrabs(target, approachDist);
    case CYLINDER:
      return generateAxisGrabs(target, approachDist);
    default:
      return generateAxisGrabs(target, approachDist);
      break;
  }
}

std::vector<Grasp> GraspGenerator::generateAxisGrabs(
    WorldObject target, float approachDist) {
  std::vector<Grasp> graspSet;
  Grasp grasp = Grasp();
  tf::Quaternion orientation = tf::Quaternion();

  tf::Stamped<tf::Pose> offset = target.getPoseTfStamped();

  //approach from positive z
  grasp = Grasp();

  orientation.setEuler(0,M_PI,0);
  grasp.approachPose.getOrigin().setZ(approachDist);

  grasp.graspPose.setRotation(orientation);
  grasp.graspPose.setData(offset * grasp.graspPose);
  grasp.graspPose.frame_id_ = offset.frame_id_;
  grasp.graspPose.setRotation(grasp.graspPose.getRotation().normalize());

  grasp.approachPose.setRotation(orientation);
  grasp.approachPose.setData(offset * grasp.approachPose);
  grasp.approachPose.frame_id_ = offset.frame_id_;
  grasp.approachPose.setRotation(grasp.approachPose.getRotation().normalize());
  graspSet.push_back(grasp);

  //approach from negative x
  grasp = Grasp();
  grasp.graspPose.setData(offset * grasp.graspPose);

  grasp.approachPose.getOrigin().setX(-approachDist);
  grasp.approachPose.setData(offset * grasp.approachPose);
  graspSet.push_back(grasp);

  //approach from negative y
  grasp = Grasp();
  grasp.graspPose.setData(offset * grasp.graspPose);

  grasp.approachPose.getOrigin().setY(-approachDist);
  grasp.approachPose.setData(offset * grasp.approachPose);
  graspSet.push_back(grasp);

  //approach from negative z
  grasp = Grasp();
  grasp.graspPose.setData(offset * grasp.graspPose);

  grasp.approachPose.getOrigin().setZ(-approachDist);
  grasp.approachPose.setData(offset * grasp.approachPose);
  graspSet.push_back(grasp);

  //approach from positive x
  grasp = Grasp();
  grasp.graspPose.setData(offset * grasp.graspPose);

  grasp.approachPose.getOrigin().setX(approachDist);
  grasp.approachPose.setData(offset * grasp.approachPose);
  graspSet.push_back(grasp);

  //approach from positive y
  grasp = Grasp();
  grasp.graspPose.setData(offset * grasp.graspPose);

  grasp.approachPose.getOrigin().setY(approachDist);
  grasp.approachPose.setData(offset * grasp.approachPose);
  graspSet.push_back(grasp);

  return graspSet;
}