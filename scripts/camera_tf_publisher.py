#!/usr/bin/env python
#
# Copyright (c) 2016, Adam Allevato
# Copyright (c) 2017, The University of Texas at Austin
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
# IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
An simple, straightforward automatically-calibrated camera transform publisher.
  - Uses AR tags to determine region of interest, and changes camera
    transformation so the region of interest (between the detected tags) is
    centered in the parent frame.

This class could be expanded in the future to include RANSAC-based floor detection, etc.
"""

import rospy
import tf
import math
import numpy
from ar_track_alvar_msgs.msg import AlvarMarkers

class CameraTFPublisher:
    parent_frame = '/world'
    frame        = '/camera_link'
    marker_topic = '/ar_pose_marker'

    cam_pos = (0,0,0)
    cam_quat = (0,0,0,1)

    def __init__(self):
        self.load_params()
        self.tag_sub = rospy.Subscriber(self.marker_topic, AlvarMarkers, self.cb_ar_tags)

        self.listener = tf.TransformListener()
        self.caster = tf.TransformBroadcaster()

        # don't start the timeout timer until the first ar tags come through
        self.started = False

        # publish forever
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.publish_transform()
            r.sleep()

    def start(self):
        self.started = True
        # only run calibration for a certain length of time, (for transients to pass)
        rospy.Timer(rospy.Duration(2), self.cb_time_up, oneshot=True)
        rospy.loginfo("Starting AR tag-based auto calibration.");


    def load_params(self):
        if rospy.has_param('parent_frame'):
            self.parent_frame = rospy.get_param('parent_frame')
        if rospy.has_param('frame'):
            self.frame = rospy.get_param('frame')
        if rospy.has_param('marker_topic'):
            self.marker_topic = rospy.get_param('marker_topic')

    def cb_ar_tags(self, tags):
        if not self.started:
            self.start()

        num_markers = len(tags.markers)
        if num_markers == 2:
            # calculate downward rotation angle. This is 1D angle calibration. To generalize this, you could do a least-squares
            # fit to any number of markers, and then orient that as the ground plane.
            if tags.markers[0].pose.pose.position.z > tags.markers[1].pose.pose.position.z:
                far_pos_msg = tags.markers[0].pose.pose.position
                near_pos_msg = tags.markers[1].pose.pose.position
            else:
                far_marker = tags.markers[1]
                far_pos_msg = tags.markers[1].pose.pose.position
                near_pos_msg = tags.markers[0].pose.pose.position
            
            far_pos = (far_pos_msg.x, far_pos_msg.y, far_pos_msg.z)
            near_pos = (near_pos_msg.x, near_pos_msg.y, near_pos_msg.z)

            dz = far_pos[2] - near_pos[2]
            dx = far_pos[0] - near_pos[0]
            angle = math.atan(dz/dx)

            self.cam_quat = tf.transformations.quaternion_from_euler(0, angle, 0)

            # note that the different tags are used for different coordinates. This line is currently application-specific:
            # one tag to the right of the interest point, one tag directly in front

            # first transform tags into the parent frame
            if self.listener.canTransform(self.parent_frame, self.frame, rospy.Time(0)):
                transformation = self.listener.lookupTransform(self.parent_frame, self.frame, rospy.Time(0))
                pos = numpy.matrix(tf.transformations.translation_matrix(transformation[0]))
                quat = numpy.matrix(tf.transformations.quaternion_matrix(transformation[1]))

                interest_point = tf.transformations.translation_from_matrix(pos * quat * tf.transformations.translation_matrix(far_pos))

                self.cam_pos = (self.cam_pos[0]-interest_point[0], -tags.markers[0].pose.pose.position.y, self.cam_pos[2]-interest_point[2])
            else:
                rospy.loginfo("Can't transform into target frame")
        else:
            # didn't have 2 markers so skip this result. This branch is here for clarity
            pass

    def lock_transform(self):
        rospy.loginfo('calibrated transform is locked in.')
        self.tag_sub.unregister()

    def publish_transform(self):
        self.caster.sendTransform(self.cam_pos, self.cam_quat, rospy.Time.now(), self.frame, self.parent_frame)

    def cb_time_up(self, event):
        self.lock_transform()

if __name__ == '__main__':
    try:
        rospy.init_node('camera_tf_publisher')
        pub = CameraTFPublisher()
    except rospy.ROSInterruptException:
        pass