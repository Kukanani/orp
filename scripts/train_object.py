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
Call the service to train an object using the pan table.
"""

import rospy
import sys
from orp.srv import DataCollect

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print 'Cant train object. Syntax is: train_object.py object_name angle_increment_in_degreees'
        exit()

    try:
        angle_inc = int(sys.argv[2])
    except ValueError:
        print "The angle increment must be an integer."
        exit()

    if angle_inc < 1 or angle_inc > 360:
        print "Error: greater than 0 and no greater than 360."
        exit()

    try:
        rospy.init_node('camera_tf_publisher')
        train = rospy.ServiceProxy('/pan360_data_collect', DataCollect)
        response = train(sys.argv[1], int(sys.argv[2]))

    except rospy.ROSInterruptException:
        pass    