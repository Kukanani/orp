#!/bin/bash
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


# Uses the cpulimit utility to forcibly restrict nodelet-based nodes to a
# certain % of cpu. This was originally designed to stop camera drivers
# from consuming all CPU on low-performance systems.

# TODO(Kukanani): there are several problems with this script:
#
#   1. It throttles all nodelets, not just camera drivers
#   2. It uses cpulimit, and there's no guarantee that the user has this
#      on there system (it's not a system package, you have to apt install it).
#   3. Using cpulimit to restrict a process is pretty hacky, and it will
#      result in reduced framerates while the node is forced to sleep.
#
#   If this script is removed, be sure to update the launch files that
#   reference it so that they don't use this script.

node_process_id=$(pidof nodelet)
while [ -z "$node_process_id" ]
do
    sleep 1
    node_process_id=$(pidof nodelet)
done

cpulimit --lazy --limit $1 --exe nodelet
