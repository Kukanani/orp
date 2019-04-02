
# Setup

Requirements
------------
Currently, the tested and known working version of this software runs with
ROS Kinetic, which requires the following:

 - Ubuntu 16.04 (Xenial)

Use this `link <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_ to go through
the installation and setup process.

These setup instructions assume you have been through the aforementioned setup
process for ROS, have completed at least the basic tutorials, and have setup a
catkin_ws.

Now download the ORP repository from Github.

1. ``cd catkin_ws/src``

2. ``git clone https://github/UTNuclearRobotics/orp.git``

3. ``catkin_make`` (this builds all packages in the workspace)

    a. If this fails, the command should return an error log, likely with the package components dependencies that are missing labeled.

    For example, the following error is provided:

        Could not find a package configuration file provided by "vision_msgs"
        with any of the following names: ...

    Specifically, this package is [here](https://github.com/Kukanani/vision_msgs.git), in which it can be installed following [this guide](https://answers.ros.org/question/252478/how-to-build-a-package-from-source-on-ubuntu-mate-1604-lts-and-ros-kinetic/?answer=252502#post-id-252502).

# TODO: Add a note about catkin_make with correct compiler flags in order to build vision_msgs correctly...    

Run Instructions
----------------

1. Every time you open up a new shell, run the following command to access vital modules (such as catkin CMake).
    ``$ source /opt/ros/kinetic/setup.bash``

    Alternatively, you can add the command as a line to your [.bashrc](https://answers.ros.org/question/206876/how-often-do-i-need-to-source-setupbash/).


2. To test run orp, call ``roslaunch orp example.launch`` (or alternatively, ``roslaunch example.launch`` in the orp/launch folder).
