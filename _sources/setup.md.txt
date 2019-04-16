
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

    The following are possible errors that might occur when setting up and building the program.

    a.  ``The program 'catkin_make' is currently not installed...``

    The solution to this program is to follow step 1 in the **run** instructions.

    b.  If the make fails, the command should return an error log, likely with the package components dependencies that are missing labeled.

    For example, the following error is provided:

        Could not find a package configuration file provided by "vision_msgs"
        with any of the following names: ...

    Specifically, this package is [here](https://github.com/Kukanani/vision_msgs.git), in which it can be installed following [this guide](https://answers.ros.org/question/252478/how-to-build-a-package-from-source-on-ubuntu-mate-1604-lts-and-ros-kinetic/?answer=252502#post-id-252502).

    c. ``Make -j8 -l8 failed`` plus indefinite stalling:

    This bug has been identified so far on SIM lab's Dell XPS 13 9360 Laptops with Intel's octocore i5-8250U CPUs.

    A proven fix is appending the ``-j1 -l1`` flag to the command.

4. ``. ~/catkin_ws/devel/setup.bash`` (Add the rebuilt workspace to the ROS environment)

5. **Sanity Check:** To see whether the build process and setup has worked, try ``rospack depends1 [no 3rd arg]``. Display all the possibilities, and ``orp`` should appear as one of the dependencies. Alternatively, if the above command gives an error, like ``Error: no package given``, try calling ``rospack depends1 orp`` to see if dependencies show up.

If not, the following steps won't work. (I'm not quite sure of a precise fix, but possible solutions include re-sourcing the env setup file in setp 4, or running ``catkin clean`` and rebuilding the entire workspace.)

6. **Refresh:** If the workspace no longer can recognize orp as a package, try step 4 again, and if that fails, ``catkin clean`` and restarting from step 3.


Run Instructions
----------------

1. Every time you open up a new shell, run the following command to access vital modules (such as catkin CMake).
    ``$ source /opt/ros/kinetic/setup.bash``

    Alternatively, you can add the command as a line to your [.bashrc](https://answers.ros.org/question/206876/how-often-do-i-need-to-source-setupbash/).

2. Start up roscore. ``roscore``

3. To test run orp, call ``roslaunch orp example.launch`` (or alternatively, ``roslaunch example.launch`` in the orp/launch folder).

    a. if this step fails because it cannot find the orp package, try step 6 in the setup instructions.

##### Note:

When running example.launch, if this is the first time setup, no data should be published to the window because the camera is not connected.

In the specific case of SIMLab, we are using an Intel Realsense Depth Camera, model D435. In order to run this, we need both
    1. the correct drivers for the camera installed and
    2. the camera enabled in example.launch to auto connect with the drivers.

1.  Install actual driver   https://github.com/IntelRealSense/librealsense/releases TODO
    Install ros driver TODO

2.  To setup the camera enabled, look at orp.launch in the same launch folder. Inside it, you should see a list of arguments for possible camera drivers like below:

```
<arg name="openni"          default="false"/>
<arg name="openi2"          default="false"/>
<arg name="realsense_r200"  default="false"/>
<arg name="realsense_4xx"   default="false"/>
<arg name="astra"           default="false"/>
```

Copy the realsense_4xx line and put it in example.orp where the default line where realsense_r200 is.

Rerun step 3.
