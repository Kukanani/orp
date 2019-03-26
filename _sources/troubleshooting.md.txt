# Troubleshooting

Introduction
------------
ORP is build entirely in a ROS-centric way, which means that you can use
standard ROS debugging tools to figure out what's going wrong with your
program. This page will list several steps you can take to find the exact
location of your issue.

If you find other tools that are helpful for debugging, please feel free to
add them to this page.

All of these commands assume that you are attempting to run your entire vision
pipeline, and that for some reason, it is not working as expected.

Step 1: Are the correct nodes running?
--------------------------------------
First, use the ``rosnode list`` command line tool to check that all nodes are
running as expected. These should be:
- A camera driver, such as openni2
- The ORP ``recognizer`` node, which orchestrates the ORP system
- The segmentation server, ``/segmentation``, if using 3D data
- One or more classifiers (node names vary, but usually contain "``classifier``").

If no camera driver is running, check what launch files you are running and
ensure that you are passing the correct flags to ``orp.launch``, if you are
using it.

If no recognizer and/or segmentation nodes are running, check that you are
launching ``orp.launch`` and that the nodes are not crashing (there should be
terminal output if something goes wrong inside these nodes). If these nodes
are crashing on startup, there may be an ORP internal bug and further digging
may be required (but finish the steps on this page to be sure).

If a built-in (provided with ORP) classifier node is not running, check the
arguments you're passing to ``orp.launch``. If your custom classifier is not
running, check that you are in fact launching it in some way, and that it's not
crashing on startup.

Step 2: Are you providing an object list?
-----------------------------------------
The ORP recognizer requires a list of possible detectable objects to be stored
on the ROS parameter server when it first launches, as a way of understanding
how to interpret the classification messages it receives. You should ensure
that some set of parameters is being loaded, preferably by uploading params
defined in a .yaml file. For example, you may need to add the following line in
one of your launch files:

::

<rosparam command="load" param="items/" file="$(find my_package)/data/database.yaml" />


For an example of how to define a list of objects, please see ORP's
``data/example_database.yaml``.

Step 3: How do the intermediate steps look?
-------------------------------------------
If your recognizer is running, but is publishing empty lists of items, the
next step is to check the ROS services and topics that are part of the
ORP pipeline.
- You should be able to subscribe to your sensor feed, either through the terminal
or by using RViz
- If you are using 3D data, the ``/segmentation`` service should be visible by
running ``rosservice list``.
- If you are using 3D data, the segmentation node should be publishing
"intermediate" point clouds, such as the bounded scene and object clusters. The
publishing of these clouds can be enabled or disabled by using the checkboxes
available in the Dynamic Reconfigure control panel, ``rqt_reconfigure``, which
should have launched with ``orp.launch`` (if it didn't you can run it with the
``rosrun rqt_reconfigure rqt_reconfigure`` command). Once these clouds are being
published, you should be able to visualize them in RViz and see if they match your
expectations. In particular, you may have to adjust the ``spatial`` parameters in
the reconfigure console so that the ``/bounded_scene`` point cloud encompasses your
entire detection region. The other parameters can be used to fine-tune the segmentation
results. in the end, the "all objects" cloud should contain points for each
of the objects you want to detect, and no other points (such as ground,
walls, or large objects in the scene you want to ignore).
- Messages should be being published on the ``/classification`` topic by the
classifier(s), checkable with ``rostopic echo /classification``. If no
object classifications are being published, but the intermediate point clouds
contain useful points for classification (as determined in the previous
step), then there is mostly likely a problem with the classifier
implementation.

Step 4: How does the final output look?
---------------------------------------
The recognizer node should be publishing results to the ROS topic
``/detected_objects``. You can check for this using ``rostopic echo``. Even
if no classifiers are running, messages should be arriving, containing empty
lists.

If no messages are arriving, it's possible that recognition is paused - this
is a feature of ORP that will allow the nodes to run without actually doing
recognition, saving a lot of CPU overhead. To start recognition automatically,
you can specify the ``autostart`` parameter when calling ``orp.launch``. To
start and stop recognition manually (which can be done later, even if the
system is autostarted on launch), publish ``std_msgs::Empty`` messages on the
topics ``/orp_start_recognition`` and ``/orp_stop_recognition``. Here is an
example of how to start recognition from the command line:

::

rostopic pub /orp_start_recognition std_msgs/Empty "{}"


Step 5: You're special
----------------------
If all of the above steps are working, but somehow you are not getting the
results that you want, it's time to ask for help from someone else.
