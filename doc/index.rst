===============================================================================
Object Recognition and Pose Perception (ORP) Documentation
===============================================================================

.. toctree::
    :hidden:

    extending
    troubleshooting

Overview
--------
ORP is an object recognition library for detecting objects using visual data
(such as information from 2D or 3D cameras). ORP bridges the gap between camera
data and useful information that your robotics application can use. It provides
the following features:

 - A unified framework for different types of visual object detectors
 - Handling of both 2D and 3D data
 - A YAML-based syntax for defining objects detectable with vision
 - An extendable set of basic classifier types
 - Example classifiers
 - A segmentation server that provides tunable versions of common algorithms
   provided by the Point Cloud library
 - Interoperability with ``tf``, ``rqt_reconfigure``, and (coming soon)
   ``vision_msgs``
 - Automatic RViz visualization of detected objects

We realize that every vision use case is different, so a key feature of ORP is
the ability to be easily modified, including tuning recognition algorithm
parameters. ORP uses functionality from the Point Cloud Library and OpenCV. If
you plan on extending or modifying ORP, you should have a good handle on at
least one of those two libraries.

Many features of ORP may seem outdated; this is partly intentional. While
machine learning-based computer vision has seen much success in recent years,
the needs of many robotics applications (such as locating a small object in a
fairly uncluttered environment) are often adequately met by using older vision
techniques, such as edge detection and clustering. With that being said,
because ORP is flexible and modular, it is possible to use newer detectors,
such as the YOLO detector, in the ORP framework, and in fact, it's encouraged
because it allows visual classifiers to be more interoperable.

Structure
---------
ORP is based on ROS, and each component of ORP is its own ROS node. These nodes
can each be modified, swapped, or configured separately, making ORP a powerful,
flexible tool for ROS-based recognition.

The key component of ORP is the ``recognizer``, which aggregates vision
information from any number of classifiers. Classifers can be created by
writing a new C++ class that extends one of the provided Classifier base types,
such as Classifier2d (for image data) or Classifier3d (for point cloud data).
Other input types are also possible under the framework, just not strongly
supported yet.

ORP classifiers can be set to start recognizing as soon as they start, or they
can be started "dormant" and enabled later. You can debug ORP by using
rostopic echo on the /classification and /detected_objects topics.

ORP also provides a single node, the *segmentation server*, which provides many
point cloud processing algorithms out of the box, and has tunable parameters
exposed by the ``rqt_reconfigure`` ROS package.

Project History
---------------
ORP was originally conceived as part of Adam Allevato's masters thesis,
published in 2016 at The University of Texas at Austin, for the Nuclear and
Applied Robotics group's entry in the 2015 Amazon Picking Challenge.
Some of the code was based on earlier point cloud research done by Brian E.
O'Neil.