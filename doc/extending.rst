=============
Extending ORP
=============
.. toctree::

When to Build a Custom Classifier
---------------------------------
In short, you should build a custom classifier when current classifiers don't
fit your needs, and you are looking to generate recognition results for 2D or
3D visual input data.

How to Build a Custom Classifier
--------------------------------
Begin by extending one of the ORP Classifier base classes, such as Classifier2d
or ClassifierNN (for Nearest Neighbors). These classes will automatically set
up the proper types of subscribers and node handles for you to use. Then,
override the ``cb_classify`` callback functions and implement your algorithm
inside. The classifier should publish ORP WorldObject messages to the
``/classification`` topic to be caught by the recognizer.
``src/sixdof_classifier.cpp`` is a good example to look at for how to build
and publish the WorldObject message.

Take special care with your tf frames. If your visual data is 3D, it should
have coordinate information contained within, and the segmentation server
may change what frame the points are expressed in. Before setting up the
header of your newly detected object's message, check the frame used by the
incoming data.

How to Use a Custom Classifier
------------------------------
Build your classifier like any other ROS node, using CMakeLists and other
standard practices. Run your node, and also run ``orp.launch`` as described in
the Usage page, providing no classifier-specific arguments.
Assuming that the recognizer node is running as expected, and that your node
is publishing objects on /classification, the pipeline should be working. See
the Troubleshooting page if you're having issues.