# Table of Contents
1. [Installation](#installation)
1. [Training Objects](#training-objects)
1. [How to Use](#how-to-use)
1. [About](#about)

***

## Installation<a name="installation" />
1. **Download the repository from GitHub**
	* Open a new terminal and enter
        ```
		cd ~/
		git clone https://github.com/UTNuclearRobotics/nrg-ros-support.git
       ```

## Training Objects<a name="training-objects" />
*This setup assumes you have a training table capable of spinning 360 degrees.*

1. Set up the camera to view the table and two AR tags

1. Run in a terminal
	```
	roslaunch orp train.launch
	```

1. Adjust filtering parameters as necessary through rqt_reconfigure. The bounded_scene point cloud should only display the table surface and the object on the table.

1. Run in a terminal
	```
	rosrun train_object.py <your_object_name> <angle_increment_in_degrees>
	```

1. The spin table will spin 360 degrees to allow point cloud data taken for every side of the object.

## How to Use<a name="how-to-use" />

~Coming Soon to Documentation Near You~


## About<a name="about" />
Maintainer: [Adam Allevato](allevato@utexas.edu)

ORP was developed in 2015 in conjunction with UT Austin's Amazon Picking Challenge team and the Nuclear and Applied Robotics Group. We continued to improve the package and add features as we used it throughout multiple projects, including pick-and-place demos and human-robot interaction studies.

We found that robotics groups (especially research labs) tend to re-create simple perception code over and over, often poorly wrapping Point Cloud Library functions in a way that was brittle and hard to reuse. ORP's focus is on flexibility and ease of configuration so that it can be quickly adapted to your needs.
