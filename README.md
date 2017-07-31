# Table of Contents
1. [Installation](#installation)
2. [Training Objects](#training-objects)
3. [How to Use](#how-to-use)

***

## About
TO BE ADDED LATER.

Maintainer: [Adam Allevato](allevato@utexas.edu "Bill Nye the Science Guy")

## Installation
1. **Generate a RSA Key**
	* Ensure you have been added to the [UT Nuclear Robotics Group](https://github.com/UTNuclearRobotics) on github
	* Open a terminal and type:
        ```
        ssh-keygen -t rsa
        ```
    * Save the key /home/username/.ssh/id_rsa
	* It is not necessary to create a passphrase. Hit enter.
	* Once saved, log into your github account
	* Add the RSA key from /home/username/.ssh/id_rsa to your [github settings](https://github.com/settings/key)

2. **Download the repository from GitHub**
	* Open a new terminal and enter
        ```
		cd ~/
		git clone https://github.com/UTNuclearRobotics/nrg-ros-support.git
       ```
3. **Install AR Track Alvar**
	* Open a terminal and enter
		```
		sudo apt-get install ros-kinetic-ar-track-alvar
		```
	* Follow the [AR Track Alvar](wiki.ros.org/ar_track_alvar) page to generate 3 AR tags and place them on the rotation table. In the following configuration:

	TODO: PLACE IMAGE HERE.

## Training Objects<a name="training-objects" />
*This setup assumes you have a training table capable of spinning 360 degrees.*

1. Set up the camera to view the table and two AR tags

2. Run in a terminal
	```
	roslaunch orp train.launch
	```

3. Adjust filtering parameters as necessary through rqt_reconfigure. The bounded_scene point cloud should only display the table surface and the object on the table.

4. Run in a terminal
	```
	rosrun train_object.py <your_object_name> <angle_increment_in_degrees>
	```

5. The spin table will spin 360 degrees to allow point cloud data taken for every side of the object.

## How to Use<a name="how-to-use" />

~Coming Soon to Documentation Near You~
