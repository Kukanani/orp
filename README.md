# Table of Contents
1. [Installation](#installation)
1. [Run Example](#run-example)
1. [How to Use](#how-to-use)
1. [About](#about)

***

## Installation<a name="installation" />
1. Download the repository from GitHub
```
cd catkin_ws/src
git clone https://github.com/UTNuclearRobotics/orp.git
```
1. Install dependencies

You may need to install camera drivers depending on what type of camera you are using. See the [package.xml](https://github.com/UTNuclearRobotics/orp/blob/kinetic/package.xml) for a list of additional dependencies.

1. Build your catkin workspace

## Test<a name="run-example" />
```
roslaunch orp example.launch
```

## How to Use<a name="how-to-use" />
Coming soon


## About<a name="about" />
Maintainer: [Adam Allevato](allevato@utexas.edu)

ORP was developed in 2015 in conjunction with UT Austin's Amazon Picking Challenge team and the Nuclear and Applied Robotics Group. We continued to improve the package and add features as we used it throughout multiple projects, including pick-and-place demos and human-robot interaction studies.

We found that robotics groups (especially research labs) tend to re-create simple perception code over and over, often poorly wrapping Point Cloud Library functions in a way that was brittle and hard to reuse. ORP's focus is on flexibility and ease of configuration so that it can be quickly adapted to your needs.
