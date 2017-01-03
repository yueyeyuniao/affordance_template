##**Quick Start:**##

We are quickly approaching the first "release" of this software, but in the meantime, here is a quick guide to using it with the Robonaut2 simulator.

###**Dependencies:**###
```
#!bash
$ sudo apt-get install ros-indigo-simulators ros-indigo-moveit* ros-indigo-control* ros-indigo-ros-control* ros-indigo-gazebo-ros-control*
```
NOTE: if you receive a 'broken packages' error, try uninstalling libsdformat2

####**Testing on Robonaut 2 upperbody simulator:**####

1. checkout the following repositories into your catkin workspace to run R2 upperbody and switch to the specified branches.

    * [r2 simulator](https://bitbucket.org/nasa_ros_pkg/nasa_r2_simulator/src/c32521004a4a8f135c4298500d6ded3ce20e0070/?at=indigo-devel) - *traclabs-devel*
    * [r2 common](https://bitbucket.org/nasa_ros_pkg/nasa_r2_common/src/41b52f1747bdb0b484fb1c3788716c950d8e5d0e/?at=traclabs-devel) - *traclabs-tracik* 
    * current [robot interaction tools](https://bitbucket.org/traclabs/robot_interaction_tools/src/faaaa732baf71a8340dfd6a24288824a7ae05cb4/?at=cpp-devel) development - *testing*

1. catkin_make at your workspace level
1. source each terminal - you'll need three

```
#!bash
    1. roslaunch r2_gazebo r2_gazebo.launch
    2. roslaunch interactive_controls r2_upperbody.launch
    3. rviz

```
1. add necessary topics in RViz
    * RobotModel (robot current state) - Robot Description: robot_description
    * RobotModel (path visualization) - TF Prefix: rit
    * interactive marker - Update Topic: /r2_interactive_controls_server/update
2. add InteractiveControls panel by going to Panels->Add New Panel->RVizInteractiveControlsPanel


####**Testing on Fetch simulator:**####

1. Follow the instructions [here](http://docs.fetchrobotics.com/gazebo.html) to install the Fetch Gazebo simulator

    * [fetch_moveit_config](https://bitbucket.org/zqsui/fetch_moveit_config) - *feature/modified_srdf*
    * current [robot interaction tools](https://bitbucket.org/traclabs/robot_interaction_tools/src/faaaa732baf71a8340dfd6a24288824a7ae05cb4/?at=cpp-devel) development - *cpp-devel*


1. catkin_make at your workspace level
1. source each terminal - you'll need three

```
#!bash
    1. roslaunch fetch_gazebo simulation.launch
    2. roslaunch interactive_controls fetch.launch
    3. rviz

```
1. add necessary displays in RViz
    * RobotModel (robot current state) - Robot Description: robot_description
    * RobotModel (path visualization) - TF Prefix: rit
    * interactive marker - Update Topic: /fetch_interactive_controls_server/update
2. add InteractiveControls panel by going to Panels->Add New Panel->RVizInteractiveControlsPanel