##**Quick Start:**##

We are quickly approaching the first "release" of this software, but in the meantime, here is a quick guide to using it with the Robonaut2 simulator.

###**Dependencies:**###
```
#!bash
$ sudo apt-get install ros-indigo-simulators ros-indigo-moveit* ros-indigo-control* ros-indigo-ros-control* ros-indigo-gazebo-ros-control*
```
NOTE: if you receive a 'broken packages' error, try uninstalling libsdformat2

1. Clone the following repositories into your catkin workspace. A [rosinstall](https://bitbucket.org/traclabs/affordance_templates/downloads/rosinstall-public) is available, or the links below can be followed to download the repositories individually, switching to the specified branches. Note that NASA recently deprecated their [r2 repositories](https://bitbucket.org/nasa_ros_pkg), moving them to 
[gitlab](https://gitlab.com/groups/nasa-jsc-robotics). To accommodate these changes, the repositories below link to forks of the deprecated repositories.

    * [r2 simulator](https://bitbucket.org/traclabs/nasa_r2_simulator/src/dc11506310d0966cbb8b1e6735893f61c2952163/?at=traclabs-devel) - *traclabs-devel*
    * [r2 common](https://bitbucket.org/traclabs/nasa_r2_common/src/9cf739663b3b4af6d939bc711471d9f5f70881f0/?at=traclabs-tracik) - *traclabs-tracik* 
    * [affordance templates](https://bitbucket.org/traclabs/affordance_templates/src/b70502d8ccae1c2afc860a010a2f799fceb45cc9/?at=public) - *public*
    * [robot interaction tools](https://bitbucket.org/traclabs/robot_interaction_tools/src/152eac8f2776e7553c93eb521d69d7963cacac79/?at=public) - *public*

1. `catkin_make` at your workspace level
1. Source each terminal (i.e., `source devel/setup.bash`) - you'll need four:

```
#!bash
    1. roslaunch r2_gazebo r2_gazebo.launch
    2. roslaunch r2_moveit_config move_group.launch
    3. rosrun affordance_template_server affordance_template_server_node
    4. rviz [-d <path_to_rviz_config>]

```
1. Add necessary dispalys in RViz; an [RViz config](https://bitbucket.org/traclabs/affordance_templates/downloads/at_public.rviz) is available.
    * `RobotModel` (robot model) - Robot Description: `robot_description`
    * `InteractiveMarkers` - Update Topic: `/affordance_template_interactive_marker_server/update`
2. Add the *affordance template panel* by going to `Panels->Add New Panel->RVizAffordanceTemplatePanel`


##**Software Overview:**##
![AT Architecture.png](https://bitbucket.org/repo/r5rydq/images/1896767692-AT%20Architecture.png)
