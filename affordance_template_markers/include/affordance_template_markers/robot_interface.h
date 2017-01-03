/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its
       contributors may be used to endorse or promote products derived from
       this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#ifndef _ROBOT_INTERFACE_H_
#define _ROBOT_INTERFACE_H_

#include <fstream>
#include "yaml-cpp/yaml.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <pluginlib/class_loader.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <planner_interface/planner_interface.h>

#include <end_effector_helper/end_effector_helper.h>

#include <affordance_template_msgs/RobotConfig.h>
#include <affordance_template_msgs/EndEffectorConfig.h>
#include <affordance_template_msgs/EndEffectorPoseData.h>

namespace affordance_template_markers 
{
  class RobotInterface
  {
    ros::NodeHandle nh_;
    
    void jointStateCallback(const sensor_msgs::JointState&);

    sensor_msgs::JointState joint_data_;
    affordance_template_msgs::RobotConfig robot_config_;
    tf::TransformListener listener_;
    boost::shared_ptr<planner_interface::PlannerInterface> robot_planner_;
    pluginlib::ClassLoader<planner_interface::PlannerInterface> planner_loader_;

    bool configured_;
    bool reload_attempted_;
    bool planner_created_;
    std::string config_file_;
    std::string root_frame_;
    
    std::vector<std::string> ee_names_;
    std::vector<std::string> ee_groups_;
    
    std::map<std::string, end_effector_helper::EndEffectorHelperConstPtr> ee_link_data_; 
    std::map<int, std::string> ee_name_map_;    
    std::map<std::string, int> ee_id_map_;
    
    // std::map<std::string, > ee_markers_;
    std::map<std::string, geometry_msgs::Pose> tool_offset_map_;
    std::map<std::string, geometry_msgs::Pose> manipulator_pose_map_;

    std::map<std::pair<std::string, int>, std::string> ee_pose_id_map_;
    std::map<std::pair<std::string, std::string>, int> ee_pose_name_map_;
    std::map<std::pair<std::string, std::string>, sensor_msgs::JointState> stored_poses_;


  public:
    RobotInterface(const ros::NodeHandle& nh);
    RobotInterface(const ros::NodeHandle& nh, const std::string &_joint_states_topic); 
    ~RobotInterface();

    bool load(const std::string&);
    bool load(const affordance_template_msgs::RobotConfig&);
    bool configure();

    inline sensor_msgs::JointState getCurrentJointState() { return joint_data_; }
    inline affordance_template_msgs::RobotConfig getRobotConfig() { return robot_config_; }
    inline std::map<int, std::string> getEENameMap() { return ee_name_map_; }
    inline std::map<std::string, int> getEEIDMap() { return ee_id_map_; }

    std::string getManipulator(const std::string&); 
    std::string getPkgPath(const std::string&);
    
    bool getEELinkData(std::string group_name, end_effector_helper::EndEffectorHelperConstPtr &link_data); 
    
    std::string getEEName(const int);
    std::string getReadableEEName(const std::string&);
    int getEEID(const std::string&);
    
    std::map<std::string, int> getEEPoseIDMap(std::string name);
    std::map<int, std::string> getEEPoseNameMap(std::string name);
    
    std::vector<std::string> getEEPoseNames(std::string name);

    geometry_msgs::Pose getManipulatorOffsetPose(std::string name) { return manipulator_pose_map_[name]; }
    geometry_msgs::Pose getToolOffsetPose(std::string name) { return tool_offset_map_[name]; }

    void tearDown();
    void reset();

    boost::shared_ptr<planner_interface::PlannerInterface> getPlanner() { return robot_planner_; }
  };
}

#endif
