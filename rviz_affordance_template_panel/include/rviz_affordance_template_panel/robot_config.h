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

#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

using namespace std;

#include <affordance_template_msgs/GripperActionMap.h>

namespace rviz_affordance_template_panel
{

    class EndEffectorConfig
    {
    public:
        EndEffectorConfig(const string& name) {name_=name;};
        ~EndEffectorConfig() {}

        string name() const { return name_; }
        void name(const string& name ) { name_=name; }

        int id() const { return id_; }
        void id(int id) { id_=id; }

        vector<float> pose_offset() const { return pose_offset_; }
        void pose_offset(const vector<float> pose_offset ) { pose_offset_=pose_offset; }

        vector<float> tool_offset() const { return tool_offset_; }
        void tool_offset(const vector<float> tool_offset ) { tool_offset_=tool_offset; }

    private:
        string name_;
        int id_;
        vector<float> pose_offset_;
        vector<float> tool_offset_;  
    };


    class EndEffectorPoseConfig
    {
    public:
        EndEffectorPoseConfig(const string& name) {name_=name;};
        ~EndEffectorPoseConfig() {}

        string name() const { return name_; }
        void name(const string& name ) { name_=name; }

        string group() const { return group_; }
        void group(const string& group ) { group_=group; }

        int id() const { return id_; }
        void id(int id) { id_=id; }

    private:
        string name_;
        string group_;
        int id_;
    };


    class RobotConfig
    {
    public:
        typedef boost::shared_ptr<EndEffectorConfig> EndEffectorConfigSharedPtr;
        typedef boost::shared_ptr<EndEffectorPoseConfig> EndEffectorPoseIDConfigSharedPtr;

        RobotConfig(const string& uid) {uid_=uid;};
        ~RobotConfig() {}

        string uid() const { return uid_; }
        void uid(const string& uid ) { uid_=uid; }

        string name() const { return name_; }
        void name(const string& name ) { name_=name; }

        string frame_id() const { return frame_id_; }
        void frame_id(const string& frame_id ) { frame_id_=frame_id; }

        string config_package() const { return config_package_; }
        void config_package(const string& config_package ) { config_package_=config_package; }

        string config_file() const { return config_file_; }
        void config_file(const string& config_file ) { config_file_=config_file; }

        string planner_type() const { return planner_type_; }
        void planner_type(const string& planner_type ) { planner_type_=planner_type; }

        vector<affordance_template_msgs::GripperActionMap> gripper_action() const { return gripper_action_; }
        void gripper_action(const vector<affordance_template_msgs::GripperActionMap>& gripper_action ) { gripper_action_=gripper_action; }

        vector<float> root_offset() const { return root_offset_; }
        void root_offset(const vector<float> root_offset ) { root_offset_=root_offset; }

        std::map<std::string, EndEffectorConfigSharedPtr> endeffectorMap;
        std::map<std::string, EndEffectorPoseIDConfigSharedPtr> endeffectorPoseMap;


    private:
        string uid_;
        string name_;
        string frame_id_;
        string config_package_;
        string config_file_;
        string planner_type_;
        vector<affordance_template_msgs::GripperActionMap> gripper_action_;
        vector<float> root_offset_;


    };
}

#endif