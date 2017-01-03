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

#ifndef INTERACTIVE_CONTROLS_H_
#define INTERACTIVE_CONTROLS_H_

#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>
#include <kdl_conversions/kdl_msg.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <rit_utils/marker_helper.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <planner_interface/planner_interface.h>
#include <end_effector_helper/end_effector_helper.h>

#include <robot_interaction_tools_msgs/SetPose.h>
#include <robot_interaction_tools_msgs/AddGroup.h>
#include <robot_interaction_tools_msgs/RemoveGroup.h>
#include <robot_interaction_tools_msgs/ExecuteCommand.h>
#include <robot_interaction_tools_msgs/JointPlanCommand.h>
#include <robot_interaction_tools_msgs/EndEffectorCommand.h>
#include <robot_interaction_tools_msgs/CartesianPlanCommand.h>
#include <robot_interaction_tools_msgs/GetGroupConfiguration.h>
#include <robot_interaction_tools_msgs/PlanGroupConfiguration.h>
#include <robot_interaction_tools_msgs/ConfigurePlanningService.h>
#include <robot_interaction_tools_msgs/ConfigureGroup.h>

#include <boost/algorithm/string.hpp>

namespace interactive_controls
{

  class InteractiveControls
  {

    public:
      /** @brief constructor.
      *
      */
      InteractiveControls(const ros::NodeHandle& nh, std::string robot_name, std::string planner_type);
      ~InteractiveControls();

      void spin();

      // don't like this way of doing it
      typedef char GroupType;
      GroupType JOINT = 0;
      GroupType CARTESIAN = 1;
      GroupType ENDEFFECTOR = 2;
      
      inline GroupType stringToGroupType(const std::string &str)
      {
        if (boost::iequals(str, groupTypeToString(JOINT)))
          return JOINT;
        else if (boost::iequals(str, groupTypeToString(CARTESIAN)))
          return CARTESIAN;
        else if (boost::iequals(str, groupTypeToString(ENDEFFECTOR)))
          return ENDEFFECTOR;
      }

      inline std::string groupTypeToString(const GroupType &grp)
      {
        if (grp == JOINT)
          return "joint";
        else if (grp == CARTESIAN)
          return "cartesian";
        else if (grp == ENDEFFECTOR)
          return "endeffector";
      }

      inline std_msgs::ColorRGBA getColorMsg(float r, float g, float b, float a=1.0) {
        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
      }

      inline geometry_msgs::Vector3 scaleVector3Msg(geometry_msgs::Vector3 v, double sf) {
        geometry_msgs::Vector3 v_out;
        v_out.x = v.x*sf;
        v_out.y = v.y*sf;
        v_out.z = v.z*sf;
        return v_out;
      }

      inline std::string getEEFrameName(std::string group_name) {
        return group_name + "/interactive_controls_frame/ee";
      }
      
      inline std::string getToolFrameName(std::string group_name) {
        return group_name + "/interactive_controls_frame/tool";
      }

      // menu config will hold the menu text as well as bool for whether it should have a checkbox
      typedef std::pair<std::string, bool> MenuConfig;

      // this will handle menus. first item is group name, vector list is nested menu text(s)
      typedef std::map<std::string, std::vector<std::string> > MenuHandleKey;

      // this is a storage DS to store "named" pose info. i.e., frame 'first' in pose.header.frame_id frame 
      typedef std::pair<std::string, geometry_msgs::PoseStamped> FrameInfo;


    protected:
      boost::shared_ptr<planner_interface::PlannerInterface> planner_;
      pluginlib::ClassLoader<planner_interface::PlannerInterface> planner_loader_;
      std::map<std::string, end_effector_helper::EndEffectorHelperConstPtr> ee_helper_map_;
      std::map<std::string, InteractiveControls::FrameInfo> frame_store_;

    private:
  
      // ros variables    
      ros::NodeHandle nh_;
      ros::ServiceServer config_srv_, get_info_srv_;
      ros::Publisher im_pub_;
      
      double loop_rate_;
      tf::TransformListener listener_;
      tf::TransformBroadcaster tf_broadcaster_;

      ros::Publisher ee_pub_;

      std::string robot_name_;
      std::string planner_plugin_;
            
      boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

      std::map<std::string, visualization_msgs::InteractiveMarker> markers_;
      std::map<std::string, std::map<std::string, visualization_msgs::InteractiveMarker> > posture_markers_;
      std::map<std::string, interactive_markers::MenuHandler> marker_menus_;
      std::map<std::string, GroupType> group_type_map_;
      std::vector<std::string> active_groups_;

      std::map<MenuHandleKey, interactive_markers::MenuHandler::EntryHandle> group_menu_handles_;

      std::map<std::string, std::string> control_frames_;
      std::map<std::string, geometry_msgs::Pose> tool_offsets_;

      std::map<std::string, bool> auto_execute_;
      std::map<std::string, bool> auto_plan_;
      std::map<std::string, bool> show_path_;
      std::map<std::string, bool> show_posture_markers_;
      std::map<std::string, bool> plan_cartesian_paths_;
      std::map<std::string, std::string> conditioning_metrics_;
      std::map<std::string, geometry_msgs::Pose> task_compatibility_vectors_;

      std::vector<MenuConfig> menu_options_;
      std::vector<MenuConfig> joint_menu_options_;
      std::vector<MenuConfig> cartesian_menu_options_;
      std::vector<MenuConfig> endeffector_menu_options_;

      std::map<std::string, std::map<std::string, sensor_msgs::JointState> > stored_pose_map_;
      
      bool auto_display_;

      // tolerances
      bool use_tolerances_;
      double joint_tolerance_;
      double position_tolerances_;
      double orientation_tolerances_;
      std::vector<std::string> tolerance_modes_;
      std::map<std::string, std::vector<double> > group_position_tolerances_;
      std::map<std::string, std::vector<double> > group_orientation_tolerances_;
      std::map<std::string, std::string> group_position_tolerance_type_;
      std::map<std::string, std::string> group_orientation_tolerance_type_;

      // service calls
      bool configService(robot_interaction_tools_msgs::ConfigureGroup::Request  &req,
                         robot_interaction_tools_msgs::ConfigureGroup::Response &res);
      bool getInfoService(robot_interaction_tools_msgs::GetGroupConfiguration::Request  &req,
                          robot_interaction_tools_msgs::GetGroupConfiguration::Response &res);

      void getGroupConfiguration(std::string group_name, robot_interaction_tools_msgs::PlanGroupConfiguration &group_configuration);
      void getGroupConfigurations(std::vector<robot_interaction_tools_msgs::PlanGroupConfiguration> &group_configurations);

      bool addPlanningGroup(robot_interaction_tools_msgs::PlanGroupConfiguration config);
      bool removePlanningGroup(robot_interaction_tools_msgs::PlanGroupConfiguration config);
      bool executePlan(robot_interaction_tools_msgs::PlanGroupConfiguration config);

      bool planToGoal(const std::string &group_name, const geometry_msgs::PoseStamped &pt);

      bool doStoredPose(std::string group_name, std::string pose_name, bool execute);
      bool planToMarker(std::string group_name);

      std::string getTypeString(const GroupType &type);
      GroupType getTypeFromString(const std::string &type_string);

      void parseGroupsFromParam();
      void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
      void toolOffsetCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
      void storedPoseCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
      void jointMaskCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
      void positionToleranceCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
      void orientationToleranceCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

      bool setupGroups();
      bool setupGroup(const std::string &group_name, GroupType group_type);

      // menu helper methods
      void setupMenuOptions();

      void setupJointMenus(const std::string& group_name);
      void setupEndEffectorMenus(const std::string& group_name);
      void setupCartesianMenus(const std::string& group_name);

      void setupStoredPoseMenu(const std::string& group_name);
      void setupPositionToleranceMenu(const std::string& group_name);
      void setupOrientationToleranceMenu(const std::string& group_name);

      void setupJointMaskMenu(const std::string& group_name);
      void setupToolOffsetMenu(const std::string& group_name);
      void setupSimpleMenuItem(const std::string& group_name, const std::string& menu_text, bool has_check_box);

      bool setSubMenuCheckState(std::string group_name, std::string menu_text, bool val);
      bool setJointMaskMenuCheckState(std::string group_name, std::string jnt, bool val);

      bool removeGroupMarkers(const std::string& group_name);

      bool initializeGroupMarkers(const std::string &group_name);
      bool initializeCartesianGroup(const std::string &group_name);
      bool initializeJointGroup(const std::string &group_name);
      bool initializeEndEffectorGroup(const std::string &group_name);

      bool resetCartesianGroupMarker(const std::string &group_name, double delay=0);
      void setCartesianEEColor(std::string group_name, std_msgs::ColorRGBA c, float scale_f);

      geometry_msgs::PoseStamped applyToolOffset(const std::string &group_name, const geometry_msgs::PoseStamped &ps);    

      bool isKnownGroupMarker(const std::string &group_name);
      bool isInTypeMap(const std::string &group_name);
      bool isToleranceMode(const std::string &mode);
      bool isGroup(const std::string &group_name);
      bool isActiveGroup(const std::string &group_name);
      bool isStoredState(const std::string &group_name, const std::string &state_name);


    
    };  



};  
#endif
