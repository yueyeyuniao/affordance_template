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

#ifndef _AFFORDANCE_TEMPLATE_H_
#define _AFFORDANCE_TEMPLATE_H_

#include <ros/ros.h>
#include <ros/package.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>

#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <rit_utils/marker_helper.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/RobotState.h>
#include <actionlib/server/simple_action_server.h>

#include <planner_interface/planner_interface.h>

#include <affordance_template_markers/robot_interface.h>

#include <affordance_template_library/affordance_template_structure.h>
#include <affordance_template_library/affordance_template_parser.h>

#include <affordance_template_msgs/PlanAction.h>
#include <affordance_template_msgs/ExecuteAction.h>
#include <affordance_template_msgs/DisplayObjectInfo.h>
#include <affordance_template_msgs/WaypointViewMode.h>


namespace affordance_template 
{

  struct WaypointTrajectoryFlags {
    bool run_backwards;
    bool auto_execute;
    bool loop;
    std::map<std::string, bool> controls_on;
    std::map<std::string, bool> compact_view;
    std::map<std::string, bool> adjust_offset;
    std::map<std::string, bool> move_offset;
  };

  struct PlanStatus {
    std::vector<int> sequence_ids;
    std::vector<geometry_msgs::PoseStamped> sequence_poses;
    bool backwards;
    bool plan_valid;
    bool exec_valid;
    bool direct;
    int current_idx = -1;
    int goal_idx = -1;
  };

  enum PlanningGroup { MANIPULATOR, EE };

  struct ContinuousPlan 
  {
    int step; // =0 means do max steps
    std::string group; // left_arm, left_hand, etc
    PlanningGroup type;
    sensor_msgs::JointState start_state;
    moveit::planning_interface::MoveGroup::Plan plan;
  };

  class AffordanceTemplate
  { 

  public:
    AffordanceTemplate(const ros::NodeHandle nh, 
                       boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server, 
                       std::string robot_nkame, 
                       std::string template_type,
                       int id);
    AffordanceTemplate(const ros::NodeHandle nh, 
                       boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server, 
                       boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface,
                       std::string robot_name, 
                       std::string template_type,
                       int id);
    ~AffordanceTemplate();

    // public methods used by server node
    void run();
    void stop();
    bool addTrajectory(const std::string&);
    bool continuousMoveToWaypoints(const std::string&, const std::string&);
    bool moveToWaypoints(const std::vector<std::string>&);
    bool saveToDisk(std::string&, const std::string&, const std::string&, bool);
    bool loadFromFile(std::string filename, geometry_msgs::Pose pose, affordance_template_object::AffordanceTemplateStructure &structure);
    bool buildTemplate(std::string traj="");

    // public getters
    inline int getID() { return id_; }
    inline std::string getType() { return template_type_; }
    inline std::string getCurrentTrajectory() { return current_trajectory_; }
    inline affordance_template_object::AffordanceTemplateStructure getCurrentStructure() { return structure_; }
    inline affordance_template_object::AffordanceTemplateStructure getDefaultStructure() { return initial_structure_; }
    inline boost::shared_ptr<affordance_template_markers::RobotInterface> getRobotInterface() { return robot_interface_; }

    int getNumWaypoints(const std::string traj_name, const int ee_id);
    bool getTrajectoryPlan(const std::string&, const std::string&, PlanStatus&);
    bool getWaypointFlags(const std::string& traj, WaypointTrajectoryFlags& flags);

    // public setters 
    bool switchTrajectory(const std::string&);
    bool setTrajectory(const std::string&);
    bool setObjectScaling(const std::string&, double, double);
    void setRobotInterface(boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface);
    bool setObjectPose(const affordance_template_msgs::DisplayObjectInfo&);
    bool setWaypointViewMode(int ee, int wp, bool m);
    bool setTemplatePose(geometry_msgs::PoseStamped ps);
    
    bool getPoseFromFrameStore(const std::string &frame, geometry_msgs::PoseStamped &ps);
    
    std::string getToolPointFrameName(std::string wp_name) { return wp_name + "/tp"; }
    std::string getControlPointFrameName(std::string wp_name) { return wp_name + "/cp"; }
    std::string getEEFrameName(std::string wp_name) { return wp_name + "/ee"; }
    
  private:
    
    // menu config will hold the menu text as well as bool for whether it should have a checkbox
    typedef std::pair<std::string, bool> MenuConfig;

    // this will handle menus. first item is group name, vector list is nested menu text(s)
    typedef std::map<std::string, std::vector<std::string> > MenuHandleKey;

    // this is a storage DS to store "named" pose info. i.e., frame 'first' in pose.header.frame_id frame 
    typedef std::pair<std::string, geometry_msgs::PoseStamped> FrameInfo;

    typedef std::map<std::string, PlanStatus> EndEffectorPlanStatusMap;
    typedef std::map<std::string, EndEffectorPlanStatusMap> TrajectoryPlanStatus;

    ros::NodeHandle nh_;
    boost::scoped_ptr<boost::thread> updateThread_;
    boost::mutex mutex_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    actionlib::SimpleActionServer<affordance_template_msgs::PlanAction> planning_server_;
    actionlib::SimpleActionServer<affordance_template_msgs::ExecuteAction> execution_server_;

    // bookkeeping and IDs
    std::string robot_name_;
    std::string template_type_;
    std::string name_;
    std::string key_;
    std::string root_object_;
    std::string root_frame_;
    int id_;

    double loop_rate_;
    bool object_controls_display_on_;
    bool running_;
    
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface_;

    std::map<std::string, visualization_msgs::InteractiveMarker> int_markers_;
    std::map<std::string, interactive_markers::MenuHandler> marker_menus_;
    std::map<MenuHandleKey, interactive_markers::MenuHandler::EntryHandle> group_menu_handles_;
    std::map<std::string, AffordanceTemplate::FrameInfo> frame_store_;

    std::vector<MenuConfig> object_menu_options_;
    std::vector<MenuConfig> waypoint_menu_options_;
    std::vector<MenuConfig> toolpoint_menu_options_;
      
    affordance_template_object::AffordanceTemplateParser at_parser_;
    affordance_template_object::AffordanceTemplateStructure initial_structure_;
    affordance_template_object::AffordanceTemplateStructure structure_;

    std::string current_trajectory_;

    std::map<std::string, double> object_scale_factor_;
    std::map<std::string, double> ee_scale_factor_;

    std::map<std::string, WaypointTrajectoryFlags> waypoint_flags_;
    TrajectoryPlanStatus plan_status_;

    std::map<std::string, std::vector<ContinuousPlan> > continuous_plans_; // @seth added new container for continuous planning via actionlib; indexed off trajectory name - may not be final design

    std::string getRootObject() { return root_object_; }
    void setRootObject(std::string root_object) { root_object_ = root_object; }

    bool isValidTrajectory(affordance_template_object::Trajectory traj);
    bool setCurrentTrajectory(affordance_template_object::TrajectoryList traj_list, std::string traj); 
    bool getTrajectory(affordance_template_object::TrajectoryList& traj_list, std::string traj_name, affordance_template_object::Trajectory &traj);

    void clearTrajectoryFlags();
    void setTrajectoryFlags(affordance_template_object::Trajectory traj);

    std::string appendID(std::string s);
    std::string removeID(std::string s);
    std::string createWaypointID(int ee_id, int wp_id);
    bool appendIDToStructure(affordance_template_object::AffordanceTemplateStructure &structure);
    int getEEIDfromWaypointName(const std::string wp_name);
    int getWaypointIDFromName(std::string wp_name);
    int getEEIDFromName(std::string wp_name);
    
    bool createDisplayObjects();
    bool createDisplayObject(affordance_template_object::DisplayObject obj, int idx);

    bool createWaypoints();
    bool createWaypoint(affordance_template_object::EndEffectorWaypoint wp, int ee_id, int wp_id); 
    bool createToolpoint(affordance_template_object::EndEffectorWaypoint wp, int ee_id, int wp_id);

    bool createDisplayObjectMarker(affordance_template_object::DisplayObject obj, visualization_msgs::Marker &marker);
    bool createWaypointMarkers(int ee_id, int wp_id, int ee_pose, visualization_msgs::MarkerArray &markers);

    bool getWaypoint(std::string trajectory, int ee_id, int wp_id, affordance_template_object::EndEffectorWaypoint &wp);

    bool addWaypointBeforeHandler(std::string wp_name);
    bool addWaypointAfterHandler(std::string wp_name);
    bool addWaypointBeforeTrajectoryHandler(std::string obj_name, int ee_id);
    bool addWaypointAfterTrajectoryHandler(std::string obj_name, int ee_id);
    bool deleteWaypointHandler(std::string wp_name);

    bool insertWaypointInTrajectory(affordance_template_object::EndEffectorWaypoint wp, int ee_id, int wp_id, std::string traj_name="");
    bool insertWaypointInList(affordance_template_object::EndEffectorWaypoint wp, int id, affordance_template_object::EndEffectorWaypointList &wp_list);

    bool deleteWaypointFromTrajectory(int ee_id, int wp_id, std::string traj_name="");
    bool deleteWaypointFromList(int ee_id, int wp_id, affordance_template_object::EndEffectorWaypointList &wp_list);

    void removeWaypoint(std::string wp_name);
    void removeAllWaypoints();
    void addInteractiveMarker(visualization_msgs::InteractiveMarker m);
    void removeInteractiveMarker(std::string marker_name);
    void removeAllMarkers();
    bool removeMarkerAndRebuild(std::string marker_name);

    void setupMenuOptions();
    void setupObjectMenu(affordance_template_object::DisplayObject obj);
    void setupWaypointMenu(std::string name);
    void setupToolpointMenu(std::string name);
    
    void setObjectMenuDefaults(std::string obj_name);
    void setWaypointMenuDefaults(std::string wp_name);
    void setToolPointMenuDefaults(std::string tp_name);
    
    void setupSimpleMenuItem(const std::string& name, const std::string& menu_text, bool has_check_box);
    void setupTrajectoryMenu(const std::string& name);
    void setupEndEffectorPoseMenu(const std::string& name);
    void setupAddWaypointMenuItem(std::string, std::string);

    bool hasFrame(std::string frame_name);
    bool hasObjectFrame(std::string obj);
    bool hasWaypointFrame(std::string wp);
    bool hasToolPointFrame(std::string tp);
    bool hasControlPointFrame(std::string wp);
    bool hasEEFrame(std::string wp);
    bool hasControls(std::string name);
    std::string getWaypointFrame(std::string frame);

    bool isWaypoint(const std::string& wp);
    bool isObject(const std::string& obj);
    bool isToolPointFrame(const std::string& tp);
    bool isControlPointFrame(const std::string& cp);
    bool isEEFrame(const std::string& ee);
    void setFrame(std::string frame_name, geometry_msgs::PoseStamped ps);

    bool updatePoseFrames(std::string name, geometry_msgs::PoseStamped ps);
    bool updatePoseInStructure(std::string name, geometry_msgs::Pose p);
    
    geometry_msgs::Pose originToPose(affordance_template_object::Origin origin);

    bool computePathSequence(std::string traj_name, int ee_id, int idx, int steps, bool direct, bool backwards, std::vector<int> &sequence_ids, int &next_path_idx);

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    void planRequest(const affordance_template_msgs::PlanGoalConstPtr&);
    void executeRequest(const affordance_template_msgs::ExecuteGoalConstPtr&);

    bool getContinuousPlan(const std::string&, const int, const std::string&, const PlanningGroup, ContinuousPlan&);
    void setContinuousPlan(const std::string&, const ContinuousPlan&);

    inline std_msgs::ColorRGBA getColorMsg(float r, float g, float b, float a=1.0) {
      std_msgs::ColorRGBA color;
      color.r = r;
      color.g = g;
      color.b = b;
      color.a = a;
      return color;
    }

    bool autoplay_display_;
  };
}

#endif