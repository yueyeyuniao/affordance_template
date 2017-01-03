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

#ifndef PLUGINLIB_PLANNER_INTERFACE_H_
#define PLUGINLIB_PLANNER_INTERFACE_H_

#include <ros/ros.h>

#include <random>

#include <tf/transform_listener.h>
#include <kdl_conversions/kdl_msg.h>

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

#include <robot_interaction_tools_msgs/JointMask.h>
#include <planner_interface/tolerance_util.h>
#include <planner_interface/SetObstacles.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>

#include <rit_utils/rdf_model.h>
#include <rit_utils/generic_utils.h>
#include <rit_utils/predictive_robot_display.h>

static const std::vector<double> PLAN_COLOR = {0.5,0.1,0.75,1};
static const char PATH_INCREMENT = 1;

typedef moveit::planning_interface::MoveGroup::Plan Plan;
typedef std::map<std::string, sensor_msgs::JointState> JointStateMap;

namespace planner_interface
{
  enum PlannerType {
    CARTESIAN=0,
    JOINT
  };

  enum ReachableType {
    UNREACHABLE=0,
    REACHABLE_WITH_TOLERANCES,
    EXACTLY_REACHABLE
  };

  struct PlanningGoal {
    geometry_msgs::PoseStamped goal;
    geometry_msgs::Pose offset;
    geometry_msgs::Pose task_compatibility;
    PlannerType type;
    std::string conditioning_metric;
    double tolerance_bounds[6][2];
  };

  /** 
   * @brief The base class for planning groups.
   * @description Base class to run as a plugin for other classes to be definded later.
   */
  class PlannerInterface {

    public:
      PlannerInterface() {}
      virtual ~PlannerInterface() {}

      /**
       * @brief Create a predictive visualiztion.
       * @details Predictive robot visualization displays the result from planning.
       * @param prefix tf prefix for robot 
       */
      void createDisplay(const std::string& prefix);

      /** 
       * @brief Remove group from active planning groups.
       * @param group_name name of planning group to Remove
       * @return true if removal was successful
       */
      bool removeGroup(const std::string& group_name);

      /** 
       * @brief Get list of active planning group names.
       * @param active_groups returned list of active groups
       */
      void getActiveGroupNames(std::vector<std::string>& active_groups);

      /**
       * @brief Get type of group as a string.
       * @descrition Return empty string if group name not found in active planning groups.
       * @param group_name group name to lookup
       * @return GroupType enum
       */
      rit_utils::GroupType getGroupType(const std::string& group_name);

      /** 
       * @brief Get list of joint names in group.
       * @description Return empty vector if group name not found in active planning groups.
       * @param group_name lookup joints using this group name
       * @param joints return list of joint names for group
       */
      void getAllGroupJoints(const std::string& group_name, std::vector<std::string>& joints);

      /** 
       * @brief Get end effector names from SRDF model.
       * @return list of end effector names
       */
      std::vector<std::string> getEndEffectorNames();

      /** 
       * @brief Get control frame name for given group.
       * @param group_name find the control frame using this group name
       * @return control frame name
       */
      std::string getControlFrame(const std::string& group_name);

      /** 
       * @brief Get group state for group.
       * @description Lookup group state using group name through the SRDF model. Returns as a JointState.
       * @todo need to find descriptions for params
       * @param group_name TODO
       * @param group_state_name TODO
       * @param group_state JointState of group to return
       */
      void getStoredGroupState(const std::string& group_name, const std::string& group_state_name, sensor_msgs::JointState& group_state);

      /** 
       * @brief Get group state list based on group name.
       * @description Lookup through the SRDF model.
       * @param group_name name of group to get state list
       * @param group_state_list list of group states to return
       */
      void getStoredStateList(const std::string& group_name, std::vector<std::string>& group_state_list);

      /** 
       * @brief Get group state map for group name.
       * @description Lookup through the SRDF model.
       * @param group_name name of group to get state list
       * @param group_states map of JointStates keyed on names
       * @return success if group has stored states
       */
      bool getStoredStateMap(const std::string& group_name, std::map<std::string, sensor_msgs::JointState>& group_states);

      /**
       * @brief Get control mesh for group name.
       * @param group_name get control meshes based on name
       * @param control_mesh name of control mesh to return
       */
      void getControlMesh(const std::string& group_name, std::string& control_mesh);

      /** 
       * @brief Get MarkerArray of all shapes/meshes for a group's link.
       * @param group_name get control meshes based on name
       * @param markers the MarkerArray to return
       */
      bool getGroupShapeMarkers(const std::string& group_name, visualization_msgs::MarkerArray& markers);

      /** 
       * @brief Generate random group ID offset.
       * @return randomly generated group ID offset
       */
      int getRandomOffsetId();

      /** 
       * @brief Get group link names based on group name.
       * @param group get link names in this group
       * @param links return list of links in group
       */
      void getGroupLinks(const std::string& group, std::vector<std::string>& links);

      /** 
       * @brief Get joint names found in group.
       * @param group group name to lookup
       * @param names return list of joints in group, empty if group name not found
       */
      void getGroupJoints(const std::string group, std::vector<std::string>& names);

      /** 
       * @brief Get base frame name for a group.
       * @param group get base frame name for this group name
       * @param base_frame frame name to return
       */
      void getBaseFrame(const std::string& group, std::string& base_frame);

      /** 
       * @brief Get base frame name for group from SRDF model.
       * @param group_name get base frame name for this group name
       * @param group_base_frame group base frame name to return
       */
      void getGroupBaseFrame(const std::string& group_name, std::string& group_base_frame);

      /** 
       * @brief Get JointMask for group from SRDF model.
       * @param group get joint mask for this group name
       * @param mask JointMask to return
       */
      bool getJointMask(const std::string& group, robot_interaction_tools_msgs::JointMask& mask);

      /** 
       * @brief Set JointMask for group in SRDF model.
       * @param group set JointMask for this group name
       * @param mask JointMask to set
       */
      bool setJointMask(const std::string& group, const robot_interaction_tools_msgs::JointMask& mask);

      /** 
       * @brief Print basic info about robot and planning groups.
       */
      void printBasicInfo(){
          ROS_WARN("PlannerInterface::printBasicInfo() -- TODO");
      }

      /** 
       * @brief Add a new planning group.
       * @param group_name name of new planning group
       * @param group_type type of new planning group
       * @param jont_tolerance joint tolerance to use when planning
       * @param position_tolerances
       * @param orientation_tolerances
       * @return success indicator
       */
      bool addPlanningGroup(const std::string& group_name,
                            const std::string& group_type,
                            const double joint_tolerance,
                            double position_tolerances,
                            double orientation_tolerances);

      /** 
       * @brief sets certain parameters on server to be read dynamically
       */
      void setupParameters();

      /**
       * @brief Get conditioning metrics.
       * @param m list of conditioning metrics to return
       */
      void getConditioningMetrics(std::vector<std::string>&);

      /**
       * @brief Add conditioning metric name.
       * @param m name of conditioning metric to add
       */
      void addConditioningMetric(std::string);

      /**
       * @brief Get list of group names.
       * @return list of groups
       */
      std::vector<std::string> getGroups();

      /**
       * @brief Plan joint path for a group. 
       * @param goals map of joint goals for planning groups
       * @param start_states state to begin planning from 
       * @param execute boolean value to execute upon successful plan
       * @param show_path boolean value to visualize the planned path
       * @return plan success
       */
      bool planJointPath(const std::map<std::string, std::vector<sensor_msgs::JointState> >& goals,
                         bool execute=false, 
                         bool show_path=true,
                         JointStateMap start_states=JointStateMap());


      /**
       * @brief Cartesian path planning for multiple groups.
       * @param goals pose goals to plan for
       * @param start_states state to start planning from 
       * @param execute boolean value to execute upon successful plan -- optional
       * @param show_path boolean value to visualize the planned path -- optional
       * @return plan success
       */
      bool plan(const std::map<std::string, PlanningGoal>& goals, 
                bool execute=false, 
                bool show_path=true,
                JointStateMap start_states=JointStateMap());


      /**
       * @brief Cartesian path planning for a single group.
       * @param group_name name of group to plan for
       * @param goal pose goal to plan for
       * @param start_states state to start planning from 
       * @param execute boolean value to execute upon successful plan -- optional
       * @param show_path boolean value to visualize the planned path -- optional
       * @return plan success
       */
      bool plan(const std::string& group_name, 
                const PlanningGoal& goal, 
                bool execute=false, 
                bool show_path=true,
                JointStateMap start_states=JointStateMap());

      /**
       * @brief Execute available plans for a list of groups.
       * @param group_names list of groups to execute
       * @return execution success
       */
      bool executePlans(const std::vector<std::string>& group_names);

      /**
       * @brief Set offset pose for tools.
       * @param group_name group to set tool offset
       * @param offset pose of tool
       */
      void setToolOffset(const std::string& group_name, geometry_msgs::Pose offset);

      /**
       * @brief Clear offset pose for tool.
       * @param group_name group to clear tool offset
       * @return success indicator
       */
      bool clearToolOffset(const std::string& group_name);

      /**
       * @brief Get a group's tool offset pose.
       * @param group_name group to get pose offset
       * @param p pose reference to return offset pose
       * @return success indicator
       */
      bool getToolOffset(const std::string& group_name, geometry_msgs::Pose& p);

      /**
       * @brief Get list of planning groups found in SRDF.
       * @param group_names names of planning groups
       */
      void getGroupNames(std::vector<std::string>& group_names);

      /**
       * @brief Transform a goal in a group's frame.
       * @param group_name desired group to transform to
       * @param goal_in pose goal to transform
       * @param goal_out transformed pose goal
       */
      void transformGoal(std::string group_name, geometry_msgs::PoseStamped goal_in, geometry_msgs::PoseStamped& goal_out);

      /**
       * @brief Transform multiple goals into new frames.
       * @param goals map of poses to transform into group's frame
       */
      void transformGoalMap(std::map<std::string, std::vector<geometry_msgs::PoseStamped> >& goals);

      /**
       * @brief Set position tolerances for a group.
       * @param group_name group to set tolerances for
       * @param tols tolerance value
       * @return success indicator
       */
      bool setPositionTolerances(const std::string& group_name, double tols);

      /**
       * @brief Set orientation tolerances for a group.
       * @param group_name group to set tolerances for
       * @param tols tolerance value
       * @return success indicator
       */
      bool setOrientationTolerances(const std::string& group_name, double tols);

      //****************************************************************
      //********************* VIRTUAL METHODS **************************
      //****************************************************************

      /** 
       * @brief Initialize the planner.
       * @details Initialize the planning class with robot details. Setup callbacks. Ready publishing topics.
       * @param nh parent node's nodehandle
       * @param robot_name the name of the robot
       * @return success indicator
       */
      virtual bool initialize(const ros::NodeHandle& nh, const std::string& robot_name);

      /**
       * @brief Get a generated plan for a group.
       * @param group group name
       * @param plan plan to return
       * @return success if plan found for group
       */
      virtual bool getPlan(const std::string& group, moveit::planning_interface::MoveGroup::Plan& plan){}

      /**
       * @brief Get current joint states for a group.
       * @param group group name to lookup
       * @param state current state to return
       * @return success indicator
       */
      virtual bool getCurrentState(const std::string& group, sensor_msgs::JointState& state){}

      /**
       * @brief Execute successive plans for a group.
       * @param group_plans list of group name and plan pairs
       * @return successful execution
       */
      virtual bool executeContinuousPlans(const std::vector<std::pair<std::string, moveit::planning_interface::MoveGroup::Plan> >&){}
      
      
      //****************************************************************
      //****************** PURE VIRTUAL METHODS ************************
      //****************************************************************
      
      /**
       * @brief A pure virtual method for all subs to implement.
       * @details Subclass setup method.
       * @return success indicator
       */
      virtual bool setup_() = 0;

      /**
       * @brief A pure virtual method for all subs to implement.
       * @details Test reachability of a pose for a group contrained by tolerances.
       * @param group_name name of group to test
       * @param pt pose to test reachability
       * @param pos_tol translation tolerances
       * @param rot_tol rotation tolerances
       * @return ReachableType enum
       */
      virtual ReachableType isReachable(std::string group_name,
                                        const geometry_msgs::PoseStamped& pt, 
                                        const std::vector<double>& pos_tol, 
                                        const std::vector<double>& rot_tol) = 0;

      /** 
       * @brief remove group from planner
       * @param group_name the group name being added
       * @return success indicator
       */
      virtual bool removeGroup_(const std::string& group_name) = 0;

      /** 
       * @brief Determines if group name is a valid planning group.
       * @param group_name name of group in question
       * @return whether group name is a valid planning group
       */
      virtual bool hasGroup(const std::string& group_name) = 0;

      /** 
       * @brief Get the planning frame for a specified group.
       * @param group_name the group
       * @param planning_frame the returned planning frame of the group
       * @return whether the group exists and has a planning frame
       */
      virtual bool getGroupPlanningFrame(const std::string& group_name, std::string& planning_frame) = 0;

      /** 
       * @brief Get the end-effector link name for a specified group.
       * @param group_name the group
       * @param ee_link the returned planning frame of the group
       * @return whether the group exists and has a planning frame
       */
      virtual bool getEndEffectorLink(const std::string& group_name, std::string& ee_link) = 0;

      /** 
       * @brief setup the group for path planning
       * @param group_name the group name being added
       * @return success indicator
       */
      virtual bool addPlanningGroup_(const std::string& group_name) = 0;
      
      /**
       * @brief Path planning for a joint goal. 
       * @details Found in subclass.
       * @param goals joint goals to plan for
       * @param trajectories resulting joint trajectories
       * @param start_states state to start planning from 
       * @param execute boolean value to execute upon successful plan
       * @param show_path boolean value to visualize the planned path
       * @return planning success
       */
      virtual bool planJointPath_(const std::map<std::string, std::vector<sensor_msgs::JointState> >& goals, 
                                  std::map<std::string, trajectory_msgs::JointTrajectory>& trajectories, 
                                  bool execute=false, 
                                  bool show_path=true,
                                  JointStateMap start_states=JointStateMap()) = 0;

      /**
       * @brief Cartesian planning.
       * @details Found in subclass.
       * @param trajectories resulting trajectories
       * @param start_states state to start planning from 
       * @param execute boolean value to execute upon successful plan -- optional
       * @param show_path boolean value to visualize the planned path -- optional
       * @return planning success
       */
      virtual bool plan_(const std::map<std::string, planner_interface::PlanningGoal>& goals, 
                         std::map<std::string, trajectory_msgs::JointTrajectory>& trajectories, 
                         bool execute=false, 
                         bool show_path=true,
                         JointStateMap start_states=JointStateMap()) = 0;

      /**
       * @brief Execute any available plans for a group.
       * @param group_names list of group names to execute
       * @return execution success
       */
      virtual bool executePlans_(const std::vector<std::string>& group_names) = 0;

      //****************************************************************
      //********************** INLINE METHODS **************************
      //****************************************************************

      /**
       * @brief Service call to set obstacles in planning scene.
       * @param req request message
       * @param res response message
       * @return success indicator
       */
      inline virtual bool setObstacles(SetObstacles::Request& req, SetObstacles::Response& res) {
        ROS_WARN("PlannerInterface plugin does not supprt obstacles at this point");
        return false;
      }

      /**
       * @brief Get partial plan flag.
       * @details Return boolean based on whether generated plan is partial or full plan
       * @return boolean value
       */
      inline bool isPartialPlan() { return is_partial_plan_; }

      /**
       * @brief Get pointer to RDF Model.
       * @return RDFModel pointer
       */
      inline rit_utils::RDFModel *getRDFModel() { return rdf_model_; } //boost::shared_ptr<utils::REDFModel>

      /**
       * @brief Display available planned paths. Passthrough method.
       */
      inline void playAnimation() { display_robot_->playPlan(); }

      /**
       * @brief Set looping flag for planning display. Passthrough method.
       * @param group group name to set flag for
       * @param loop bool value to set
       */
      inline void loopAnimation(const std::string& group, const bool loop) { display_robot_->setLoop(group, loop); }

      /**
       * @brief Clear path visualization data. Passthrough method.
       * @param all optional boolean value to clear all data stored data or just current data being visualized
       */
      inline void resetAnimation(bool all=false) { display_robot_->resetDisplay(all); }

      //****************************************************************
      //********************** CLASS MEMBERS ***************************
      //****************************************************************

      ros::NodeHandle nh_;
      ros::Publisher obstacle_pub_;
      ros::ServiceServer set_obstacle_srv_;
      
      visualization_msgs::MarkerArray obstacles_;
      tf::TransformListener tf_listener_;

      bool is_partial_plan_;
      
      double trajectory_scaler_;
      double trajectory_density_;
      
      std::string robot_name_;
      std::string robot_description_;
      std::string semantic_description_;

      std::vector<std::string> active_groups_;
      std::vector<std::string> conditioning_metrics_;

      std::map<std::string, std::string> group_types_;
      std::map<std::string, std::string> base_frames_;
      std::map<std::string, std::string> control_frames_;
      std::map<std::string, std::string> control_meshes_;
      std::map<std::string, std::string> parent_frames_;
      std::map<std::string, std::string> display_modes_;

      std::map<std::string, bool> plan_generated_;
      std::map<std::string, bool> execution_status_;
      std::map<std::string, bool> auto_execute_;

      std::map<std::string, int> group_id_offset_;

      std::map<std::string, double> joint_tolerance_;
      std::map<std::string, double> position_tolerances_;
      std::map<std::string, double> orientation_tolerances_;

      std::map<std::string, geometry_msgs::Pose> tool_offset_;
      std::map<std::string, visualization_msgs::MarkerArray> trajectory_display_markers_;
      std::map<std::string, trajectory_msgs::JointTrajectory> stored_plans_;
      std::map<std::string, visualization_msgs::MarkerArray> marker_store_;
      std::map<std::string, robot_interaction_tools_msgs::JointMask> joint_mask_;

      rit_utils::RDFModel *rdf_model_; // FIXME -- // boost::shared_ptr<utils::REDFModel> rdf_model_;
      boost::shared_ptr<robot_display::PredictiveDisplay> display_robot_;
      tolerance_util::ToleranceUtilSharedPtr toleranceUtil;
  };
};

#endif
