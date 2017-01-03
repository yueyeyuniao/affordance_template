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

#ifndef MOVEIT_PLANNER_H_
#define MOVEIT_PLANNER_H_

#include <planner_interface/planner_interface.h>
#include <pluginlib/class_list_macros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/BoundingVolume.h>
#include <shape_msgs/SolidPrimitive.h>

#include <robot_interaction_tools_msgs/JointMask.h>

namespace moveit_planner
{
  /** @brief The base class for the moveit planner.
  * Base class to wrap the MoveIt! code API in the planner plugin architecture.
  */
  class MoveItPlanner : public planner_interface::PlannerInterface
  {
    public:

      /** @brief add and setup the group for path planning
      *
      * @param group_name name of new planning group
      * @return whether add was successful
      */
      bool addPlanningGroup_(const std::string &group_name);

      MoveItPlanner();
      ~MoveItPlanner();

      typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupSharedPtr;

    protected:

      // setup function      
      bool setup_();

      planner_interface::ReachableType isReachable(std::string group_name,
                        const geometry_msgs::PoseStamped& pt, 
                        const std::vector<double>& pos_tol, 
                                                   const std::vector<double>& rot_tol) { return planner_interface::EXACTLY_REACHABLE; }


      // functions that base class functions call that need to be implemented
      bool removeGroup_(const std::string &group_name);

      // planning functions
      bool plan_(const std::map<std::string, planner_interface::PlanningGoal> &goals, std::map<std::string, trajectory_msgs::JointTrajectory> &trajectories, bool execute=false, bool show_path=true, JointStateMap start_states=JointStateMap());
      bool planJointPath_(const std::map<std::string, std::vector<sensor_msgs::JointState> > &goals, std::map<std::string, trajectory_msgs::JointTrajectory>& trajectories, bool execute=false, bool show_path=true, JointStateMap start_states=JointStateMap());
      bool executePlans_(const std::vector<std::string> &group_names);
      bool executeContinuousPlans(const std::vector<std::pair<std::string, moveit::planning_interface::MoveGroup::Plan> >&);
      void clearGroupGoals(const std::string &group_name);

      // bookkeeping functions
      bool hasGroup(const std::string &group_name);
      bool getGroupPlanningFrame(const std::string &group_name, std::string &planning_frame);
      bool getEndEffectorLink(const std::string &group_name, std::string &ee_link);

      bool getPlan(const std::string&, moveit::planning_interface::MoveGroup::Plan&);
      bool getCurrentState(const std::string&, sensor_msgs::JointState&);

      bool applyMaskConstraints(const std::string &group_name, moveit_msgs::Constraints &constraints);

      bool setObstacles(planner_interface::SetObstacles::Request  &req   ,
                          planner_interface::SetObstacles::Response &res);

      moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;  
      std::map<std::string, MoveGroupSharedPtr> groups_;

      ros::Publisher display_publisher;
      std::map<std::string, moveit_msgs::DisplayTrajectory> stored_display_trajectory_;
      std::map<std::string, moveit::planning_interface::MoveGroup::Plan> stored_plan_;

      bool use_aggregate_groups_;

  };
};

#endif
