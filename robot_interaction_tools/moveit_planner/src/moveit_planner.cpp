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

#include <moveit_planner/moveit_planner.h>

// using namespace moveit::core;

namespace moveit_planner
{

MoveItPlanner::MoveItPlanner() :
  use_aggregate_groups_(false)
{
  ROS_INFO("MoveItPlanner() created");
}

MoveItPlanner::~MoveItPlanner() { }
      

bool MoveItPlanner::setup_() {

  display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  return true;
}

bool MoveItPlanner::addPlanningGroup_(const std::string &group_name)
{

    MoveGroupSharedPtr g( new moveit::planning_interface::MoveGroup(group_name) );
    groups_[group_name] = g;

    groups_[group_name]->setGoalJointTolerance(joint_tolerance_[group_name]);
    groups_[group_name]->setGoalPositionTolerance(position_tolerances_[group_name]);
    groups_[group_name]->setGoalOrientationTolerance(orientation_tolerances_[group_name]);

    ROS_INFO("MoveItPlanner::setupGroup(%s) -- reference frame: %s", group_name.c_str(), g->getPlanningFrame().c_str());
    ROS_INFO("MoveItPlanner::setupGroup(%s) -- EE Link: %s", group_name.c_str(), g->getEndEffectorLink().c_str());

    // DEBUG
    std::vector<double> lower, upper;
    rdf_model_->getJointLimits(group_name, lower, upper );
    // END DEBUG

    return true;
}

bool MoveItPlanner::hasGroup(const std::string &group_name) {
  return (groups_.find(group_name) != std::end(groups_)); 
}

bool MoveItPlanner::getGroupPlanningFrame(const std::string &group_name, std::string &planning_frame) {
  ROS_INFO("MoveItPlanner::getGroupPlanningFrame() -- group: %s", group_name.c_str());
  if(hasGroup(group_name)) {
    planning_frame = groups_[group_name]->getPlanningFrame();
    ROS_INFO("MoveItPlanner::getGroupPlanningFrame(%s) -- %s", group_name.c_str(),planning_frame.c_str());
    return true;
  } else {
    return false;
  }
}


bool MoveItPlanner::getEndEffectorLink(const std::string &group_name, std::string &ee_link) {
  if(hasGroup(group_name)) {
    ee_link = groups_[group_name]->getEndEffectorLink();
    return true;
  } else {
    return false;
  }
}


bool MoveItPlanner::removeGroup_(const std::string &group_name) {
  ROS_WARN("MoveItPlanner::removeGroup() -- %s", group_name.c_str());
  if(hasGroup(group_name)) {
    groups_[group_name].reset();
    groups_.erase(group_name);
  }
  return true;
}

void MoveItPlanner::clearGroupGoals(const std::string &group_name) {
  if(hasGroup(group_name)) {
    ROS_DEBUG("MoveItPlanner::clearGroupGoals() -- clearing targets or group %s", group_name.c_str()); 
    groups_[group_name]->clearPoseTargets();  
    try {
      stored_display_trajectory_[group_name].trajectory.clear();
    } catch(...) {
      ROS_WARN("MoveItPlanner::clearGroupGoals() -- no display trajectory to clear");
    } 
    stored_display_trajectory_.erase(group_name);
    stored_plan_.erase(group_name);
  }
}

bool MoveItPlanner::setObstacles(planner_interface::SetObstacles::Request  &req,
                          planner_interface::SetObstacles::Response &res) {

    std::vector<moveit_msgs::CollisionObject> collision_objects;

    for(auto &obj : req.marker_obstacles.markers) {

      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = obj.header.frame_id;

      /* The id of the object is used to identify it. */
      collision_object.id = obj.ns + "/" + obj.text;


      if(obj.type==visualization_msgs::Marker::CUBE) {
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = obj.scale.x;
        primitive.dimensions[1] = obj.scale.y;
        primitive.dimensions[2] = obj.scale.z;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(obj.pose);
      } else if(obj.type==visualization_msgs::Marker::SPHERE) {
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[0] = obj.scale.x;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(obj.pose);
      } else if(obj.type==visualization_msgs::Marker::CYLINDER) {
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = obj.scale.x;
        primitive.dimensions[1] = obj.scale.y;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(obj.pose);
      } else if(obj.type==visualization_msgs::Marker::MESH_RESOURCE) {
        ROS_WARN("TracIKPlanner::setObstacles() -- mesh obstacles not supported");
        continue;
      }
   
      if(req.action == req.ADD) {
        collision_object.operation = collision_object.ADD;
        ROS_INFO("Add obstacle %s into the world", collision_object.id.c_str());
        obj.action = visualization_msgs::Marker::ADD;
      } else if(req.action == req.DELETE) {
        collision_object.operation = collision_object.REMOVE;
        ROS_INFO("Removing obstacle %s from the world", collision_object.id.c_str());
        obj.action = visualization_msgs::Marker::DELETE;
      } else if(req.action == req.MOVE) {
        collision_object.operation = collision_object.MOVE;
        ROS_INFO("Moving obstacle %s in world", collision_object.id.c_str());
        obj.action = visualization_msgs::Marker::MODIFY;
      } 
      obstacles_.markers.push_back(obj);

      collision_objects.push_back(collision_object);

    }
 
    for(auto &obj : req.moveit_obstacles) {
      moveit_msgs::CollisionObject co = obj;
      if(req.action == req.ADD) {
        co.operation = co.ADD;
        ROS_INFO("Add MoveIt! obstacle %s into the world", co.id.c_str());
      } else if(req.action == req.DELETE) {
        co.operation = co.REMOVE;
        ROS_INFO("Removing MoveIt! obstacle %s from the world", co.id.c_str());
      } else if(req.action == req.MOVE) {
        co.operation = co.MOVE;
        ROS_INFO("Moving MoveIt! obstacle %s in world", co.id.c_str());
      } 
      collision_objects.push_back(co);
    }


    planning_scene_interface_.addCollisionObjects(collision_objects);

    obstacle_pub_.publish(obstacles_);

    res.result = true;
    return true;
}
    

bool MoveItPlanner::planJointPath_(const std::map<std::string, std::vector<sensor_msgs::JointState> > &goals, std::map<std::string, trajectory_msgs::JointTrajectory>& trajectories, bool execute, bool show_path, JointStateMap start_states) {

  // storage
  std::vector<std::string> group_names;

  // go through all the group goal arrays
  for(auto &req : goals) {

    std::string group_name = req.first;
    std::vector<sensor_msgs::JointState> waypoints = req.second;

    // safety check
    if(!hasGroup(group_name)) {
        ROS_ERROR("MoveItPlanner::planJointPath_() -- unknown group: %s", group_name.c_str());
        return false;
    }

    ROS_DEBUG("MoveItPlanner::planJointPath_() -- plan group %s with %d joint goals", group_name.c_str(), (int)waypoints.size());
    if(waypoints.size() > 1 ) {
      ROS_WARN("MoveItPlanner::planJointPath_() -- doesn't support multiple joint goal waypoints yet!");
      return false;
    }
    
    // store the group names in easily accesible vector
    group_names.push_back(group_name);

    robot_state::RobotStatePtr current_state = groups_[group_name]->getCurrentState();
    if (start_states.find(group_name) != start_states.end()) {
      if (start_states[group_name].name.size() == start_states[group_name].position.size() && start_states[group_name].name.size() > 0) {
        ROS_INFO("[MoveItPlanner::planJointPath_] valid start state provided for %s", group_name.c_str());
        current_state->setVariableValues(start_states[group_name]);
        groups_[group_name]->setStartState(*current_state);
      } else {
        groups_[group_name]->setStartStateToCurrentState();
        if (start_states[group_name].name.size() == 0) 
          ROS_INFO("[MoveItPlanner::planJointPath_] start state provided for %s is empty, will use current robot state", group_name.c_str());
        else
          ROS_WARN("[MoveItPlanner::planJointPath_] invalid start state provided for %s, will use current robot state", group_name.c_str());
      }
    } else {
      ROS_INFO("[MoveItPlanner::planJointPath_] no start state provided for %s, will use current robot state", group_name.c_str());
      groups_[group_name]->setStartStateToCurrentState();
    }

    clearGroupGoals(group_name);
    groups_[group_name]->setJointValueTarget(waypoints[0]);
    moveit::planning_interface::MoveGroup::Plan joint_plan;
    if(groups_[group_name]->plan(joint_plan)) {
      for(auto &pt : joint_plan.trajectory_.joint_trajectory.points) {
        pt.velocities.clear();
        pt.accelerations.clear();
      }
      stored_plan_[group_name] = joint_plan;
      trajectories[group_name] = joint_plan.trajectory_.joint_trajectory;
    } else {
      ROS_WARN("MoveItPlanner::planJointPath_() -- could not find joint plan for group %s", group_name.c_str());     
    }
  }

  // if executing all at once
  if(execute) {
    return executePlans(group_names);
  }

  return true;

}
      
bool MoveItPlanner::applyMaskConstraints(const std::string &group_name, moveit_msgs::Constraints &constraints) {

  if(!hasGroup(group_name)) {
    ROS_ERROR("MoveItPlanner::applyMaskConstraints() -- unknown group: %s", group_name.c_str());
    return false;
  }

  robot_interaction_tools_msgs::JointMask mask;
  robot_state::RobotStatePtr robotState = groups_[group_name]->getCurrentState(); 
  if(getJointMask(group_name, mask)) {
    int idx = 0;
    for(auto &joint_name: mask.joint_names) {
      if(!mask.mask[idx]) {
        ROS_WARN("MoveItPlanner::applyMaskConstraints(%s) masking joint: %s", group_name.c_str(), joint_name.c_str());
        const double *jnt_pos = robotState->getJointPositions(joint_name);
        moveit_msgs::JointConstraint jc;
        jc.joint_name = joint_name;
        jc.position = jnt_pos[0];
        jc.tolerance_above = 0.001;
        jc.tolerance_below = 0.001;
        jc.weight = 1.0;
        constraints.joint_constraints.push_back(jc);
        groups_[group_name]->setPathConstraints(constraints);
      }
      idx++;
    }
  }

  return true;
}
    
bool MoveItPlanner::plan_(const std::map<std::string, planner_interface::PlanningGoal> &goals, std::map<std::string, trajectory_msgs::JointTrajectory> &trajectories, bool execute, bool show_path, JointStateMap start_states) {
  
  // storage
  std::vector<std::string> group_names;

  // go through all the group goal arrays
  for(auto &req : goals) {

    std::string group_name = req.first;
    ROS_INFO("[MoveItPlanner::plan] planning for group %s", group_name.c_str());
    
    // safety check
    if(!hasGroup(group_name)) {
        ROS_ERROR("MoveItPlanner::plan() -- unknown group: %s", group_name.c_str());
        return false;
    }

    // store the group names in easily accesible vector
    group_names.push_back(group_name);

    groups_[group_name]->clearPathConstraints();
    moveit_msgs::Constraints constraints;
    applyMaskConstraints(group_name, constraints);

    if(req.second.type == planner_interface::CARTESIAN) { 

      std::vector<geometry_msgs::Pose> pose_goals;
      pose_goals.push_back(req.second.goal.pose);      

      moveit::planning_interface::MoveGroup::Plan cart_plan;      
      moveit_msgs::RobotTrajectory traj;

      // use start state for planning, if available
      moveit_msgs::RobotState start_state;
      robot_state::RobotStatePtr current_state = groups_[group_name]->getCurrentState();
      moveit::core::robotStateToRobotStateMsg(*current_state, start_state);
      if (start_states.find(group_name) != start_states.end()) {
        if (start_states[group_name].name.size() == start_states[group_name].position.size() && start_states[group_name].name.size() > 0) {
          ROS_INFO("[MoveItPlanner::plan] valid start state provided for %s", group_name.c_str());
          current_state->setVariableValues(start_states[group_name]);
          groups_[group_name]->setStartState(*current_state);
          moveit::core::robotStateToRobotStateMsg(*current_state, start_state);
        } else {
          groups_[group_name]->setStartStateToCurrentState();
          if (start_states[group_name].name.size() == 0) 
            ROS_INFO("[MoveItPlanner::plan] start state provided for %s is empty, will use current robot state", group_name.c_str());
          else
            ROS_WARN("[MoveItPlanner::plan] invalid start state provided for %s, will use current robot state", group_name.c_str());
        }
      } else {
        ROS_INFO("[MoveItPlanner::plan] no start state provided for %s, will use current robot state", group_name.c_str());
        groups_[group_name]->setStartStateToCurrentState();
      }

      double fraction = groups_[group_name]->computeCartesianPath(pose_goals, 0.01, 10, traj);

      if(fraction > 0) {
        is_partial_plan_ = true;
        ROS_WARN("MoveItPlanner::plan() -- planned cartesian path for group %s, result fraction = %f", group_name.c_str(), fraction); 
      } else {
        is_partial_plan_ = false;
        ROS_INFO("MoveItPlanner::plan() -- planned cartesian path for group %s, result fraction = %f", group_name.c_str(), fraction);  
      }

      if (fraction < 1)
        return false;

      // fix cart traj to have velocities and accelerations cause this call doesnt populate those fields
      if(traj.joint_trajectory.points.size() > 0) {
        int n = traj.joint_trajectory.points[0].positions.size();
        if(traj.joint_trajectory.points[0].velocities.size() == 0) {
          for(size_t i=0; i<traj.joint_trajectory.points.size(); i++) {
            traj.joint_trajectory.points[i].velocities.resize(n,0);
            traj.joint_trajectory.points[i].accelerations.resize(n,0);
          }
        }
      }

      cart_plan.trajectory_ = traj;
      cart_plan.start_state_ = start_state;
      stored_plan_[group_name] = cart_plan;
      trajectories[group_name] = cart_plan.trajectory_.joint_trajectory;
      ROS_DEBUG("[MoveItPlanner::plan] cartesian goal with traj size %d", (int)trajectories[group_name].points.size());
    } else {
      clearGroupGoals(group_name);
      geometry_msgs::PoseStamped pose_goal = req.second.goal;
      pose_goal.header.stamp = ros::Time::now();
      groups_[group_name]->setPoseTarget(pose_goal);
      moveit::planning_interface::MoveGroup::Plan joint_plan;
      ROS_INFO("MoveItPlanner::plan() -- calling MoveIt! joint planner for %s with EE link: %s", group_name.c_str(), groups_[group_name]->getEndEffectorLink().c_str());
      if(groups_[group_name]->plan(joint_plan)) {
        ROS_INFO("MoveItPlanner::plan() -- done");
        for(auto &pt : joint_plan.trajectory_.joint_trajectory.points) {
          pt.velocities.clear();
          pt.accelerations.clear();
        }
        stored_plan_[group_name] = joint_plan;
        trajectories[group_name] = joint_plan.trajectory_.joint_trajectory;
        ROS_DEBUG("[MoveItPlanner::plan] non cartesian goal with traj size %d", (int)trajectories[group_name].points.size());
      } else {
        ROS_WARN("MoveItPlanner::plan() -- could not find joint plan for group %s", group_name.c_str());  
      }
    }
  }

  // if executing all at once
  if(execute) {
    return executePlans(group_names);
  }

  return true;
}


bool MoveItPlanner::executeContinuousPlans(const std::vector<std::pair<std::string, moveit::planning_interface::MoveGroup::Plan> >& group_plans)
{
  ROS_DEBUG("[MoveItPlanner::executeContinuousPlans] executing %d plans continuously", (int)group_plans.size());

  // FIXME this doesn't have any error checking because many times it returns false even when the plan executes and appears to get to the goal
  for (auto p : group_plans)
  {
    ROS_DEBUG("[MoveItPlanner::executeContinuousPlans] executing group %s plan", p.first.c_str());
    groups_[p.first]->execute(p.second);
  }

  return true;
}

bool MoveItPlanner::executePlans_(const std::vector<std::string> &group_names) {
  for(auto &group_name : group_names) {
    ROS_INFO("MoveItPlanner::executePlans_() -- %s", group_name.c_str());  
    if(stored_plan_.find(group_name) != std::end(stored_plan_)) {
      //scale trajectories
      moveit::planning_interface::MoveGroup::Plan cart_plan= stored_plan_[group_name];
      
      for (uint i=0; i < cart_plan.trajectory_.joint_trajectory.points.size(); i++) {
        cart_plan.trajectory_.joint_trajectory.points[i].time_from_start = ros::Duration(cart_plan.trajectory_.joint_trajectory.points[i].time_from_start.toSec() / trajectory_scaler_);
      }
      
      groups_[group_name]->asyncExecute(cart_plan);
      clearGroupGoals(group_name);
    } else {
      ROS_WARN("MoveItPlanner::executePlans_() -- no stored plan for group %s", group_name.c_str()); 
      return false;
    }
  }
  return true;
}

bool MoveItPlanner::getPlan(const std::string& group, moveit::planning_interface::MoveGroup::Plan& plan)
{ 
  if (stored_plan_.find(group) != stored_plan_.end())
  {
    plan = stored_plan_[group];
    return true;
  }

  return false;
}

bool MoveItPlanner::getCurrentState(const std::string& group, sensor_msgs::JointState& state)
{
  if (groups_.find(group) == groups_.end())
  {
    ROS_ERROR("[MoveItPlanner::getCurrentState] group %s not a current planning group!!", group.c_str());
    return false;
  }

  state.name = groups_[group]->getActiveJoints();
  state.position = groups_[group]->getCurrentJointValues();

  if (state.name.size() != state.position.size())
  {
    ROS_ERROR("[MoveItPlanner::getCurrentState] current state size doesn't match!!");
    return false;
  }

  return true;
}

}; // namespace

PLUGINLIB_EXPORT_CLASS(moveit_planner::MoveItPlanner, planner_interface::PlannerInterface)
