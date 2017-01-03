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

#include <planner_interface/planner_interface.h>

using namespace planner_interface;

bool PlannerInterface::initialize(const ros::NodeHandle &nh, const std::string& robot_name)
{
  robot_name_ = robot_name;
  nh_ = nh;
  semantic_description_ = "/robot_description_semantic";
  robot_description_ = "/robot_description";
  trajectory_scaler_ = 1.0; 
  trajectory_density_ = 0.001;  // shows joint config at 1 ms intervals
  is_partial_plan_=false;

  std::string node_name = ros::this_node::getName();

  set_obstacle_srv_ = nh_.advertiseService(node_name + "/set_obstacles", &PlannerInterface::setObstacles, this);
  obstacle_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(node_name + "/obstacles", 1000);

  ROS_DEBUG("PlannerInterface::initialize() -- nh namespace: %s", node_name.c_str());
  ROS_DEBUG("PlannerInterface::initialize() -- loading tolerances");
  toleranceUtil = tolerance_util::ToleranceUtilSharedPtr(new tolerance_util::ToleranceUtil(nh));

  ros::Duration(1.0).sleep();

  // load up URDF and SRDF models to utility
  rdf_model_ = new rit_utils::RDFModel(robot_name);
  if (!rdf_model_->init(semantic_description_, robot_description_))
  {
    ROS_FATAL("PathPlanner::init() -- failed creating RDF models");
    return false;
  }

  setupParameters();

  return setup_();
}

void PlannerInterface::createDisplay(const std::string& prefix)
{
  display_robot_.reset(new robot_display::PredictiveDisplay(nh_, "joint_states", prefix));
}

void PlannerInterface::getActiveGroupNames(std::vector<std::string> &active_groups)
{
  active_groups.clear();
  for (auto &g : active_groups_) {
    active_groups.push_back(g);
  }
}

void PlannerInterface::setupParameters() {

  std::string node_name = ros::this_node::getName();
  ROS_DEBUG("PlannerInterface::setupParameters() -- nh namespace: %s", node_name.c_str());
  if(!nh_.getParam(node_name + "/param/trajectory_scaler", trajectory_scaler_)) {
    nh_.setParam(node_name + "/param/trajectory_scaler", trajectory_scaler_);
  }
  if(!nh_.getParam(node_name+"/param/trajectory_density", trajectory_density_)) {
    nh_.setParam(node_name+"/param/trajectory_density", trajectory_density_);
  }
  ROS_DEBUG("PlannerInterface::setupParameters()) -- trajectory scaler: %.5f, trajectory density: %.5f", trajectory_scaler_, trajectory_density_);

}


void PlannerInterface::getGroupNames(std::vector<std::string> &group_names) {
  group_names = rdf_model_->getGroups();
}

std::vector<std::string> PlannerInterface::getGroups() {
  return rdf_model_->getGroups();
}


bool PlannerInterface::removeGroup(const std::string &group_name)
{
  ROS_WARN("PathPlanner::removeGroup() -- %s", group_name.c_str());
  active_groups_.erase(std::remove(active_groups_.begin(), active_groups_.end(), group_name), active_groups_.end());
  joint_mask_.erase(group_name);
  return removeGroup_(group_name);
}

void PlannerInterface::getAllGroupJoints(const std::string &group_name, std::vector<std::string> &joints) {
  if(!rdf_model_->getJoints(group_name, joints))
    ROS_ERROR("PlannerInterface::getAllGroupJoints(%s) -- can't get group joints!!", group_name.c_str());
}

std::vector<std::string> PlannerInterface::getEndEffectorNames() {
  return rdf_model_->getEndEffectors();
}

std::string PlannerInterface::getControlFrame(const std::string &group_name)
{
  std::string control_frame;
  if(rdf_model_->getTipLink(group_name, control_frame)) {
    return control_frame;
  } else {

    if(rdf_model_->getEndEffectorBaseLink(group_name, control_frame))
      return control_frame;

    ROS_DEBUG("PlannerInterface::getControlFrame(%s) -- no control frame found", group_name.c_str());
    return "";
  }
}

void PlannerInterface::getStoredGroupState(const std::string &group_name, const std::string &group_state_name, sensor_msgs::JointState &group_state) {
  if(!rdf_model_->getGroupState(group_name, group_state_name, group_state)) {
    ROS_ERROR("PlannerInterface::getStoredGroupState(%s) -- problem getting group state %s!!", group_name.c_str(), group_state_name.c_str());
  }
}

void PlannerInterface::getStoredStateList(const std::string &group_name, std::vector<std::string> &group_state_list) {
  std::map<std::string, sensor_msgs::JointState> group_states;
  if(rdf_model_->getGroupStateMap(group_name, group_states)) {
    group_state_list.clear();
    for(auto &g : group_states) {
      group_state_list.push_back(g.first);
    }
  } else {
    ROS_ERROR("PlannerInterface::getStoredStateList(%s) -- error getting stored states!!", group_name.c_str());
  }
}

void PlannerInterface::getControlMesh(const std::string &group_name, std::string &control_mesh) {
  ROS_WARN("[PlannerInterface::getControlMesh] this method is not implemented yet..");
}

bool PlannerInterface::getGroupShapeMarkers(const std::string &group_name, visualization_msgs::MarkerArray &markers) {
  markers = rdf_model_->getGroupMeshes(group_name);
  return (markers.markers.size() > 0);
}

void PlannerInterface::getConditioningMetrics(std::vector<std::string> &m) {
  for(auto &k: conditioning_metrics_)
    m.push_back(k);
}

void PlannerInterface::addConditioningMetric(std::string m) {
  conditioning_metrics_.push_back(m);
}

bool PlannerInterface::addPlanningGroup(const std::string &group_name,
                                        const std::string &group_type,
                                        const double joint_tolerance,
                                        double position_tolerances,
                                        double orientation_tolerances)
{

  ROS_INFO("PlannerInterface::addPlanningGroup() -- adding %s of type %s", group_name.c_str(), group_type.c_str());

  // safety checks on types
  if(group_type=="cartesian") {
    if(!rdf_model_->isChain(group_name)) {
      ROS_ERROR("PlannerInterface::addPlanningGroup() -- can't add %s as a CARTESIAN group unless chain is specified in SRDF", group_name.c_str());
      return false;
    }
  }

  if(group_type=="endeffector") {
    if(!rdf_model_->checkValidEndEffector(group_name)) {
      ROS_ERROR("PlannerInterface::addPlanningGroup() -- can't add %s as a ENDEFFECTOR group unless specified as one in SRDF", group_name.c_str());
      return false;
    }
  }

  // add planning group name if not already in active groups
  if (std::find(active_groups_.begin(), active_groups_.end(), group_name) == active_groups_.end())
    active_groups_.push_back(group_name);

  plan_generated_[group_name] = false;
  // stored_plans[group_name] = none TODO
  display_modes_[group_name] = "last_point";
  auto_execute_[group_name] = false;
  // group_types_[group_name] = "";
  control_frames_[group_name] = "";
  control_meshes_[group_name] = "";

  clearToolOffset(group_name);
  // marker_store_[group_name] = empty MarkerArray TODO

  // go get joints and figure out which are masked
  robot_interaction_tools_msgs::JointMask mask;
  std::vector<std::string> joints, full_joints;
  if(rdf_model_->getJoints(group_name, joints, false)) {
    if(rdf_model_->getJoints(group_name, full_joints, true)) {
      for(auto &j: full_joints) {
        mask.joint_names.push_back(j);
        bool m = std::find(joints.begin(), joints.end(), j) != joints.end();
        ROS_DEBUG("PlannerInterface::addPlanningGroup() -- group[%s] found joint: %s, masked=%d", group_name.c_str(), j.c_str(), 1-(int)m );
        mask.mask.push_back(m);
      }
      joint_mask_[group_name] = mask;
    }
  }

  position_tolerances_[group_name] = position_tolerances;
  orientation_tolerances_[group_name] = orientation_tolerances;
  joint_tolerance_[group_name] = joint_tolerance;

  if (!addPlanningGroup_(group_name))
    return false;

  // FIXME does this need TRY/CATCH blocks??
  group_id_offset_[group_name] = getRandomOffsetId();
  ROS_DEBUG("PlannerInterface::addPlanningGroup(%s) -- settled on random number %d", group_name.c_str(), group_id_offset_[group_name]);

  // CARTESIAN testing for now

  return true;
}

int PlannerInterface::getRandomOffsetId()
{
  static std::default_random_engine e;
  std::mt19937 mt(e());
  static std::uniform_int_distribution<int> d(1, RAND_MAX);
  int r = d(mt);
  ROS_DEBUG("PlannerInterface::getRandomOffsetId() -- coming up with random number %d",r);
  // see if the random id number already exists as an offset for groups
  bool r_exists = false;
  for (auto const & grp : group_id_offset_)
  {
    if (r == grp.second)
    {
      r_exists = true;
      break;
    }
  }

  // recursive call if it already exists
  if ( r_exists )
    return r = getRandomOffsetId();
  else
    return r;
}

bool PlannerInterface::getStoredStateMap(const std::string& group_name, std::map<std::string, sensor_msgs::JointState> &group_states) {
  return rdf_model_->getGroupStateMap(group_name, group_states);
}

void PlannerInterface::setToolOffset(const std::string &group_name, geometry_msgs::Pose offset) {
  tool_offset_[group_name] = offset;
}

rit_utils::GroupType PlannerInterface::getGroupType(const std::string &group_name) {
  return rit_utils::CARTESIAN;
}

bool PlannerInterface::clearToolOffset(const std::string &group_name) {
  tool_offset_[group_name] = geometry_msgs::Pose();
  tool_offset_[group_name].orientation.w = 1.0;
}

bool PlannerInterface::getToolOffset(const std::string &group_name, geometry_msgs::Pose &p) {
  if(tool_offset_.find(group_name) != std::end(tool_offset_)) {
    p = tool_offset_[group_name];
  } else {
    ROS_WARN("PlannerInterface::getToolOffset() -- %s not found in tool offset list", group_name.c_str());
    clearToolOffset(group_name);
  }
  return true;
}

void PlannerInterface::transformGoal(std::string group_name, geometry_msgs::PoseStamped goal_in, geometry_msgs::PoseStamped &goal_out) {

  ROS_DEBUG("PlannerInterface::transformGoalMap() -- initial goal pose: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f), frame_id: %s",
                                                  goal_in.pose.position.x, goal_in.pose.position.y, goal_in.pose.position.z,
                                                  goal_in.pose.orientation.x, goal_in.pose.orientation.y, goal_in.pose.orientation.z, goal_in.pose.orientation.w,
                                                  goal_in.header.frame_id.c_str());

  // transform the pose if necessary
  geometry_msgs::PoseStamped transformed_goal = goal_in;
  std::string planning_frame;
  if(getGroupPlanningFrame(group_name, planning_frame)) {
    if(goal_in.header.frame_id != planning_frame) {
      ROS_DEBUG("PlannerInterface::transformGoalMap() -- transforming pose from %s to %s", goal_in.header.frame_id.c_str(), planning_frame.c_str());
      tf_listener_.transformPose(planning_frame, ros::Time(0), goal_in, goal_in.header.frame_id, transformed_goal);
    }
    ROS_DEBUG("PlannerInterface::transformGoalMap() -- adding transformed goal pose: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f), frame_id: %s",
                                                       transformed_goal.pose.position.x, transformed_goal.pose.position.y, transformed_goal.pose.position.z,
                                                       transformed_goal.pose.orientation.x, transformed_goal.pose.orientation.y, transformed_goal.pose.orientation.z, transformed_goal.pose.orientation.w,
                                                       transformed_goal.header.frame_id.c_str());

    KDL::Frame T, Toffset;
    tf::poseMsgToKDL(transformed_goal.pose, T);
    tf::poseMsgToKDL(tool_offset_[group_name], Toffset);
    tf::poseKDLToMsg(T*Toffset.Inverse(), transformed_goal.pose);

    goal_out = transformed_goal;

  }
}


void PlannerInterface::transformGoalMap(std::map<std::string, std::vector<geometry_msgs::PoseStamped> > &goals) {

  // go through all the group goal arrays
  for(auto &req : goals) {

    std::string group_name = req.first;
    std::vector<geometry_msgs::PoseStamped> waypoints = req.second;

    ROS_DEBUG("PlannerInterface::transformGoalMap() -- adding group %s with %d pose goals", group_name.c_str(), (int)waypoints.size());

    std::map<std::string, std::vector<geometry_msgs::PoseStamped> > transformed_goals;
    for(auto &goal : waypoints) {

      ROS_DEBUG("PlannerInterface::transformGoalMap() -- initial goal pose: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f), frame_id: %s",
                                                      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                                                      goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w,
                                                      goal.header.frame_id.c_str());

      // transform the pose if necessary
      geometry_msgs::PoseStamped transformed_goal = goal;
      std::string planning_frame;
      if(getGroupPlanningFrame(group_name, planning_frame)) {
        if(goal.header.frame_id != planning_frame) {
          ROS_DEBUG("PlannerInterface::transformGoalMap() -- transforming pose from %s to %s", goal.header.frame_id.c_str(), planning_frame.c_str());
          tf_listener_.transformPose(planning_frame, ros::Time(0), goal, goal.header.frame_id, transformed_goal);
        }
        ROS_DEBUG("PlannerInterface::transformGoalMap() -- adding transformed goal pose: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f), frame_id: %s",
                                                           transformed_goal.pose.position.x, transformed_goal.pose.position.y, transformed_goal.pose.position.z,
                                                           transformed_goal.pose.orientation.x, transformed_goal.pose.orientation.y, transformed_goal.pose.orientation.z, transformed_goal.pose.orientation.w,
                                                           transformed_goal.header.frame_id.c_str());

        KDL::Frame T, Toffset;
        tf::poseMsgToKDL(transformed_goal.pose, T);
        tf::poseMsgToKDL(tool_offset_[group_name], Toffset);
        tf::poseKDLToMsg(T*Toffset.Inverse(), transformed_goal.pose);

        transformed_goals[group_name].push_back(transformed_goal);
      } else {
        ROS_WARN("PlannerInterface::transformGoalMap() -- no group name %s found", group_name.c_str());
      }
    }
    goals[group_name] = transformed_goals[group_name];

  }
}


bool PlannerInterface::plan(const std::string &group_name, const PlanningGoal &goal, bool execute, bool show_path, JointStateMap start_states) {
  std::string node_name = ros::this_node::getName();
  nh_.getParamCached(node_name+"/param/trajectory_density", trajectory_density_);
  if (trajectory_density_ <= 1e-4) {
    trajectory_density_ = 1e-4;
    nh_.setParam(node_name+"/param/trajectory_density", trajectory_density_);
  }
  nh_.getParamCached(node_name+"/param/trajectory_scaler", trajectory_scaler_);

  ROS_WARN("PlannerInterface::plan(%s) -- trajectory scaler: %.5f, trajectory density: %.5f", group_name.c_str(), trajectory_scaler_, trajectory_density_);

  std::vector<double> pos_tol = {goal.tolerance_bounds[0][1],goal.tolerance_bounds[1][1],goal.tolerance_bounds[2][1]};
  std::vector<double> rot_tol = {goal.tolerance_bounds[3][1],goal.tolerance_bounds[4][1],goal.tolerance_bounds[5][1]};

  is_partial_plan_=false;

  for (uint j=0;j<6; j++)
    ROS_WARN_STREAM("Tolerances: "<<goal.tolerance_bounds[j][1]);

  if(isReachable(group_name, goal.goal, pos_tol, rot_tol)==planner_interface::UNREACHABLE) {
    ROS_INFO("PlannerInterface::plan(%s) -- not reachable", group_name.c_str());
    return false;
  }

  std::map<std::string, PlanningGoal> transformed_goals;
  std::map<std::string, trajectory_msgs::JointTrajectory> trajectories;
  geometry_msgs::PoseStamped ps;
  transformGoal(group_name, goal.goal, ps);
  PlanningGoal pg = goal;
  pg.goal = ps;
  pg.offset=tool_offset_[group_name];
  transformed_goals[group_name] = pg;
  //setToolOffset(group_name, pg.offset);

  if(plan_(transformed_goals, trajectories, execute, show_path, start_states)) {
    display_robot_->createAnimation(group_name, trajectories[group_name], show_path);
    return true;
  } else {
    return false;
  }
}

bool PlannerInterface::plan(const std::map<std::string, PlanningGoal> &goals, bool execute, bool show_path, JointStateMap start_states) {
  std::string node_name = ros::this_node::getName();
  nh_.getParamCached(node_name+"/param/trajectory_density", trajectory_density_);
  if (trajectory_density_ <= 1e-4) {
    trajectory_density_ = 1e-4;
    nh_.setParam(node_name+"/param/trajectory_density", trajectory_density_);
  }
  nh_.getParamCached(node_name+"/param/trajectory_scaler", trajectory_scaler_);

  ROS_WARN("PlannerInterface::plan() -- trajectory scaler: %.5f, trajectory density: %.5f", trajectory_scaler_, trajectory_density_);

  std::map<std::string, PlanningGoal> transformed_goals;
  std::map<std::string, trajectory_msgs::JointTrajectory> trajectories;
  for(auto &req : goals) {
    geometry_msgs::PoseStamped ps;
    transformGoal(req.first, req.second.goal, ps);
    PlanningGoal pg = req.second;
    pg.offset = tool_offset_[req.first];
    pg.goal = ps;
    transformed_goals[req.first] = pg;
    // setToolOffset(req.first, pg.offset);

  }
  if(plan_(transformed_goals, trajectories, execute, show_path, start_states)) {
    for(auto &req : trajectories) {
      display_robot_->createAnimation(req.first, req.second, false);
    }
    if (show_path)
      playAnimation();
    return true;
  } else {
    return false;
  }
}


bool PlannerInterface::executePlans(const std::vector<std::string> &group_names) {
  std::string node_name = ros::this_node::getName();
  nh_.getParamCached(node_name+"/param/trajectory_scaler", trajectory_scaler_);
  if (trajectory_scaler_ <= 1e-4) {
    trajectory_scaler_ = 1e-4;
    nh_.setParam(node_name+"/param/trajectory_scaler", trajectory_scaler_);
  }
  return executePlans_(group_names);
}

bool PlannerInterface::planJointPath(const std::map<std::string, std::vector<sensor_msgs::JointState> > &goals, bool execute, bool show_path, JointStateMap start_states) {
  std::map<std::string, trajectory_msgs::JointTrajectory> trajectories;
  if (planJointPath_(goals, trajectories, execute, show_path, start_states)) {
    for(auto &req : trajectories) {
      display_robot_->createAnimation(req.first, req.second, show_path);
    }
    return true;
  } else {
    return false;
  }
}

void PlannerInterface::getGroupLinks(const std::string &group, std::vector<std::string> &links)
{
  if(!rdf_model_->getLinks(group, links))
    ROS_ERROR("[PlannerInterface::getGroupLinks] Could not get links for group %s", group.c_str());
}

void PlannerInterface::getGroupJoints(const std::string group, std::vector<std::string> &names)
{
  if(!rdf_model_->getJoints(group, names))
    ROS_ERROR("[PlannerInterface::getGroupJoints] Could not get joints for group %s", group.c_str());
}

void PlannerInterface::getBaseFrame(const std::string &group, std::string &base_frame)
{
  ROS_ERROR("[PlannerInterface::getBaseFrame]");
  // TODO is this how this should be implemented or is this a planner object thing?
  if(!rdf_model_->getBaseLink(group, base_frame))
    ROS_ERROR("[PlannerInterface::getBaseFrame] Could not get base frame for group %s", group.c_str());
}

void PlannerInterface::getGroupBaseFrame(const std::string &group_name, std::string &group_base_frame)
{
  ROS_ERROR("[PlannerInterface::getGroupBaseFrame]");
  getBaseFrame(group_name, group_base_frame);
}

bool PlannerInterface::getJointMask(const std::string &group, robot_interaction_tools_msgs::JointMask &mask)
{
  if(hasGroup(group)) {
    mask = joint_mask_[group];
    return true;
  }
  return false;
}

bool PlannerInterface::setJointMask(const std::string &group, const robot_interaction_tools_msgs::JointMask &mask)
{
  if(hasGroup(group)) {
    joint_mask_[group] = mask;
    return true;
  }
  return false;
}

bool PlannerInterface::setPositionTolerances(const std::string &group_name, double tols) {
  position_tolerances_[group_name] = tols;
  return true;
}

bool PlannerInterface::setOrientationTolerances(const std::string &group_name, double tols) {
  orientation_tolerances_[group_name] = tols;
  return true;
}
