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

#include <interactive_controls/interactive_controls.h>

using namespace interactive_controls;

InteractiveControls::InteractiveControls(const ros::NodeHandle& nh, std::string robot_name="robot", std::string plugin_planner="moveit_planner::MoveItPlanner") :
  planner_(NULL),
  planner_loader_("planner_interface", "planner_interface::PlannerInterface"),
  nh_(nh),
  use_tolerances_(false),
  robot_name_(robot_name),
  planner_plugin_(plugin_planner)
{

  // set up spin rate
  nh_.param("loop_rate", loop_rate_, 25.0); 

  ROS_DEBUG("InteractiveControls -- advertising services");

  std::string base = "/interactive_controls";
  config_srv_ = nh_.advertiseService(base + "/configure", &InteractiveControls::configService, this);
  get_info_srv_ = nh_.advertiseService(base + "/get_info", &InteractiveControls::getInfoService, this);

  // set up dummy tolerances until we load these in from file
  joint_tolerance_ = 0.1;
  position_tolerances_=0.01;
  orientation_tolerances_=0.005;

  // use the plugin factory to connect to appropriate planner plugin.  
  // this should come from config file 
  try {
    ROS_INFO("Setup %s plugin for: %s", planner_plugin_.c_str(), robot_name_.c_str());
    planner_ = planner_loader_.createInstance(planner_plugin_);
    planner_->initialize(nh_, robot_name);
    planner_->createDisplay("rit");
  } catch(pluginlib::PluginlibException& ex) {
    ROS_ERROR("InteractiveControls() -- The planner plugin failed to load for some reason. Error: %s", ex.what());
  }

  // reset IM server
  server_.reset( new interactive_markers::InteractiveMarkerServer(std::string(robot_name + "_interactive_controls_server"),"",false) );

  // get requested groups from param server
  parseGroupsFromParam();
  
  // setup menus
  setupMenuOptions();

  im_pub_ = nh_.advertise<visualization_msgs::InteractiveMarker>("plan_marker", 1000);
  ee_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("ee_markers", 1000);

  // setup the groups
  if(!setupGroups()) {
    ROS_ERROR("InteractiveControls() -- error setting up groups!!");
  }

  auto_display_ = true;
}

InteractiveControls::~InteractiveControls()
{ 
  planner_.reset();
  server_.reset();
}


void InteractiveControls::parseGroupsFromParam() 
{

  active_groups_.clear();

  std::vector<std::string> joint_groups;
  std::vector<std::string> cartesian_groups;
  std::vector<std::string> endeffector_groups;

  nh_.getParam("groups/cartesian", cartesian_groups); 
  nh_.getParam("groups/joint", joint_groups); 
  nh_.getParam("groups/endeffector", endeffector_groups); 

  for(auto &g: cartesian_groups) {
    ROS_DEBUG("InteractiveControls::parseGroupsFromParam() -- parsed CARTESIAN group: %s", g.c_str());
    active_groups_.push_back(g);
    group_type_map_[g] = CARTESIAN;
  }
  for(auto &g: joint_groups) {
    ROS_DEBUG("InteractiveControls::parseGroupsFromParam() -- parsed JOINT group: %s", g.c_str());
    active_groups_.push_back(g);
    group_type_map_[g] = JOINT;
  }
  for(auto &g: endeffector_groups) {
    ROS_DEBUG("InteractiveControls::parseGroupsFromParam() -- parsed ENDEFFECTOR group: %s", g.c_str());
    active_groups_.push_back(g);
    group_type_map_[g] = ENDEFFECTOR;
  }
} 

bool InteractiveControls::getInfoService(robot_interaction_tools_msgs::GetGroupConfiguration::Request  &req,
                                         robot_interaction_tools_msgs::GetGroupConfiguration::Response &res)
{
  getGroupConfigurations(res.group_configurations);
  return true;
}

void InteractiveControls::getGroupConfiguration(std::string group_name, robot_interaction_tools_msgs::PlanGroupConfiguration &group_configuration)
{

  ROS_DEBUG("InteractiveControls::getGroupConfiguration() -- getting gc for: %s", group_name.c_str());
  if(isGroup(group_name)) {
    ROS_DEBUG("InteractiveControls::getGroupConfiguration() -- %s is valid", group_name.c_str());

    group_configuration.group_name = group_name;
    // set if active (and the type if so)
    if(std::find(std::begin(active_groups_), std::end(active_groups_), group_name)!=active_groups_.end()) {
      group_configuration.group_type = getTypeString(group_type_map_[group_name]);
      group_configuration.is_active = true;   
    } else {
      group_configuration.is_active = false;
    }

    // add the auto flags
    group_configuration.plan_on_move = auto_plan_[group_name];
    group_configuration.execute_on_plan = auto_execute_[group_name];
    group_configuration.compute_cartesian_plans = plan_cartesian_paths_[group_name];
    // add the joint mask
    robot_interaction_tools_msgs::JointMask mask;
    if(planner_->getJointMask(group_name, mask)) {
      group_configuration.joint_mask = mask;
    }
    // add stored state names
    for(auto &p: stored_pose_map_[group_name]) {
      group_configuration.stored_pose_list.push_back(p.first);
    }
  
    // get tolerance info
    std::vector<std::string> tolerance_modes = planner_->toleranceUtil->getToleranceModes();
    for(auto &m : tolerance_modes) {
      robot_interaction_tools_msgs::ToleranceInfo tol_info;
      std::vector<std::string> tolerance_types;
      planner_->toleranceUtil->getToleranceTypes(m, tolerance_types);
      tol_info.mode = m;
      tol_info.types.push_back("User Defined");
      geometry_msgs::Vector3 v3;
      tol_info.vals.push_back(v3);

      for(auto &t : tolerance_types) {
        tolerance_util::ToleranceVal v;
        geometry_msgs::Vector3 v3;
        planner_->toleranceUtil->getToleranceVal(m, t, v);
        v3.x = v[0];
        v3.y = v[1];
        v3.z = v[2];
        tol_info.types.push_back(t);
        tol_info.vals.push_back(v3);
      }
      group_configuration.tolerances.push_back(tol_info);
    }
    // set current
    robot_interaction_tools_msgs::ToleranceInfo pos_tol_setting, rot_tol_setting;
    tolerance_util::ToleranceVal pos_tol_val, rot_tol_val;
    geometry_msgs::Vector3 pos_tol_vec, rot_tol_vec;
    planner_->toleranceUtil->getToleranceVal("position", group_position_tolerance_type_[group_name], pos_tol_val);
    planner_->toleranceUtil->getToleranceVal("orientation", group_orientation_tolerance_type_[group_name], rot_tol_val);

    std::vector<double> rosparam_tols1(3), rosparam_tols2(3);
    std::string node_name = ros::this_node::getName();
    nh_.getParam(node_name+"/param/"+group_name+"/tolerance_pos/x",rosparam_tols1[0]);
    nh_.getParam(node_name+"/param/"+group_name+"/tolerance_pos/y",rosparam_tols1[1]);
    nh_.getParam(node_name+"/param/"+group_name+"/tolerance_pos/z", rosparam_tols1[2]);
    nh_.getParam(node_name+"/param/"+group_name+"/tolerance_rot/roll",rosparam_tols2[0]);
    nh_.getParam(node_name+"/param/"+group_name+"/tolerance_rot/pitch",rosparam_tols2[1]);
    nh_.getParam(node_name+"/param/"+group_name+"/tolerance_rot/yaw",rosparam_tols2[2]);

    pos_tol_vec.x = rosparam_tols1[0];
    pos_tol_vec.y = rosparam_tols1[1];
    pos_tol_vec.z = rosparam_tols1[2];

    rot_tol_vec.x = rosparam_tols2[0];
    rot_tol_vec.y = rosparam_tols2[1];
    rot_tol_vec.z = rosparam_tols2[2];
    
    bool user_mode = false;
    for(uint i=0; i<3 && !user_mode; i++)
      if (pos_tol_val[i]!=rosparam_tols1[i]) 
        user_mode=true;

    if (user_mode) 
      group_position_tolerance_type_[group_name]="User Defined";

    user_mode=false;
    for(uint i=0; i<3 && !user_mode; i++)
      if (rot_tol_val[i]!=rosparam_tols2[i]) 
        user_mode=true;

    if (user_mode) 
      group_orientation_tolerance_type_[group_name]="User Defined";

    group_position_tolerances_[group_name]=rosparam_tols1;
    group_orientation_tolerances_[group_name]=rosparam_tols2;

    
    pos_tol_setting.mode = "position";
    pos_tol_setting.types.push_back(group_position_tolerance_type_[group_name]);
    pos_tol_setting.vals.push_back(pos_tol_vec);

    rot_tol_setting.mode = "orientation";
    rot_tol_setting.types.push_back(group_orientation_tolerance_type_[group_name]);
    rot_tol_setting.vals.push_back(rot_tol_vec);

    group_configuration.tolerance_settings.push_back(pos_tol_setting);
    group_configuration.tolerance_settings.push_back(rot_tol_setting);

    // get conditioning metric stuff
    planner_->getConditioningMetrics(group_configuration.conditioning_metrics);
    for(auto &m: group_configuration.conditioning_metrics) {
      ROS_DEBUG("InteractiveControls::getGroupConfiguration() -- found conditioning metric: %s", m.c_str());
    }
    group_configuration.conditioning_metric = conditioning_metrics_[group_name];

    ROS_DEBUG("InteractiveControls::getGroupConfiguration() -- %s, metric=%s", group_name.c_str(), group_configuration.conditioning_metric.c_str());
  }
}


void InteractiveControls::getGroupConfigurations(std::vector<robot_interaction_tools_msgs::PlanGroupConfiguration> &group_configurations) 
{
  std::vector<std::string> group_names;
  planner_->getGroupNames(group_names);
  for(auto &g: group_names) {
    ROS_DEBUG("InteractiveControls::getGroupConfigurations() -- found group %s", g.c_str());
    // create a new group configuration struct
    robot_interaction_tools_msgs::PlanGroupConfiguration gc;
    getGroupConfiguration(g, gc);
    group_configurations.push_back(gc);
  }
}


bool InteractiveControls::configService(robot_interaction_tools_msgs::ConfigureGroup::Request  &req,
                                        robot_interaction_tools_msgs::ConfigureGroup::Response &res)
{

  bool ret = true;

  ROS_INFO("InteractiveControls::configService() -- action: %d", (int)req.action);
    
  switch(req.action) {
    case robot_interaction_tools_msgs::ConfigureGroupRequest::GET_INFO :
      {
        getGroupConfigurations(res.group_configurations);
        return ret;  
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::ADD_GROUP:
      {
        std::string group_name = req.group_configuration.group_name;
        std::string group_type = req.group_configuration.group_type;

        if (std::find(std::begin(active_groups_), std::end(active_groups_), group_name)!=active_groups_.end()) {
          ROS_WARN("InteractiveControls::configService() -- Can't Add Group %s, already exists", group_name.c_str());
          ret = false;
        } else {                  
          active_groups_.push_back(group_name);
          group_type_map_[group_name] = getTypeFromString(group_type);
          ret = setupGroup(group_name, group_type_map_[group_name]);
        }
        getGroupConfigurations(res.group_configurations);
        return ret;
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::REMOVE_GROUP:
      {
        std::string group_name = req.group_configuration.group_name;

        if (std::find(std::begin(active_groups_), std::end(active_groups_), group_name)==active_groups_.end()) {
          ROS_WARN("InteractiveControls::configService() -- Can't Add Group %s, doesnt exist", group_name.c_str());
          ret = false;
        } else {                  
          ret = removeGroupMarkers(group_name);
        }
        getGroupConfigurations(res.group_configurations);
        return ret;
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::SET_MARKER_POSE:
      {
        std::string group_name = req.group_configuration.group_name;
        geometry_msgs::PoseStamped ps;
        try {
          ROS_INFO("got new pose in frame: %s", req.group_configuration.goal_pose.header.frame_id.c_str());
          ROS_INFO("need to transform to frame: %s", planner_->getControlFrame(group_name).c_str());
          listener_.transformPose(planner_->getControlFrame(group_name), req.group_configuration.goal_pose, ps);
          ROS_INFO("new pose: ");
          std::cout << ps << std::endl;
          ROS_INFO("huh?");
          
          geometry_msgs::Pose pose = ps.pose;
          if(isActiveGroup(group_name)) {
            if(group_type_map_[group_name] == CARTESIAN) {
              server_->setPose(markers_[group_name].name, pose);
              server_->applyChanges();
            }
          }
        } catch(const std::exception& e) {
          ROS_ERROR("InteractiveControls::configService() -- can't transform pose");
        }
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::SET_PLAN_ON_MOVE:
      {
        std::string group_name = req.group_configuration.group_name;
        auto_plan_[group_name] = req.group_configuration.plan_on_move;
        ret = setSubMenuCheckState(group_name, "Plan On Move", auto_plan_[group_name]);
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::SET_EXECUTE_ON_PLAN:
      {
        std::string group_name = req.group_configuration.group_name;
        auto_execute_[group_name] = req.group_configuration.execute_on_plan;
        ret = setSubMenuCheckState(group_name, "Execute On Plan", auto_execute_[group_name]);
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::SET_TOLERANCES:
      {
        try {
          for(auto &m: req.group_configuration.tolerance_settings) {
            tolerance_util::ToleranceVal val;
            if(m.mode == "position") {
              if(m.types.size()==0 && m.vals.size()==0) {
                planner_->toleranceUtil->getDefaultToleranceType(m.mode, group_position_tolerance_type_[req.group_configuration.group_name]);
                planner_->toleranceUtil->getToleranceVal(m.mode, group_position_tolerance_type_[req.group_configuration.group_name], val);
                group_position_tolerances_[req.group_configuration.group_name][0] = val[0];
                group_position_tolerances_[req.group_configuration.group_name][1] = val[1];
                group_position_tolerances_[req.group_configuration.group_name][2] = val[2];
                ROS_DEBUG("InteractiveControls::configService() -- no type or vals given, setting default %s", group_position_tolerance_type_[req.group_configuration.group_name].c_str());
              } else if(m.types.size()==0 && m.vals.size()>0) {
                group_position_tolerances_[req.group_configuration.group_name][0] = val[0] = m.vals[0].x;
                group_position_tolerances_[req.group_configuration.group_name][1] = val[1] = m.vals[0].y;
                group_position_tolerances_[req.group_configuration.group_name][2] = val[2] = m.vals[0].z;
                group_position_tolerance_type_[req.group_configuration.group_name]="User Defined";
                ROS_DEBUG("InteractiveControls::configService() -- no type given, setting from vals to %s", group_position_tolerance_type_[req.group_configuration.group_name].c_str());
              } else if(m.types.size()>0 && m.vals.size()==0) {
                group_position_tolerance_type_[req.group_configuration.group_name] = m.types[0];               
                if (m.types[0]!="User Defined") {
                  planner_->toleranceUtil->getToleranceVal(m.mode, m.types[0], val);
                  group_position_tolerances_[req.group_configuration.group_name][0] = val[0];
                  group_position_tolerances_[req.group_configuration.group_name][1] = val[1];
                  group_position_tolerances_[req.group_configuration.group_name][2] = val[2];
                }
                ROS_DEBUG("InteractiveControls::configService() -- no vals given, setting from type %s", group_position_tolerance_type_[req.group_configuration.group_name].c_str());
              } else {
                ROS_DEBUG("InteractiveControls::configService() -- both types and vals given, checking checking validity");
                val[0] = m.vals[0].x;
                val[1] = m.vals[0].y;
                val[2] = m.vals[0].z;
                std::string test_type;
                planner_->toleranceUtil->getToleranceType(m.mode, val, test_type);
                if(test_type==m.types[0]) {
                  group_position_tolerance_type_[req.group_configuration.group_name] = test_type;
                  group_position_tolerances_[req.group_configuration.group_name][0] = val[0];
                  group_position_tolerances_[req.group_configuration.group_name][1] = val[1];
                  group_position_tolerances_[req.group_configuration.group_name][2] = val[2];
                  ROS_DEBUG("InteractiveControls::configService() -- okay, setting from type %s", group_position_tolerance_type_[req.group_configuration.group_name].c_str());
                } else {
                  group_position_tolerance_type_[req.group_configuration.group_name] = m.types[0];
                  if (m.types[0]!="User Defined")
                    planner_->toleranceUtil->getToleranceVal(m.mode, m.types[0], val);
                  group_position_tolerances_[req.group_configuration.group_name][0] = val[0];
                  group_position_tolerances_[req.group_configuration.group_name][1] = val[1];
                  group_position_tolerances_[req.group_configuration.group_name][2] = val[2];
                  ROS_DEBUG("InteractiveControls::configService() -- something out of sync, setting from type %s (not vals)", group_position_tolerance_type_[req.group_configuration.group_name].c_str());                  
                }
              }

              std::string node_name = ros::this_node::getName();
              nh_.setParam(node_name + "/param/"+req.group_configuration.group_name+"/tolerance_pos/x",group_position_tolerances_[req.group_configuration.group_name][0]);
              nh_.setParam(node_name + "/param/"+req.group_configuration.group_name+"/tolerance_pos/y",group_position_tolerances_[req.group_configuration.group_name][1]);
              nh_.setParam(node_name + "/param/"+req.group_configuration.group_name+"/tolerance_pos/z",group_position_tolerances_[req.group_configuration.group_name][2]);



              std::vector<std::string> types;
              tolerance_util::ToleranceVal tol_val;
              if(planner_->toleranceUtil->getToleranceTypes("position", types)) {
                std::string menu_text = "Position Tolerance";
                for(auto &t: types) {
                  MenuHandleKey key;
                  key[req.group_configuration.group_name] = {menu_text, t};
                  if(group_position_tolerance_type_[req.group_configuration.group_name]==t) {
                    marker_menus_[req.group_configuration.group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
                  } else {
                    marker_menus_[req.group_configuration.group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );          
                  }
                }
              }
            } else if(m.mode == "orientation") {
              if(m.types.size()==0 && m.vals.size()==0) {
                planner_->toleranceUtil->getDefaultToleranceType(m.mode, group_orientation_tolerance_type_[req.group_configuration.group_name]);
                planner_->toleranceUtil->getToleranceVal(m.mode, group_orientation_tolerance_type_[req.group_configuration.group_name], val);
                group_orientation_tolerances_[req.group_configuration.group_name][0] = val[0];
                group_orientation_tolerances_[req.group_configuration.group_name][1] = val[1];
                group_orientation_tolerances_[req.group_configuration.group_name][2] = val[2];
                ROS_DEBUG("InteractiveControls::configService() -- no type or vals given, setting default %s", group_orientation_tolerance_type_[req.group_configuration.group_name].c_str());
              } else if(m.types.size()==0 && m.vals.size()>0) {
                group_orientation_tolerances_[req.group_configuration.group_name][0] = val[0] = m.vals[0].x;
                group_orientation_tolerances_[req.group_configuration.group_name][1] = val[1] = m.vals[0].y;
                group_orientation_tolerances_[req.group_configuration.group_name][2] = val[2] = m.vals[0].z;
                group_orientation_tolerance_type_[req.group_configuration.group_name]="User Defined";
                ROS_DEBUG("InteractiveControls::configService() -- no type given, setting from vals to %s", group_orientation_tolerance_type_[req.group_configuration.group_name].c_str());
              } else if(m.types.size()>0 && m.vals.size()==0) {
                group_orientation_tolerance_type_[req.group_configuration.group_name] = m.types[0];  
                if (m.types[0]!="User Defined") {
                  planner_->toleranceUtil->getToleranceVal(m.mode, m.types[0], val);
                  group_orientation_tolerances_[req.group_configuration.group_name][0] = val[0];
                  group_orientation_tolerances_[req.group_configuration.group_name][1] = val[1];
                  group_orientation_tolerances_[req.group_configuration.group_name][2] = val[2];
                }
                ROS_DEBUG("InteractiveControls::configService() -- no vals given, setting from type %s", group_orientation_tolerance_type_[req.group_configuration.group_name].c_str());
              } else {
                ROS_DEBUG("InteractiveControls::configService() -- both types and vals given, checking checking validity");
                val[0] = m.vals[0].x;
                val[1] = m.vals[0].y;
                val[2] = m.vals[0].z;
                std::string test_type;
                planner_->toleranceUtil->getToleranceType(m.mode, val, test_type);
                if(test_type==m.types[0]) {
                  group_orientation_tolerance_type_[req.group_configuration.group_name] = test_type;
                  group_orientation_tolerances_[req.group_configuration.group_name][0] = val[0];
                  group_orientation_tolerances_[req.group_configuration.group_name][1] = val[1];
                  group_orientation_tolerances_[req.group_configuration.group_name][2] = val[2];
                  ROS_DEBUG("InteractiveControls::configService() -- okay, setting from type %s", group_orientation_tolerance_type_[req.group_configuration.group_name].c_str());
                } else {
                  group_orientation_tolerance_type_[req.group_configuration.group_name] = m.types[0];
                  if (m.types[0]!="User Defined")
                    planner_->toleranceUtil->getToleranceVal(m.mode, m.types[0], val);
                  group_orientation_tolerances_[req.group_configuration.group_name][0] = val[0];
                  group_orientation_tolerances_[req.group_configuration.group_name][1] = val[1];
                  group_orientation_tolerances_[req.group_configuration.group_name][2] = val[2];
                  ROS_DEBUG("InteractiveControls::configService() -- something out of sync, setting from type %s (not vals)", group_orientation_tolerance_type_[req.group_configuration.group_name].c_str());                  
                }
              }

              std::string node_name = ros::this_node::getName();
              nh_.setParam(node_name + "/param/"+req.group_configuration.group_name+"/tolerance_rot/roll",group_orientation_tolerances_[req.group_configuration.group_name][0]);
              nh_.setParam(node_name + "/param/"+req.group_configuration.group_name+"/tolerance_rot/pitch",group_orientation_tolerances_[req.group_configuration.group_name][1]);
              nh_.setParam(node_name + "/param/"+req.group_configuration.group_name+"/tolerance_rot/yaw",group_orientation_tolerances_[req.group_configuration.group_name][2]);



              std::vector<std::string> types;
              tolerance_util::ToleranceVal tol_val;
              if(planner_->toleranceUtil->getToleranceTypes("orientation", types)) {
                std::string menu_text = "Orientation Tolerance";
                for(auto &t: types) {
                  MenuHandleKey key;
                  key[req.group_configuration.group_name] = {menu_text, t};
                  if(group_orientation_tolerance_type_[req.group_configuration.group_name]==t) {
                    marker_menus_[req.group_configuration.group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
                  } else {
                    marker_menus_[req.group_configuration.group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );          
                  }
                }
              }
            } else {
              ROS_WARN("InteractiveControls::configService() -- unknown tolerance mode %s", m.mode.c_str());
            }
          }
        } catch(...) {
          ROS_ERROR("InteractiveControls::configService() -- bad formed tolerance setting");
        }
      }
      marker_menus_[req.group_configuration.group_name].apply( *server_, req.group_configuration.group_name );
      server_->applyChanges();

      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::SET_JOINT_MASK:
      {
        int idx = 0;
        for(auto &jnt: req.group_configuration.joint_mask.joint_names) {
            setJointMaskMenuCheckState(req.group_configuration.group_name, jnt, req.group_configuration.joint_mask.mask[idx++]);
        }    
        planner_->setJointMask(req.group_configuration.group_name, req.group_configuration.joint_mask);
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::EXECUTE_STORED_POSE:
      {
        doStoredPose(req.group_configuration.group_name, req.group_configuration.stored_pose, true);
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::TOGGLE_POSTURE_CONTROLS:
      {
      // TODO
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::SYNC_MARKERS_TO_ROBOT:
      {
        resetCartesianGroupMarker(req.group_configuration.group_name);
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::PLAN_TO_MARKER:
      {
        planToMarker(req.group_configuration.group_name);
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::EXECUTE_PLAN:
      {
        std::vector<std::string> groups;
        groups.push_back(req.group_configuration.group_name);
        planner_->executePlans(groups); 
        if(group_type_map_[req.group_configuration.group_name] == CARTESIAN) {
          resetCartesianGroupMarker(req.group_configuration.group_name);
        }
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::SET_CARTESIAN_PLANNING:
      {
        std::string group_name = req.group_configuration.group_name;
        plan_cartesian_paths_[group_name] = req.group_configuration.compute_cartesian_plans;
        ret = setSubMenuCheckState(group_name, "Use Cartesian Plans", plan_cartesian_paths_[group_name]);
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::SET_TOOL_OFFSET:
      {
        visualization_msgs::InteractiveMarker im;
        server_->get(req.group_configuration.group_name, im);
        planner_->setToolOffset(req.group_configuration.group_name, im.pose);
        std::string tf_frame_name = getEEFrameName(req.group_configuration.group_name);
        std::string tf_tool_frame_name = getToolFrameName(req.group_configuration.group_name);
        geometry_msgs::PoseStamped pt_ee, pt_tool;
        pt_ee.header.frame_id = im.header.frame_id;
        pt_ee.pose.orientation.w = 1.0;
        frame_store_[tf_frame_name].second = pt_ee;  
        pt_tool.header.frame_id = tf_frame_name;
        pt_tool.pose = im.pose;
        frame_store_[tf_tool_frame_name].second = pt_tool;  
        resetCartesianGroupMarker(req.group_configuration.group_name);
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::CLEAR_TOOL_OFFSET:
      {
        visualization_msgs::InteractiveMarker im;
        server_->get(req.group_configuration.group_name, im);
        planner_->clearToolOffset(req.group_configuration.group_name);
        std::string tf_frame_name = getEEFrameName(req.group_configuration.group_name);
        std::string tf_tool_frame_name = getToolFrameName(req.group_configuration.group_name);
        geometry_msgs::PoseStamped pt_ee, pt_tool;
        pt_tool.header.frame_id = tf_frame_name;
        pt_tool.pose.orientation.w = 1.0;
        frame_store_[tf_tool_frame_name].second = pt_tool;  
        pt_ee.pose = im.pose;
        pt_ee.header.frame_id = im.header.frame_id;
        frame_store_[tf_frame_name].second = pt_ee;  
        resetCartesianGroupMarker(req.group_configuration.group_name);
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::SET_CONDITIONING_METRIC:
      {
        ROS_WARN("setting conditioning metric: %s", req.group_configuration.conditioning_metric.c_str());
        conditioning_metrics_[req.group_configuration.group_name] = req.group_configuration.conditioning_metric;
      }
      break;
    case robot_interaction_tools_msgs::ConfigureGroupRequest::SET_TASK_COMPATIBILITY:
      {
        task_compatibility_vectors_[req.group_configuration.group_name] = req.group_configuration.compatibility_vector;
      }
      break;
    default:
      ROS_ERROR("InteractiveControls::configService() -- unknown action: %d", (int)req.action);
      break;
  }

  // populate the return val with current state of things
  robot_interaction_tools_msgs::PlanGroupConfiguration gc;
  getGroupConfiguration(req.group_configuration.group_name, gc);
  res.group_configurations.push_back(gc);
  
  return ret;
}

bool InteractiveControls::removePlanningGroup(robot_interaction_tools_msgs::PlanGroupConfiguration config)
{
  bool result = planner_->removeGroup(config.group_name);
  
  if (result)
    ROS_INFO("[InteractiveControls::removePlanningGroup] Removed planning group %s.", config.group_name.c_str());
  else
    ROS_ERROR("[InteractiveControls::removePlanningGroup] Failed to remove planning group %s.", config.group_name.c_str());
  
  return result;
}

bool InteractiveControls::executePlan(robot_interaction_tools_msgs::PlanGroupConfiguration config)
{
  ROS_INFO("[InteractiveControls::executePlan] Starting to execute plan...");

  std::vector<std::string> group_names {config.group_name};
  bool result = planner_->executePlans(group_names);
  if (result)
    ROS_ERROR("[InteractiveControls::executePlan] Failed to excute plan!!");
  
  return result;
}

bool InteractiveControls::planToGoal(const std::string &group_name, const geometry_msgs::PoseStamped &pt)
{
  planner_interface::PlanningGoal goal;
  if(plan_cartesian_paths_[group_name]) {
    goal.type = planner_interface::CARTESIAN;
  } else {
    goal.type = planner_interface::JOINT;
  }
  ROS_INFO_STREAM("[InteractiveControls::planToGoal] Generating "<<(plan_cartesian_paths_[group_name] ? "Cartesian" : "Joint")<<" path..."); 
  goal.goal = pt;
  goal.offset = tool_offsets_[group_name];
  goal.conditioning_metric = conditioning_metrics_[group_name];
  goal.task_compatibility = task_compatibility_vectors_[group_name];

  goal.tolerance_bounds[0][1] = group_position_tolerances_[group_name][0];
  goal.tolerance_bounds[1][1] = group_position_tolerances_[group_name][1];
  goal.tolerance_bounds[2][1] = group_position_tolerances_[group_name][2];
  goal.tolerance_bounds[3][1] = group_orientation_tolerances_[group_name][0];
  goal.tolerance_bounds[4][1] = group_orientation_tolerances_[group_name][1];
  goal.tolerance_bounds[5][1] = group_orientation_tolerances_[group_name][2];

  for(uint i=0;i<3;i++) {
    goal.tolerance_bounds[i][1] = std::abs(goal.tolerance_bounds[i][1]);
    goal.tolerance_bounds[i][0] = -goal.tolerance_bounds[i][1];
    goal.tolerance_bounds[i+3][1] = std::abs(goal.tolerance_bounds[i+3][1]);
    goal.tolerance_bounds[i+3][0] = -goal.tolerance_bounds[i+3][1];
  }

  planner_->resetAnimation();
  bool plan_found = planner_->plan(group_name, goal, auto_execute_[group_name], auto_display_);
  
  if(group_type_map_[group_name] == CARTESIAN) {
    std_msgs::ColorRGBA ee_color;
    if(plan_found) {
      ee_color = getColorMsg(0.0,1.0,0.0,0.8);
    } else {
      if(planner_->isPartialPlan()) {
        ee_color = getColorMsg(1.0,1.0,0.0,0.8);
      } else {
        ee_color = getColorMsg(1.0,0.0,0.0,0.8);        
      }
    }
    setCartesianEEColor(group_name, ee_color, 0.95);

  }

  if (!plan_found) {
    ROS_ERROR("[InteractiveControls::planToGoal] Failed to plan!!");
  }

  return plan_found;

}

void InteractiveControls::setupMenuOptions() 
{

  // joint menu options
  joint_menu_options_.clear();
  joint_menu_options_.push_back(MenuConfig("Execute", false));
  joint_menu_options_.push_back(MenuConfig("Execute On Plan", true));
  joint_menu_options_.push_back(MenuConfig("Show Path", true));
  // joint_menu_options_.push_back(MenuConfig("Toggle Joint Control", false));
  joint_menu_options_.push_back(MenuConfig("Stored Poses", false));
  joint_menu_options_.push_back(MenuConfig("(Re)Play Plan", false));
  joint_menu_options_.push_back(MenuConfig("Loop Animation", true));
  joint_menu_options_.push_back(MenuConfig("Autoplay", true));

  // cartesian menu options
  cartesian_menu_options_.clear();
  cartesian_menu_options_.push_back(MenuConfig("Plan", false));
  cartesian_menu_options_.push_back(MenuConfig("Execute", false));
  cartesian_menu_options_.push_back(MenuConfig("Use Cartesian Plans", true));
  cartesian_menu_options_.push_back(MenuConfig("Plan On Move", true));
  cartesian_menu_options_.push_back(MenuConfig("Execute On Plan", true));
  cartesian_menu_options_.push_back(MenuConfig("Show Path", true));       
  cartesian_menu_options_.push_back(MenuConfig("Tool Offset", false));
  cartesian_menu_options_.push_back(MenuConfig("Stored Poses", false));       
  cartesian_menu_options_.push_back(MenuConfig("Position Tolerance", false));       
  cartesian_menu_options_.push_back(MenuConfig("Orientation Tolerance", false));       
  cartesian_menu_options_.push_back(MenuConfig("Sync To Actual", false));
  cartesian_menu_options_.push_back(MenuConfig("Joint Mask", false));
  cartesian_menu_options_.push_back(MenuConfig("(Re)Play Plan", false));
  cartesian_menu_options_.push_back(MenuConfig("Loop Animation", true));
  cartesian_menu_options_.push_back(MenuConfig("Autoplay", true));
  //cartesian_menu_options_.push_back(MenuConfig("Toggle Joint Control", false));
  
  if(use_tolerances_) {
    for(auto &mode : tolerance_modes_) {
      cartesian_menu_options_.push_back(MenuConfig(mode, false));
    }
  }

  // ee menu options
  endeffector_menu_options_.clear();
  endeffector_menu_options_.push_back(MenuConfig("Stored Poses", false));     

  // copy all to single master list
  for(auto &o : joint_menu_options_) {
    menu_options_.push_back(o);
  }
  for(auto &o : cartesian_menu_options_) {
    menu_options_.push_back(o);
  }
  for(auto &o : endeffector_menu_options_) {
    menu_options_.push_back(o);
  }

}


bool InteractiveControls::setupGroups() 
{
  control_frames_.clear();
  tool_offsets_.clear();
  for (auto& g : active_groups_) {
    ROS_INFO("InteractiveControls() -- setting up group: %s", g.c_str());
    control_frames_[g] = planner_->getControlFrame(g);
    if(control_frames_[g] != "") {
      ROS_INFO("InteractiveControls() -- control_frame: %s!!", control_frames_[g].c_str());
    }
    if(!setupGroup(g, group_type_map_[g])) {
      ROS_ERROR("InteractiveControls() -- error setting up group: %s!!", g.c_str());
      return false;
    }
  }
  return true;
}


bool InteractiveControls::setupGroup(const std::string &group_name, GroupType group_type) 
{

  // error checking
  if(isInTypeMap(group_name)) {
    group_type = group_type_map_[group_name];
  } else {
    ROS_ERROR("InteractiveControls::setupGroup() -- group[%s] not in type map", group_name.c_str());
    return false;
  }
  if(!planner_->addPlanningGroup(group_name, getTypeString(group_type), joint_tolerance_, position_tolerances_, orientation_tolerances_)) {
    group_type_map_.erase(group_name);
    ROS_ERROR("InteractiveControls::setupGroup() -- planner rejected group[%s]", group_name.c_str());
    return false;
  }
  
  // set up defaults for more complex planning stuff    
  tolerance_util::ToleranceVal pos_val, rot_val;
  planner_->toleranceUtil->getDefaultToleranceType("position", group_position_tolerance_type_[group_name]);
  planner_->toleranceUtil->getDefaultToleranceType("orientation", group_orientation_tolerance_type_[group_name]);
  planner_->toleranceUtil->getDefaultToleranceVal("position", pos_val);
  planner_->toleranceUtil->getDefaultToleranceVal("orientation", rot_val);
  std::vector<double> pos_tol = {pos_val[0], pos_val[1], pos_val[2]};
  std::vector<double> rot_tol = {rot_val[0], rot_val[1], rot_val[2]};
  group_position_tolerances_[group_name] = pos_tol;
  group_orientation_tolerances_[group_name] = rot_tol;


  std::string node_name = ros::this_node::getName();
  nh_.setParam(node_name + "/param/"+group_name+"/tolerance_pos/x", pos_val[0]);
  nh_.setParam(node_name + "/param/"+group_name+"/tolerance_pos/y", pos_val[1]);
  nh_.setParam(node_name + "/param/"+group_name+"/tolerance_pos/z", pos_val[2]);
  nh_.setParam(node_name + "/param/"+group_name+"/tolerance_rot/roll", rot_val[0]);
  nh_.setParam(node_name + "/param/"+group_name+"/tolerance_rot/pitch", rot_val[1]);
  nh_.setParam(node_name + "/param/"+group_name+"/tolerance_rot/yaw", rot_val[2]);
  
  geometry_msgs::Pose tc;
  tc.position.x = 1.0;
  tc.position.y = 1.0;
  tc.position.z = 1.0;
  tc.orientation.x = 0.0;
  tc.orientation.y = 0.0;
  tc.orientation.z = 0.0;
  tc.orientation.w = 1.0;
  task_compatibility_vectors_[group_name] = tc;
  std::vector<std::string> m;
  planner_->getConditioningMetrics(m);
  if (!m.empty())
    conditioning_metrics_[group_name] = m[0];

  // get stored poses from model
  planner_->getStoredStateMap(group_name, stored_pose_map_[group_name]);

  // create interactive markers for group
  ROS_INFO("InteractiveControls::setupGroup() -- setting up markers for %s", group_name.c_str());
  return initializeGroupMarkers(group_name);

}


bool InteractiveControls::initializeGroupMarkers(const std::string &group_name)
{

  bool ret;
  // get the group type if possible
  GroupType group_type;
  if(isInTypeMap(group_name)) {
    group_type = group_type_map_[group_name];
  } else {
    ROS_ERROR("InteractiveControls::initializeGroupMarkers() -- group[%s] not in type map", group_name.c_str());
    return false;
  }
 
  // set up the new interactive marker for the group
  markers_[group_name] = visualization_msgs::InteractiveMarker();
  markers_[group_name].name = group_name;
  markers_[group_name].description = group_name;
  
  // set up the menu for this group
  marker_menus_[group_name] = interactive_markers::MenuHandler();

  // initialize the group according to its type
  if(group_type==CARTESIAN) {
    ret = initializeCartesianGroup(group_name);
  } else if(group_type==JOINT) {
    ret = initializeJointGroup(group_name);
  } else if(group_type==ENDEFFECTOR) {
    ret = initializeEndEffectorGroup(group_name);
  } else {
    ROS_ERROR("InteractiveControls::initializeGroupMarkers() -- can't parse group[%s] type!!", group_name.c_str());
    return false;
  }

  if(ret) {

    // insert marker and menus
    server_->insert(markers_[group_name]);
    server_->setCallback(markers_[group_name].name, boost::bind( &InteractiveControls::processFeedback, this, _1 ));

    // bookkeeping
    auto_execute_[group_name] = false;
    auto_plan_[group_name] = false;
    plan_cartesian_paths_[group_name] = false;

    // posture IMs for each joint in the group
    posture_markers_[group_name].clear();
    show_posture_markers_[group_name] = false;

    // add menus to server
    marker_menus_[group_name].apply( *server_, group_name );
        
    server_->applyChanges();
  }

  return true;

}

bool InteractiveControls::initializeCartesianGroup(const std::string &group_name) 
{    

  ROS_INFO("InteractiveControls::initializeCartesianGroup() -- initializing group[%s] of type CARTESIAN", group_name.c_str());

  setupCartesianMenus(group_name);

  // create an ee helper instance for drawing the hand
  std::string tf_frame_name = getEEFrameName(group_name);
  std::string tf_tool_frame_name = getToolFrameName(group_name);
  std::string ee_name, ee_group_name, ee_link;
  visualization_msgs::MarkerArray ee_markers;
  planner_->rdf_model_->getEndEffector(group_name, ee_name);
  planner_->rdf_model_->getEndEffectorPlanningGroup(ee_name, ee_group_name);
  planner_->rdf_model_->getEndEffectorBaseLink(ee_group_name, ee_link);

  geometry_msgs::Pose offset;
  planner_->getToolOffset(group_name, offset);
  tool_offsets_[group_name] = offset;

  ROS_DEBUG("found ee %s (group=%s) of group %s with link %s", ee_group_name.c_str(), ee_name.c_str(), group_name.c_str(), ee_link.c_str());

  // set poses for hand and tool and store them in frame_store
  geometry_msgs::PoseStamped pt, pt_tool;
  pt.header.frame_id = planner_->getControlFrame(group_name);
  pt.pose.orientation.w = 1.0; 
  pt_tool.header.frame_id = tf_frame_name;
  pt_tool.pose = tool_offsets_[group_name];
 
  frame_store_[tf_frame_name] = FrameInfo(tf_frame_name, pt);
  frame_store_[tf_tool_frame_name] = FrameInfo(tf_tool_frame_name, pt_tool);

  ee_helper_map_[group_name].reset(new end_effector_helper::EndEffectorHelper(ee_group_name, ee_link, planner_->rdf_model_));
  //ee_helper_map_[group_name]->getCurrentPositionMarkerArray(ee_markers, ee_link);

  // make the marker have full 6-DOF controls at the control frame 
  markers_[group_name].controls = rit_utils::MarkerHelper::make6DOFControls();
  markers_[group_name].header.frame_id = planner_->getControlFrame(group_name);;
  markers_[group_name].scale = 0.2;

  // ROS_WARN("found %d markers for ee %s (group=%s) of group %s", (int)ee_markers.markers.size(), ee_group_name.c_str(), ee_name.c_str(), group_name.c_str());
  // for(auto &m: ee_markers.markers) {
  //    m.color = getColorMsg(0.0,1.0,0.0,0.8);
  //  markers_[group_name].controls[markers_[group_name].controls.size()-1].markers.push_back(m);
  // }

  // create marker and menus 
  InteractiveMarkerControl menu_control;
  menu_control.interaction_mode = InteractiveMarkerControl::BUTTON;
  markers_[group_name].controls.push_back(menu_control);
  server_->insert(markers_[group_name], boost::bind( &InteractiveControls::processFeedback, this, _1 ));


  return resetCartesianGroupMarker(group_name);

}

bool InteractiveControls::initializeJointGroup(const std::string &group_name) 
{    

  ROS_INFO("InteractiveControls::initializeJointGroup() -- initializing group[%s] of type JOINT", group_name.c_str());
  setupJointMenus(group_name);
  markers_[group_name].header.stamp = ros::Time(0);
  visualization_msgs::MarkerArray markers;
  if(planner_->getGroupShapeMarkers(group_name, markers)) {
    ROS_INFO("InteractiveControls::initializeJointGroup() -- found %d markers", (int)markers.markers.size());
    // create marker and menus 
    InteractiveMarkerControl menu_control;
    menu_control.interaction_mode = InteractiveMarkerControl::BUTTON;
    for(auto &m: markers.markers) {
      markers_[group_name].header.frame_id = m.header.frame_id;
      m.header.stamp = ros::Time(0);
      m.color.a = 0.1;
      m.scale = scaleVector3Msg(m.scale, 1.03);
      menu_control.markers.push_back(m);
    }
    markers_[group_name].controls.push_back(menu_control);   
    return true;
  } else {
    ROS_ERROR("InteractiveControls::initializeJointGroup() -- got no shape markers for group %s!!", group_name.c_str());
    return false;
  }

}

bool InteractiveControls::initializeEndEffectorGroup(const std::string &group_name) 
{    
  ROS_INFO("InteractiveControls::initializeEndEffectorGroup() -- initializing group[%s] of type ENDEFFECTOR", group_name.c_str());
  setupEndEffectorMenus(group_name);
  markers_[group_name].header.stamp = ros::Time(0); 
  std::string control_frame = planner_->getControlFrame(group_name);
  if(control_frame=="") {
    planner_->getGroupPlanningFrame(group_name, control_frame);
  }
  markers_[group_name].header.frame_id = control_frame;  
  visualization_msgs::MarkerArray markers;
  if(planner_->getGroupShapeMarkers(group_name, markers)) {
    // create marker and menus 
    InteractiveMarkerControl menu_control;
    menu_control.interaction_mode = InteractiveMarkerControl::BUTTON;
    for(auto &m: markers.markers) {
      m.header.stamp = ros::Time(0);
      m.color.a = 0.1;
      m.scale = scaleVector3Msg(m.scale, 1.03);
      menu_control.markers.push_back(m);
    }
    markers_[group_name].controls.push_back(menu_control);   
    im_pub_.publish(markers_[group_name]);
    return true;
  } else {
    ROS_ERROR("InteractiveControls::initializeEndEffectorGroup() -- got no shape markers for group %s!!", group_name.c_str());
    return false;
  }
}

void InteractiveControls::setupJointMenus(const std::string& group_name) 
{

  for(auto& o : joint_menu_options_) {
    if(o.first == "Stored Poses") {
      setupStoredPoseMenu(group_name);
    } else if(o.first == "Joint Mask") {
      setupJointMaskMenu(group_name);
    } else {
      setupSimpleMenuItem(group_name, o.first, o.second);
    }
  }
  auto_execute_[group_name] = true;
  
  // setup default checkbox stuff
  MenuHandleKey key;
  key[group_name] = {"Show Path"};
  marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  show_path_[group_name] = true;

  key[group_name] = {"Autoplay"};
  marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  show_path_[group_name] = true;
}

void InteractiveControls::setupEndEffectorMenus(const std::string& group_name) 
{
  for(auto& o : endeffector_menu_options_) {
    if(o.first == "Stored Poses") {
      setupStoredPoseMenu(group_name);
    } else {
      setupSimpleMenuItem(group_name, o.first, o.second);
    }
  }
}


void InteractiveControls::setupCartesianMenus(const std::string& group_name) 
{
  
  for(auto& o : cartesian_menu_options_) {
    if(o.first == "Stored Poses") {
      setupStoredPoseMenu(group_name);
    } else if(o.first == "Joint Mask") {
      setupJointMaskMenu(group_name);
    } else if(o.first == "Tool Offset") {
      setupToolOffsetMenu(group_name);
    } else if(o.first == "Position Tolerance") {
      setupPositionToleranceMenu(group_name);
    } else if(o.first == "Orientation Tolerance") {
      setupOrientationToleranceMenu(group_name);
    } else {
      setupSimpleMenuItem(group_name, o.first, o.second);
    }
  }
  // setup default checkbox stuff
  MenuHandleKey key;
  key[group_name] = {"Show Path"};
  marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  show_path_[group_name] = true;

  key[group_name] = {"Autoplay"};
  marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  show_path_[group_name] = true;
}


void InteractiveControls::setupStoredPoseMenu(const std::string& group_name)
{
  std::string menu_text = "Stored Poses";
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = marker_menus_[group_name].insert( menu_text );
  for(auto &p: stored_pose_map_[group_name]) {
    std::string state_name = p.first;
    MenuHandleKey key;
    key[group_name] = {menu_text, state_name};
    group_menu_handles_[key] = marker_menus_[group_name].insert( sub_menu_handle, state_name, boost::bind( &InteractiveControls::storedPoseCallback, this, _1 ) );
  }
}

void InteractiveControls::setupJointMaskMenu(const std::string& group_name)
{
  robot_interaction_tools_msgs::JointMask mask;
  if(planner_->getJointMask(group_name, mask)) {
    std::string menu_text = "Joint Mask";
    interactive_markers::MenuHandler::EntryHandle sub_menu_handle = marker_menus_[group_name].insert( menu_text );
    int idx = 0;
    for(auto &jnt: mask.joint_names) {
      MenuHandleKey key;
      key[group_name] = {menu_text, jnt};
      group_menu_handles_[key] = marker_menus_[group_name].insert( sub_menu_handle, jnt, boost::bind( &InteractiveControls::jointMaskCallback, this, _1 ) );
      if(mask.mask[idx]) {
        marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
      } else {
        marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );       
      }
      idx++;
    }
  }
}

void InteractiveControls::setupToolOffsetMenu(const std::string& group_name)
{
  std::string menu_text = "Tool Offset";
  MenuHandleKey set_key, clear_key;
  set_key[group_name] = {menu_text, "Set Offset"};
  clear_key[group_name] = {menu_text, "Clear Offset"};
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = marker_menus_[group_name].insert( menu_text );
  group_menu_handles_[set_key] = marker_menus_[group_name].insert( sub_menu_handle, "Set Offset", boost::bind( &InteractiveControls::toolOffsetCallback, this, _1 ) );
  group_menu_handles_[clear_key] = marker_menus_[group_name].insert( sub_menu_handle, "Clear Offset", boost::bind( &InteractiveControls::toolOffsetCallback, this, _1 ) );
}

void InteractiveControls::setupPositionToleranceMenu(const std::string& group_name) {
  std::vector<std::string> types;
  if(planner_->toleranceUtil->getToleranceTypes("position", types)) {
    std::string menu_text = "Position Tolerance";
    std::string def_tol;
    planner_->toleranceUtil->getDefaultToleranceType("position", def_tol);
    interactive_markers::MenuHandler::EntryHandle sub_menu_handle = marker_menus_[group_name].insert( menu_text );
    for(auto &t: types) {
      MenuHandleKey key;
      key[group_name] = {menu_text, t};
      group_menu_handles_[key] = marker_menus_[group_name].insert( sub_menu_handle, t, boost::bind( &InteractiveControls::positionToleranceCallback, this, _1 ) );
      if(t==def_tol) {
        marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
      } else {
        marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );       
      }
    }
  }
  marker_menus_[group_name].apply( *server_, group_name );
  server_->applyChanges();
}

void InteractiveControls::setupOrientationToleranceMenu(const std::string& group_name) {
  std::vector<std::string> types;
  if(planner_->toleranceUtil->getToleranceTypes("orientation", types)) {
    std::string menu_text = "Orientation Tolerance";
    std::string def_tol;
    planner_->toleranceUtil->getDefaultToleranceType("orientation",def_tol);
    interactive_markers::MenuHandler::EntryHandle sub_menu_handle = marker_menus_[group_name].insert( menu_text );
    for(auto &t: types) {
      MenuHandleKey key;
      key[group_name] = {menu_text, t};
      group_menu_handles_[key] = marker_menus_[group_name].insert( sub_menu_handle, t, boost::bind( &InteractiveControls::orientationToleranceCallback, this, _1 ) );
      if(t==def_tol) {
        marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
      } else {
        marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );       
      }
    }
  }
  marker_menus_[group_name].apply( *server_, group_name );
  server_->applyChanges();
}

void InteractiveControls::setupSimpleMenuItem(const std::string& group_name, const std::string& menu_text, bool has_check_box)
{
  MenuHandleKey key;
  key[group_name] = {menu_text};
  group_menu_handles_[key] = marker_menus_[group_name].insert( menu_text, boost::bind( &InteractiveControls::processFeedback, this, _1 ) );
  if(has_check_box) {
    marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
  }
}

bool InteractiveControls::setJointMaskMenuCheckState(std::string group_name, std::string jnt, bool val) 
{
  if(!isGroup(group_name)) {
    ROS_ERROR("InteractiveControls::setJointMaskMenuCheckState(%s) -- can't find group", group_name.c_str());
    return false;
  }

  MenuHandleKey key;
  key[group_name] = {"Joint Mask", jnt};
  if(val) {
    marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  } else {
    marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
  }
  marker_menus_[group_name].apply( *server_, group_name );
  server_->applyChanges();

  return true;

}

bool InteractiveControls::setSubMenuCheckState(std::string group_name, std::string menu_text, bool val) {

  if(!isGroup(group_name)) {
    ROS_ERROR("InteractiveControls::setSubMenuCheckState(%s) -- can't find group", group_name.c_str());
    return false;
  }
  auto group_search = marker_menus_.find(group_name);
  if(group_search == marker_menus_.end()) {
    ROS_ERROR("InteractiveControls::setSubMenuCheckState(%s) -- can't find group in marker menus", group_name.c_str());
    return false;
  }

  MenuHandleKey key;
  key[group_name] = {menu_text};
  int h = group_menu_handles_[key];
  if(val) {
    marker_menus_[group_name].setCheckState( h, interactive_markers::MenuHandler::CHECKED );
  } else {
    marker_menus_[group_name].setCheckState( h, interactive_markers::MenuHandler::UNCHECKED );          
  }
  marker_menus_[group_name].apply( *server_, group_name );
  server_->applyChanges();

  return true;

}

void InteractiveControls::setCartesianEEColor(std::string group_name, std_msgs::ColorRGBA c, float scale_f=1.0) {
  if(!group_type_map_[group_name] == CARTESIAN) {
    ROS_WARN("InteractiveControls::setCartesianEEColor() -- %s not a CARTESIAN group, can't set EE color", group_name.c_str());
    return;
  }
  visualization_msgs::InteractiveMarker im;
  visualization_msgs::MarkerArray ee_markers;
  server_->get(group_name, im);
  im.controls[im.controls.size()-1].markers.clear();
  ee_helper_map_[group_name]->getCurrentPositionMarkerArray(ee_markers, planner_->getControlFrame(group_name));
  std::string tf_frame_name = getEEFrameName(group_name);
  for(auto &m: ee_markers.markers) {
    m.color = c;
    m.scale = scaleVector3Msg(m.scale, scale_f);
    m.header.frame_id = tf_frame_name;
    im.controls[im.controls.size()-1].markers.push_back(m);
  }
  markers_[group_name] = im;
  server_->insert(im);
  server_->applyChanges();

}

void InteractiveControls::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) 
{

  double sleep_seconds = 3.0;
  double sleep_inc = 100.0;

  std::string group_name = feedback->marker_name;
  geometry_msgs::PoseStamped pt, pt_ee;
  geometry_msgs::Pose tool_offset;
  pt.header = feedback->header;
  pt.pose = feedback->pose;

  visualization_msgs::InteractiveMarker im;
  visualization_msgs::MarkerArray ee_markers;
  std::string tf_frame_name = getEEFrameName(group_name);
  std::string tf_tool_frame_name = getToolFrameName(group_name);

  interactive_markers::MenuHandler::CheckState state;

  // set up key maps for easy comparison to menu handler ID
  MenuHandleKey plan_key;
  MenuHandleKey execute_key;
  MenuHandleKey auto_execute_key;
  MenuHandleKey auto_plan_key;
  MenuHandleKey sync_key;
  MenuHandleKey show_path_key;
  MenuHandleKey plan_cartesian_key;
  MenuHandleKey play_plan_key;
  MenuHandleKey loop_key;
  MenuHandleKey autoplay_key;
  
  plan_key[group_name]           = {"Plan"};
  plan_cartesian_key[group_name] = {"Use Cartesian Plans"};
  execute_key[group_name]        = {"Execute"};
  auto_execute_key[group_name]   = {"Execute On Plan"};
  auto_plan_key[group_name]      = {"Plan On Move"};
  sync_key[group_name]           = {"Sync To Actual"};
  show_path_key[group_name]      = {"Show Path"};
  play_plan_key[group_name]      = {"(Re)Play Plan"};
  loop_key[group_name]           = {"Loop Animation"};
  autoplay_key[group_name]       = {"Autoplay"};

  if(group_type_map_[group_name] == CARTESIAN) {

    planner_->getToolOffset(group_name, tool_offset);
    tool_offsets_[group_name] = tool_offset;

    KDL::Frame T, Toffset;
    tf::poseMsgToKDL(pt.pose, T);
    tf::poseMsgToKDL(tool_offsets_[group_name], Toffset);
    tf::poseKDLToMsg(T*Toffset.Inverse(), pt_ee.pose);

    pt_ee.header.frame_id = pt.header.frame_id;

    frame_store_[tf_frame_name].second = pt_ee;  
  }

  switch ( feedback->event_type ) {

    ROS_INFO("InteractiveControls::processFeedback() -- %s", feedback->marker_name.c_str());

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP :      
      ROS_DEBUG("InteractiveControls::processFeedback() --   POSE UPDATE");
      ROS_DEBUG("InteractiveControls::processFeedback() --   pose: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f), frame_id: %s", 
                                                                  feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z, 
                                                                  feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w,
                                                                  feedback->header.frame_id.c_str());    

      if(auto_plan_[group_name]) {
        if(!planToGoal(group_name, pt)) {
          ROS_ERROR("InteractiveControls::processFeedback() -- failed to plan to goal");
        } else {
          if(group_type_map_[group_name] == CARTESIAN && auto_execute_[group_name]) {
            resetCartesianGroupMarker(feedback->marker_name);
          }
        }
      } else {
        if(group_type_map_[group_name] == CARTESIAN) {
          planner_interface::ReachableType reach = planner_->isReachable(group_name, pt, group_position_tolerances_[group_name], group_orientation_tolerances_[group_name]);
          if (reach == planner_interface::EXACTLY_REACHABLE) {
            setCartesianEEColor(group_name, getColorMsg(1.0,0.0,1.0,0.8), 0.95);
          } else if (reach == planner_interface::REACHABLE_WITH_TOLERANCES) {
            setCartesianEEColor(group_name, getColorMsg(0.0,1.0,1.0,0.8), 0.95);
          }
          else {
            setCartesianEEColor(group_name, getColorMsg(1.0,0.0,0.0,0.8), 0.95);
          }
        }
      }
      break;
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT :
      ROS_DEBUG("InteractiveControls::processFeedback() --   MENU_SELECT");
      ROS_DEBUG("InteractiveControls::processFeedback() --   menu id: %d", feedback->menu_entry_id);
      ROS_DEBUG("InteractiveControls::processFeedback() --   pose: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f), frame_id: %s", 
                                                                  feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z, 
                                                                  feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w,
                                                                  feedback->header.frame_id.c_str());

      if(group_menu_handles_.find(plan_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[plan_key] == feedback->menu_entry_id) {
          if(!planToGoal(group_name, pt)) {
            ROS_ERROR("InteractiveControls::processFeedback() -- failed to plan to goal");
          } else {
            if(group_type_map_[group_name] == CARTESIAN && auto_execute_[group_name]) {
              resetCartesianGroupMarker(feedback->marker_name);
            }
          }
        }
      }
      if(group_menu_handles_.find(execute_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[execute_key] == feedback->menu_entry_id) {
          ROS_DEBUG("InteractiveControls::processFeedback() --   EXECUTE");
          std::vector<std::string> groups;
          groups.push_back(group_name);
          planner_->executePlans(groups); 

          if(group_type_map_[group_name] == CARTESIAN) {
            resetCartesianGroupMarker(feedback->marker_name);
          } else {
            // I hate this dumb hack, but its the only way afaik to get the IM meshes to move with the links 
            for(int i=0; i<(int)sleep_inc; i++) {
              marker_menus_[feedback->marker_name].reApply( *server_ );
              server_->applyChanges();
              ros::Duration(sleep_seconds/sleep_inc).sleep();
            }
          }
        }
      }

      if(group_menu_handles_.find(plan_cartesian_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[plan_cartesian_key] == feedback->menu_entry_id) {
          ROS_DEBUG("InteractiveControls::processFeedback() --   PLAN CARTESIAN PATHS");
          if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state ) ) {
            if(state == interactive_markers::MenuHandler::CHECKED) {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED );
              plan_cartesian_paths_[feedback->marker_name] = false;
            } else {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED );
              plan_cartesian_paths_[feedback->marker_name] = true;
            }
          }
        }
      }

      if(group_menu_handles_.find(auto_execute_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[auto_execute_key] == feedback->menu_entry_id) {
          ROS_DEBUG("InteractiveControls::processFeedback() --   AUTO EXECUTE");
          if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state ) ) {
            if(state == interactive_markers::MenuHandler::CHECKED) {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED );
              auto_execute_[feedback->marker_name] = false;
            } else {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED );
              auto_execute_[feedback->marker_name] = true;
            }
          }
        }
      }

      if(group_menu_handles_.find(auto_plan_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[auto_plan_key] == feedback->menu_entry_id) {
          ROS_DEBUG("InteractiveControls::processFeedback() --   AUTO PLAN");
          if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state ) ) {
            if(state == interactive_markers::MenuHandler::CHECKED) {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED );
              auto_plan_[feedback->marker_name] = false;
            } else {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED );
              auto_plan_[feedback->marker_name] = true;
            }
          }
        }
      }

      if(group_menu_handles_.find(show_path_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[show_path_key] == feedback->menu_entry_id) {
          ROS_DEBUG("InteractiveControls::processFeedback() --   SHOW PATH");
          if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state ) ) {
            if(state == interactive_markers::MenuHandler::CHECKED) {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED );
              show_path_[feedback->marker_name] = false;
            } else {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED );
              show_path_[feedback->marker_name] = true;
            }
          }
        }
      }

      if(group_menu_handles_.find(sync_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[sync_key] == feedback->menu_entry_id) {
          ROS_DEBUG("InteractiveControls::processFeedback() --   SYNC");
          resetCartesianGroupMarker(feedback->marker_name);
        }
      }

      if (group_menu_handles_.find(play_plan_key) != std::end(group_menu_handles_)){
        if (group_menu_handles_[play_plan_key] == feedback->menu_entry_id) {
          ROS_INFO("[InteractiveControls::processFeedback] playing available plan");
          planner_->playAnimation();
        }
      }

      if (group_menu_handles_.find(loop_key) != std::end(group_menu_handles_)) {
        if (group_menu_handles_[loop_key] == feedback->menu_entry_id) {
          ROS_INFO("[InteractiveControls::processFeedback] changing looping functionality");

          MenuHandleKey key;
          key[feedback->marker_name] = {"Loop Animation"};

          bool loop = false;
          if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state)) {
            if(state == interactive_markers::MenuHandler::CHECKED) {
              marker_menus_[feedback->marker_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
            } else {
              marker_menus_[feedback->marker_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
              loop = true; // transitioning from not looping to looping
            }
            planner_->loopAnimation(feedback->marker_name, loop);
          }
        }
      }

      if (group_menu_handles_.find(autoplay_key) != std::end(group_menu_handles_)) {
        if (group_menu_handles_[autoplay_key] == feedback->menu_entry_id) {
          MenuHandleKey key;
          key[group_name] = {"Autoplay"};

          bool set;
          if(marker_menus_[group_name].getCheckState( feedback->menu_entry_id, state)) {
            if(state == interactive_markers::MenuHandler::CHECKED) {
              ROS_INFO("[InteractiveControls::processFeedback] flipping autoplay functionality for group %s to FALSE", group_name.c_str());
              marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
              auto_display_ = false;
            } else {
              ROS_INFO("[InteractiveControls::processFeedback] flipping autoplay functionality for group %s to TRUE", group_name.c_str());
              marker_menus_[group_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
              auto_display_ = true;
            }
          } else {
            ROS_ERROR("[InteractiveControls::processFeedback] can't get the autoplay state for %s group!!", group_name.c_str());
          }

          // shared autoplay flag so reflect changes to other groups          
          for (auto a : active_groups_) {
            MenuHandleKey group_key;
            group_key[a] = {"Autoplay"};

            if (marker_menus_.find(a) != marker_menus_.end() && group_menu_handles_.find(group_key) != group_menu_handles_.end()) {
              marker_menus_[a].setCheckState( group_menu_handles_[group_key], auto_display_ ? interactive_markers::MenuHandler::CHECKED : interactive_markers::MenuHandler::UNCHECKED);
              marker_menus_[a].reApply( *server_ );
            }
          }
        }
      }
      break;
  }

  marker_menus_[feedback->marker_name].reApply( *server_ );
  server_->applyChanges();

}

void InteractiveControls::storedPoseCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
    std::string group_name = feedback->marker_name;
    std::string menu_text = "Stored Poses";
    for(auto &p: stored_pose_map_[group_name]) {
      std::string state_name = p.first;
      MenuHandleKey key;
      key[group_name] = {menu_text, state_name};
      if(group_menu_handles_[key] == feedback->menu_entry_id) {       
        bool execute = auto_execute_[group_name] || (group_type_map_[group_name]==ENDEFFECTOR);
        doStoredPose(group_name, state_name, execute);
      }
    }
  }

  server_->applyChanges();
}


void InteractiveControls::jointMaskCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) {

  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
    std::string group_name = feedback->marker_name;
    std::string menu_text = "Joint Mask";
    robot_interaction_tools_msgs::JointMask mask;
    interactive_markers::MenuHandler::CheckState state;
    planner_->getJointMask(group_name, mask);
    int idx = 0;
    for(auto &jnt: mask.joint_names) {
      MenuHandleKey key;
      key[group_name] = {menu_text, jnt};
      if(group_menu_handles_[key] == feedback->menu_entry_id) {      
        if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state ) ) {
          if(state == interactive_markers::MenuHandler::CHECKED) {
            ROS_INFO("InteractiveControls::jointMaskCallback() -- %s, masking: %s", group_name.c_str(), jnt.c_str());    
            marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED );
            mask.mask[idx] = false;
          } else {
            ROS_INFO("InteractiveControls::jointMaskCallback() -- %s, unmasking: %s", group_name.c_str(), jnt.c_str());    
            marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED );
            mask.mask[idx] = true;
          }
        }
      }
      idx++;
    }
    planner_->setJointMask(group_name, mask);
  }
  marker_menus_[feedback->marker_name].reApply( *server_ );
  server_->applyChanges();
}

void InteractiveControls::positionToleranceCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) {
  ROS_INFO("InteractiveControls::positionToleranceCallback()");
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
    std::string group_name = feedback->marker_name;
    std::string new_type;
    std::vector<std::string> types;
    tolerance_util::ToleranceVal tol_val;
    if(planner_->toleranceUtil->getToleranceTypes("position", types)) {
      std::string menu_text = "Position Tolerance";
      for(auto &t: types) {
        MenuHandleKey key;
        key[group_name] = {menu_text, t};
        if(group_menu_handles_[key] == feedback->menu_entry_id) {
          new_type = t;      
          ROS_INFO("InteractiveControls::positionToleranceCallback() -- setting %s", t.c_str());
          marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED );
        } else {
          marker_menus_[feedback->marker_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );          
        }
      }
      planner_->toleranceUtil->getToleranceVal("position", new_type, tol_val);
      group_position_tolerance_type_[group_name] = new_type;
      group_position_tolerances_[group_name][0] = tol_val[0];
      group_position_tolerances_[group_name][1] = tol_val[1];
      group_position_tolerances_[group_name][2] = tol_val[2];

      ROS_INFO("InteractiveControls::positionToleranceCallback() -- vals: %.4f, %.4f, %.4f", tol_val[0],tol_val[1],tol_val[2]);
      //      planner_->setPositionTolerances(group_name, group_position_tolerances_[group_name]);
    }
  }
  marker_menus_[feedback->marker_name].reApply( *server_ );
  server_->applyChanges();
}

void InteractiveControls::orientationToleranceCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) {
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
    std::string group_name = feedback->marker_name;
    std::string new_type;
    std::vector<std::string> types;
    tolerance_util::ToleranceVal tol_val;
    if(planner_->toleranceUtil->getToleranceTypes("orientation", types)) {
      std::string menu_text = "Orientation Tolerance";
      interactive_markers::MenuHandler::CheckState state;
      int idx = 0;
      for(auto &t: types) {
        MenuHandleKey key;
        key[group_name] = {menu_text, t};
        if(group_menu_handles_[key] == feedback->menu_entry_id) {
          new_type = t;      
          marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED );
        } else {
          marker_menus_[feedback->marker_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );          
        }
        idx++;
      }
      group_orientation_tolerance_type_[group_name] = new_type;
      planner_->toleranceUtil->getToleranceVal("orientation", new_type, tol_val);
      group_orientation_tolerances_[group_name][0] = tol_val[0];
      group_orientation_tolerances_[group_name][1] = tol_val[1];
      group_orientation_tolerances_[group_name][2] = tol_val[2];;
      //      planner_->setOrientationTolerances(group_name, group_orientation_tolerances_[group_name]);
    }
  }
  marker_menus_[feedback->marker_name].reApply( *server_ );
  server_->applyChanges();

}

void InteractiveControls::toolOffsetCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::string group_name = feedback->marker_name;
  std::string tf_frame_name = getEEFrameName(group_name);
  std::string tf_tool_frame_name = getToolFrameName(group_name);
  std::string menu_text = "Tool Offset";
  MenuHandleKey set_key, clear_key;
  set_key[group_name] = {menu_text, "Set Offset"};
  clear_key[group_name] = {menu_text, "Clear Offset"}; 
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
    if(group_menu_handles_[set_key] == feedback->menu_entry_id) {
      planner_->setToolOffset(feedback->marker_name, feedback->pose);
            
      geometry_msgs::PoseStamped pt_ee, pt_tool;
      pt_ee.header.frame_id = feedback->header.frame_id;        
      pt_ee.pose.orientation.w = 1.0;
      frame_store_[tf_frame_name].second = pt_ee;  

      pt_tool.header.frame_id = tf_frame_name;
      pt_tool.pose = feedback->pose;
      frame_store_[tf_tool_frame_name].second = pt_tool;  

    } else if(group_menu_handles_[clear_key] == feedback->menu_entry_id) {
      planner_->clearToolOffset(feedback->marker_name);

      geometry_msgs::PoseStamped pt_ee, pt_tool;
      pt_ee.header.frame_id = feedback->header.frame_id;        
      pt_ee.pose = feedback->pose;
      frame_store_[tf_frame_name].second = pt_ee;  

      pt_tool.header.frame_id = tf_frame_name;
      pt_tool.pose.orientation.w = 1.0;
      frame_store_[tf_tool_frame_name].second = pt_tool;  

    }
  }
  resetCartesianGroupMarker(feedback->marker_name);
}
      
bool InteractiveControls::doStoredPose(std::string group_name, std::string pose_name, bool execute) 
{
  double sleep_seconds = 3.0;
  double sleep_inc = 100.0;

  planner_->resetAnimation();

  if(isStoredState(group_name, pose_name)) {
    std::map<std::string, std::vector<sensor_msgs::JointState> > goals;
    goals[group_name].push_back(stored_pose_map_[group_name][pose_name]);
    planner_->planJointPath(goals, execute, show_path_[group_name]); 

    // I hate this dumb hack, but its the only way afaik to get the IM meshes to move with the links 
    if(execute) {
      for(int i=0; i<(int)sleep_inc; i++) {
        marker_menus_[group_name].reApply( *server_ );
        server_->applyChanges();
        ros::Duration(sleep_seconds/sleep_inc).sleep();
      }
    }
    return true;
  }
  return false;
}

bool InteractiveControls::planToMarker(std::string group_name)
{
  geometry_msgs::PoseStamped pt;
  visualization_msgs::InteractiveMarker im;
  server_->get(group_name, im);
  pt.header = im.header;
  pt.pose = im.pose; 
  if(!planToGoal(group_name, pt)) {
    ROS_ERROR("[InteractiveControls::planToMarker] -- failed to plan");
    return false;
  }
  if(group_type_map_[group_name] == CARTESIAN && auto_execute_[group_name]) {
    resetCartesianGroupMarker(group_name);
  }
  return true;
}
      

std::string InteractiveControls::getTypeString(const GroupType &group_type)
{
  if(group_type==CARTESIAN) {
    return "cartesian";
  } else if(group_type==JOINT) {
    return "joint";
  } else if(group_type==ENDEFFECTOR) {
    return "endeffector";
  } else {
    ROS_ERROR("InteractiveControls::getTypeString() -- can't parse group type!!");
    return "";
  }
}

InteractiveControls::GroupType InteractiveControls::getTypeFromString(const std::string &type_string)
{
  if(type_string=="cartesian") {
    return CARTESIAN;
  } else if(type_string=="joint") {
    return JOINT;
  } else if(type_string=="endeffector") {
    return ENDEFFECTOR;
  } else {
    ROS_ERROR("InteractiveControls::getTypeFromString() -- can't parse group string!!");
    return JOINT;
  }
}

bool InteractiveControls::resetCartesianGroupMarker(const std::string& group_name, double delay) 
{

  if(!isKnownGroupMarker(group_name)) {
    ROS_ERROR("InteractiveControls::resetCartesianGroupMarker() -- unkown group: %s", group_name.c_str());
    return false;
  }

  if(!isInTypeMap(group_name)) {
    ROS_ERROR("InteractiveControls::resetCartesianGroupMarker() -- unknown type for group: %s", group_name.c_str());
    return false;
  }

  ros::Duration(delay).sleep();

  if(group_type_map_[group_name] == CARTESIAN) {
    // create a default pose offset wrt frame of marker. this clears any tool offsets
    visualization_msgs::InteractiveMarker im;
    geometry_msgs::Pose offset;
    geometry_msgs::PoseStamped pt;
    pt.header.frame_id = planner_->getControlFrame(group_name);
    pt.pose.orientation.w = 1.0;
    planner_->getToolOffset(group_name, offset);
    server_->setPose(markers_[group_name].name, offset);
    std::string tf_frame_name = getEEFrameName(group_name);
    frame_store_[tf_frame_name] = FrameInfo(tf_frame_name, pt);

    server_->get(group_name, im);
    im.controls[im.controls.size()-1].markers.clear();
    markers_[group_name] = im;
    server_->insert(im);
    server_->applyChanges();

  } else {
    ROS_ERROR("InteractiveControls::resetCartesianGroupMarker() -- group[%s] not a CARTESIAN group!!", group_name.c_str());
    return false;
  }
  server_->applyChanges();

  return true;

}

bool InteractiveControls::removeGroupMarkers(const std::string& group_name) 
{

  if(!isKnownGroupMarker(group_name)) {
    ROS_ERROR("InteractiveControls::removeGroupMarkers() -- unknown group: %s", group_name.c_str());
    return false;
  }

  active_groups_.erase(std::remove(active_groups_.begin(), active_groups_.end(), group_name), active_groups_.end());
  server_->erase(markers_[group_name].name);
  markers_.erase(group_name);
  marker_menus_.erase(group_name);
  planner_->removeGroup(group_name);
  server_->applyChanges();

  return true;
}


bool InteractiveControls::isKnownGroupMarker(const std::string &group_name)
{
  return (markers_.find(group_name) != std::end(markers_)); 
}


bool InteractiveControls::isInTypeMap(const std::string &group_name)
{
  return (group_type_map_.find(group_name) != std::end(group_type_map_)); 
}


bool InteractiveControls::isToleranceMode(const std::string &mode) 
{
  return (std::find(std::begin(tolerance_modes_), std::end(tolerance_modes_), mode)!=tolerance_modes_.end());
}

bool InteractiveControls::isGroup(const std::string &group_name) 
{
  std::vector<std::string> groups = planner_->getGroups();
  return (std::find(std::begin(groups), std::end(groups), group_name)!=groups.end());
}

bool InteractiveControls::isActiveGroup(const std::string &group_name) 
{
  return (std::find(std::begin(active_groups_), std::end(active_groups_), group_name)!=active_groups_.end());
}

bool InteractiveControls::isStoredState(const std::string &group_name, const std::string &state_name) 
{
  auto group_search = stored_pose_map_.find(group_name);
  if(group_search != stored_pose_map_.end()) {
     auto pose_search = stored_pose_map_[group_name].find(state_name);
     return (pose_search != stored_pose_map_[group_name].end());
  } else {
    return false;
  }
}


void InteractiveControls::spin()
{

  ros::Rate loop_rate(loop_rate_);
  tf::Transform transform;
  FrameInfo fi;

  ros::AsyncSpinner spinner(1.0/loop_rate_);
  spinner.start();
  ROS_INFO("%s started.", nh_.getNamespace().c_str());


  while(ros::ok())
  {
    ros::Time t = ros::Time::now();
    for(auto &f: frame_store_) 
    {
      fi = f.second;
      tf::poseMsgToTF(fi.second.pose, transform);
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, t, fi.second.header.frame_id, fi.first));
      server_->applyChanges();
    }
    server_->applyChanges();
    loop_rate.sleep();
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_interaction_tools");
  ros::NodeHandle nh("~");
  std::string robot_name, plugin_planner;
  nh.getParam("robot_name", robot_name); 
  nh.getParam("planner_plugin", plugin_planner);
  if(plugin_planner=="") {
    plugin_planner="moveit_planner::MoveItPlanner";
  } 
  InteractiveControls controls(nh, robot_name, plugin_planner);

  controls.spin();
  return 0;
}







