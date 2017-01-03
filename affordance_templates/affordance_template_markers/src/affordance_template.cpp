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

#include <affordance_template_markers/affordance_template.h>

using namespace affordance_template;
using namespace affordance_template_object;
using namespace affordance_template_markers;
using namespace affordance_template_msgs;

AffordanceTemplate::AffordanceTemplate(const ros::NodeHandle nh, 
                                        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,  
                                        std::string robot_name, 
                                        std::string template_type,
                                        int id) :
  nh_(nh),
  server_(server),
  robot_name_(robot_name),
  template_type_(template_type),
  id_(id),
  root_object_(""),
  loop_rate_(25.0),
  object_controls_display_on_(true),
  planning_server_(nh, (template_type + "_" + std::to_string(id) + "/plan_action"), boost::bind(&AffordanceTemplate::planRequest, this, _1), false),
  execution_server_(nh, (template_type + "_" + std::to_string(id) + "/execute_action"), boost::bind(&AffordanceTemplate::executeRequest, this, _1), false)
{
  ROS_DEBUG("AffordanceTemplate::init() -- creating new AffordanceTemplate of type %s for robot: %s", template_type_.c_str(), robot_name_.c_str());
  name_ = template_type_ + ":" + std::to_string(id);

  setupMenuOptions();

  planning_server_.start();
  execution_server_.start();
 
  updateThread_.reset(new boost::thread(boost::bind(&AffordanceTemplate::run, this)));

  // set to false when template gets destroyed, otherwise we can get a dangling pointer
  running_ = true;
  autoplay_display_ = true;
}


AffordanceTemplate::AffordanceTemplate(const ros::NodeHandle nh, 
                                        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,  
                                        boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface,
                                        std::string robot_name, 
                                        std::string template_type,
                                        int id) :
  AffordanceTemplate(nh, server, robot_name, template_type, id)
{
  setRobotInterface(robot_interface);
}

AffordanceTemplate::~AffordanceTemplate() 
{
  updateThread_->join();
  robot_interface_->getPlanner()->resetAnimation(true);
}

void AffordanceTemplate::setRobotInterface(boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface)
{
  robot_interface_ = robot_interface;
}

void AffordanceTemplate::setupMenuOptions() 
{
  waypoint_menu_options_.clear();
  waypoint_menu_options_.push_back(MenuConfig("Change End-Effector Pose", false));
  waypoint_menu_options_.push_back(MenuConfig("Hide Controls", true));
  waypoint_menu_options_.push_back(MenuConfig("Compact View", true));
  waypoint_menu_options_.push_back(MenuConfig("Add Waypoint Before", false));
  waypoint_menu_options_.push_back(MenuConfig("Add Waypoint After", false));
  waypoint_menu_options_.push_back(MenuConfig("Delete Waypoint", false));
  waypoint_menu_options_.push_back(MenuConfig("Move Forward", false));
  waypoint_menu_options_.push_back(MenuConfig("Move Back", false));
  waypoint_menu_options_.push_back(MenuConfig("Change Tool Offset", true));
  waypoint_menu_options_.push_back(MenuConfig("Move Tool Point", true));
  
  object_menu_options_.clear();
  object_menu_options_.push_back(MenuConfig("Add Waypoint Before", false));
  object_menu_options_.push_back(MenuConfig("Add Waypoint After", false));
  object_menu_options_.push_back(MenuConfig("Reset", false));
  object_menu_options_.push_back(MenuConfig("Save", false));
  object_menu_options_.push_back(MenuConfig("Hide Controls", true));
  object_menu_options_.push_back(MenuConfig("Choose Trajectory", false));
  object_menu_options_.push_back(MenuConfig("(Re)Play Plan", false));
  object_menu_options_.push_back(MenuConfig("Loop Animation", true));
  object_menu_options_.push_back(MenuConfig("Autoplay", true));

  toolpoint_menu_options_.clear();
  toolpoint_menu_options_.push_back(MenuConfig("Change Tool Offset", true));
  toolpoint_menu_options_.push_back(MenuConfig("Move Tool Point", true));
}

void AffordanceTemplate::setObjectMenuDefaults(std::string obj_name) {
  MenuHandleKey key;
  key[obj_name] = {"Hide Controls"};
  if(object_controls_display_on_) {
    marker_menus_[obj_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
  } else {
    marker_menus_[obj_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  }

  key[obj_name] = {"Autoplay"};
  if(autoplay_display_) {
    marker_menus_[obj_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  } else {
    marker_menus_[obj_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
  }
}

void AffordanceTemplate::setWaypointMenuDefaults(std::string wp_name) {
  MenuHandleKey key;
  if(waypoint_flags_[current_trajectory_].run_backwards) {
    key[wp_name] = {"Compute Backwards Path"};
    marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  }  
  if(waypoint_flags_[current_trajectory_].auto_execute) {
    key[wp_name] = {"Execute On Move"};
    marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  }  
  if(waypoint_flags_[current_trajectory_].loop) {
    key[wp_name] = {"Loop Path"};
    marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  }  
  if(waypoint_flags_[current_trajectory_].controls_on[wp_name]) {
    key[wp_name] = {"Hide Controls"};
    marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
  } else {
    key[wp_name] = {"Hide Controls"};
    marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  }
  if(waypoint_flags_[current_trajectory_].compact_view[wp_name]) {
    key[wp_name] = {"Compact View"};
    marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  } else {
    key[wp_name] = {"Compact View"};
    marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
  }
  if(waypoint_flags_[current_trajectory_].adjust_offset[wp_name]) {
    key[wp_name] = {"Change Tool Offset"};
    marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  } else {
    key[wp_name] = {"Change Tool Offset"};
    marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
  }
  if(waypoint_flags_[current_trajectory_].move_offset[wp_name]) {
    key[wp_name] = {"Move Tool Point"};
    marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  } else {
    key[wp_name] = {"Move Tool Point"};
    marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
  }
}

void AffordanceTemplate::setToolPointMenuDefaults(std::string tp_name) {
  MenuHandleKey key;
  std::string wp_name = getWaypointFrame(tp_name);
  if(waypoint_flags_[current_trajectory_].adjust_offset[wp_name]) {
    key[tp_name] = {"Change Tool Offset"};
    marker_menus_[tp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  } else {
    key[tp_name] = {"Change Tool Offset"};
    marker_menus_[tp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
  }
  if(waypoint_flags_[current_trajectory_].move_offset[wp_name]) {
    key[tp_name] = {"Move Tool Point"};
    marker_menus_[tp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
  } else {
    key[tp_name] = {"Move Tool Point"};
    marker_menus_[tp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
  }
}

bool AffordanceTemplate::loadFromFile(std::string filename, geometry_msgs::Pose pose, AffordanceTemplateStructure &structure)
{
  at_parser_.loadFromFile(filename, structure);

  // store copies in class, one as backup to reset/restore later
  initial_structure_ = structure;
  structure_ = structure;

  bool append = appendIDToStructure(structure_);
  bool created = buildTemplate();

  return (append && created);
}

bool AffordanceTemplate::saveToDisk(std::string& filename, const std::string& image, const std::string& key, bool save_scale_updates)
{
  std::string class_type = template_type_;
  if (filename.empty())
    filename = template_type_ + ".json";
  else
  {
    std::vector<std::string> keys;
    boost::split(keys, filename, boost::is_any_of("."));
    if (keys.size())
      class_type = keys.front();
  }

  std::string root = ros::package::getPath("affordance_template_library");
  if (root.empty())
    return false;
  root += "/templates";
  std::string output_path = root + "/" + filename;

  // if filename given is actually directory, return
  if (boost::filesystem::is_directory(output_path))
  {
    ROS_ERROR("[AffordanceTemplate::saveToDisk] error formatting filename!!");
    return false;
  }

  ROS_INFO("[AffordanceTemplate::saveToDisk] writing template to file: %s", output_path.c_str());

  if (!boost::filesystem::exists(output_path)) {
    ROS_WARN("[AffordanceTemplate::saveToDisk] no file found with name: %s. cannot create backup.", filename.c_str());
  } else {
    // count how many bak files we have for this particular template type
    boost::filesystem::recursive_directory_iterator dir_it(root);
    boost::filesystem::recursive_directory_iterator end_it;
    int bak_counter = 0;
    while (dir_it != end_it) {
      if (std::string(dir_it->path().string()).find(".bak") != std::string::npos
          && std::string(dir_it->path().string()).find(class_type) != std::string::npos)
        ++bak_counter;
      ++dir_it;
    }

    // copy current class_type.json into .bak
    std::string bak_path = "";
    if (bak_counter > 0) {
      ROS_INFO("[AffordanceTemplate::saveToDisk] creating backup file: %s.bak%d", filename.c_str(), bak_counter);
      bak_path = output_path + ".bak" + std::to_string(bak_counter);
    } else {
      ROS_INFO("[AffordanceTemplate::saveToDisk] creating backup file: %s.bak", filename.c_str());
      bak_path = output_path + ".bak";
    }
    boost::filesystem::copy_file(output_path, bak_path, boost::filesystem::copy_option::overwrite_if_exists);
  }

  // set structure key as the new key name
  std::vector<std::string> keys;
  boost::split(keys, key, boost::is_any_of(":"));
  if (keys.size())
    structure_.name = keys.front();

  return at_parser_.saveToFile(output_path, structure_);
}

bool AffordanceTemplate::appendIDToStructure(AffordanceTemplateStructure &structure) 
{
  structure.name = appendID(structure.name);
  for(auto &obj : structure.display_objects) {
      obj.name = appendID(obj.name);
      if(obj.parent != "") {
        obj.parent = appendID(obj.parent);
      }
  }
  return true;
}
 
std::string AffordanceTemplate::createWaypointID(int ee_id, int wp_id)
{
  return std::to_string(ee_id) + "." + std::to_string(wp_id) + ":" + name_;
}

std::string AffordanceTemplate::appendID(std::string s) 
{
  return s + ":" + std::to_string(id_);
} 

std::string AffordanceTemplate::removeID(std::string s)
{
  size_t endpos = s.rfind(":");
  if( std::string::npos != endpos ) {
    return s.substr(0, endpos);
  }
}
    
int AffordanceTemplate::getWaypointIDFromName(std::string wp_name)
{
  std::size_t dot_pos = wp_name.find(".");
  std::size_t colon_pos = wp_name.find(":");
  if(dot_pos == std::string::npos || colon_pos == std::string::npos) {
    ROS_ERROR("AffordanceTemplate""getWaypointIDFromName() -- malformed wp name %s", wp_name.c_str());
    return -1;
  }
  int wp_id = stoi(wp_name.substr(dot_pos + 1,colon_pos));  
  return wp_id;
}

int AffordanceTemplate::getEEIDFromName(std::string wp_name)
{
  std::size_t dot_pos = wp_name.find(".");
  if(dot_pos == std::string::npos) {
    ROS_ERROR("AffordanceTemplate""getEEIDFromName() -- malformed wp name %s", wp_name.c_str());
    return -1;
  }
  int ee_id = stoi(wp_name.substr(0,dot_pos));  
  return ee_id;
}


bool AffordanceTemplate::addTrajectory(const std::string& trajectory_name) 
{
  affordance_template_object::Trajectory traj;
  traj.name = trajectory_name;
  structure_.ee_trajectories.push_back(traj);
  setTrajectory(trajectory_name);
  setupTrajectoryMenu(trajectory_name);
  return buildTemplate(trajectory_name);
}

bool AffordanceTemplate::getTrajectory(TrajectoryList& traj_list, std::string traj_name, Trajectory& traj) 
{
  for (auto &t: traj_list) {
    if(t.name == traj_name) {
      traj = t;
      return true;
    }
  }
  ROS_ERROR("AffordanceTemplate::getTrajectory() -- could not find %s in list of trajectories", traj_name.c_str());
  return false;
}

bool AffordanceTemplate::getTrajectoryPlan(const std::string& trajectory, const std::string& ee, PlanStatus& plan)
{
  if (plan_status_.find(trajectory) != plan_status_.end()) {
    if (plan_status_[trajectory].find(ee) != plan_status_[trajectory].end()) {
      plan = plan_status_[trajectory][ee];
    } else {
      return false;
    }
  } else {
    return false;
  }
  return true;
}

bool AffordanceTemplate::setTrajectory(const std::string& trajectory_name)
{
  return setCurrentTrajectory( getCurrentStructure().ee_trajectories, trajectory_name);
}

bool AffordanceTemplate::switchTrajectory(const std::string& trajectory_name)
{
  removeAllMarkers();
  if(setTrajectory(trajectory_name)) {
    setupTrajectoryMenu(trajectory_name);
    if(buildTemplate(trajectory_name)) {
      ROS_DEBUG("AffordanceTemplate::switchTrajectory() -- %s succeeded", trajectory_name.c_str());
    } else {
      ROS_ERROR("AffordanceTemplate::switchTrajectory() -- %s failed", trajectory_name.c_str());
      return false;
    }
  }
 return true;
}

void AffordanceTemplate::clearTrajectoryFlags()
{
  waypoint_flags_.clear();
}

bool AffordanceTemplate::getWaypointFlags(const std::string& traj, WaypointTrajectoryFlags& flags) {
  if(waypoint_flags_.find(traj)!=waypoint_flags_.end()) {
    flags = waypoint_flags_[traj];
  } else {
    ROS_ERROR("AffordanceTemplate::getWaypointFlags() -- no traj=%s found", traj.c_str());
    return false;
  }
  return true;
}
    
void AffordanceTemplate::setTrajectoryFlags(Trajectory traj) 
{  
  if(waypoint_flags_.find(traj.name) == std::end(waypoint_flags_)) {
    WaypointTrajectoryFlags wp_flags;
    wp_flags.run_backwards = false;
    wp_flags.loop = false;
    wp_flags.auto_execute = false;
    for(auto &ee : traj.ee_waypoint_list) {
      for(size_t idx=0; idx<ee.waypoints.size(); idx++) {
        std::string wp_name = createWaypointID(ee.id,idx);
        wp_flags.controls_on[wp_name] = false;
        wp_flags.compact_view[wp_name] = false;
        wp_flags.adjust_offset[wp_name] = false;
        wp_flags.move_offset[wp_name] = false;
      }
    }
    waypoint_flags_[traj.name] = wp_flags;
  }
}

bool AffordanceTemplate::isValidTrajectory(Trajectory traj)  
{
  bool valid_trajectory = true;
  for(auto &g: traj.ee_waypoint_list) {
    for(auto &wp: g.waypoints) {
      if(robot_interface_->getEENameMap().find(g.id) == std::end(robot_interface_->getEENameMap())) {
        valid_trajectory = false;
        ROS_DEBUG("AffordanceTemplate::is_valid_trajectory() -- can't find ee ID: %d", g.id);
        return valid_trajectory;
      }
   }
 }
 return valid_trajectory;
} 

// set the default (current) traj to the input one.
// if no input request, find the first valid one
bool AffordanceTemplate::setCurrentTrajectory(TrajectoryList traj_list, std::string traj) 
{
  ROS_DEBUG("AffordanceTemplate::setCurrentTrajectory() -- will attempt to set current trajectory to %s", traj.c_str());
  current_trajectory_ = "";
  if ( !traj.empty()) {
    for (auto &t: traj_list) {
      if(t.name == traj) {
        if(isValidTrajectory(t)) {
          current_trajectory_ = t.name;
          ROS_DEBUG("AffordanceTemplate::setCurrentTrajectory() -- setting current trajectory to: %s", current_trajectory_.c_str());
          break;
        } else {
          ROS_ERROR("[AffordanceTemplate::setCurrentTrajectory] -- \'%s\' not a valid trajectory", t.name.c_str());
        }
      }
    }
  } else {
    ROS_DEBUG("AffordanceTemplate::setCurrentTrajectory() -- input trajectory is empty");
  }
  // get the first valid trajectory
  if ( current_trajectory_.empty()) {
    for (auto &t: traj_list) {
      if(isValidTrajectory(t)) {
        current_trajectory_ = t.name;
        if(traj!=current_trajectory_) {
          ROS_DEBUG("AffordanceTemplate::setCurrentTrajectory() -- \'%s\' not valid, setting current trajectory to: %s", traj.c_str(), current_trajectory_.c_str());
        } else {
          ROS_DEBUG("AffordanceTemplate::setCurrentTrajectory() -- setting current trajectory to: %s", current_trajectory_.c_str());          
        }
        break;
      } 
    }
  } 
  if (current_trajectory_.empty()) // still no valid traj found
  {
    ROS_ERROR("AffordanceTemplate::createDisplayObjects() -- no valid trajectory found");
    return false;
  }
  return true;
}

bool AffordanceTemplate::setTemplatePose(geometry_msgs::PoseStamped ps)
{
  key_ = structure_.name;
  frame_store_[key_] = FrameInfo(key_, ps);
  return true;
}
    
bool AffordanceTemplate::setWaypointViewMode(int ee, int wp, bool m)
{
  std::string wp_name = createWaypointID(ee, wp);
  waypoint_flags_[current_trajectory_].compact_view[wp_name] = m;
  ROS_DEBUG("AffordanceTemplate::setWaypointViewMode() -- setting compact_view for [%s] to %d", wp_name.c_str(), (int)m);
  if(!removeMarkerAndRebuild(wp_name)) {
    ROS_ERROR("AffordanceTemplate::setWaypointViewMode() -- error building template with wp [%s] view mode %d", wp_name.c_str(), (int)m);
    return false;
  }
  return true;
}

bool AffordanceTemplate::buildTemplate(std::string traj) 
{
  ROS_DEBUG("AffordanceTemplate::buildTemplate() -- %s", template_type_.c_str());

  // set trajectoy to current if empty
  if(traj.empty()) {
    traj = current_trajectory_;
  }
  // set the trajectory, and build the AT interactive markers and data structures
  if(setCurrentTrajectory(structure_.ee_trajectories, traj)) {
    if(!createDisplayObjects()) {
      ROS_ERROR("AffordanceTemplate::buildTemplate() -- couldn't createDisplayObjects()");
      return false;
    }
    if(!createWaypoints()) {
      ROS_ERROR("AffordanceTemplate::buildTemplate() -- couldn't createWaypoints()"); 
      return false;
    }
  } else {
    ROS_ERROR("AffordanceTemplate::buildTemplate() -- couldn't set the current trajectory");
    return false;
  }
  ROS_DEBUG("AffordanceTemplate::buildTemplate() -- done creating %s", template_type_.c_str());  
  return true;
}

 
bool AffordanceTemplate::createDisplayObjects() {
  // get AT name (<at_name>:<at_instance>)
  key_ = structure_.name;
  root_frame_ = key_;
  ROS_DEBUG("AffordanceTemplate::createDisplayObjects() -- key: %s", key_.c_str());
  // set the root frame for the AT
  geometry_msgs::PoseStamped ps;
  ps.pose = robot_interface_->getRobotConfig().root_offset;
  ps.header.frame_id = robot_interface_->getRobotConfig().frame_id;
  
  setFrame(key_, ps);

  // go through and draw each display object and corresponding marker
  int idx=0;
  for(auto &obj: structure_.display_objects) {
    if(!createDisplayObject(obj, idx++)) {   
      ROS_ERROR("AffordanceTemplate::createDisplayObjects() -- error creating obj: %s", obj.name.c_str());
      return false;
    }
  }
  return true;
}


bool AffordanceTemplate::createDisplayObject(affordance_template_object::DisplayObject obj, int idx)
{
  ROS_DEBUG("AffordanceTemplate::createDisplayObject() -- creating Display Object: %s", obj.name.c_str());
  ROS_DEBUG("AffordanceTemplate::createDisplayObject() --  parent: %s", obj.parent.c_str());

  // get the parent frame for the object.  If no parent in the structure, set it as AT root frame
  std::string obj_frame;
  if(obj.parent != "") {
    obj_frame = obj.parent;
  } else {
    obj_frame = root_frame_;
  }
  ROS_DEBUG("AffordanceTemplate::createDisplayObject() -- root_frame: %s", obj_frame.c_str());
  
  // set scaling information (if not already set)
  if(object_scale_factor_.find(obj.name) == std::end(object_scale_factor_)) {
    object_scale_factor_[obj.name] = 1.0;
    ee_scale_factor_[obj.name] = 1.0;
    ROS_DEBUG("AffordanceTemplate::createDisplayObject() -- setting scale factor for %s to default 1.0", obj.name.c_str());
  } 

  if(object_scale_factor_.find(obj.parent) == std::end(object_scale_factor_)) {
    object_scale_factor_[obj.parent] = 1.0;
    ee_scale_factor_[obj.parent] = 1.0;
    ROS_DEBUG("AffordanceTemplate::createDisplayObject() -- setting parent scale factor for %s to default 1.0", obj.parent.c_str());
  }

  // get the object origin pose 
  geometry_msgs::PoseStamped display_pose;
  display_pose.header.frame_id = obj_frame;
  display_pose.pose = originToPoseMsg(obj.origin);

  ROS_DEBUG("AffordanceTemplate::createDisplayObject() -- pose: (%.3f,%.3f,%.3f),(%.3f,%.3f,%.3f,%.3f)", 
    display_pose.pose.position.x,display_pose.pose.position.y,display_pose.pose.position.z,
    display_pose.pose.orientation.x,display_pose.pose.orientation.y,display_pose.pose.orientation.z,display_pose.pose.orientation.w);

  // scale the object location according to parent object scale
  if(object_scale_factor_[obj.parent] != 1.0) {
    display_pose.pose.position.x *= object_scale_factor_[obj.parent];
    display_pose.pose.position.y *= object_scale_factor_[obj.parent];
    display_pose.pose.position.z *= object_scale_factor_[obj.parent];
    ROS_DEBUG("AffordanceTemplate::createDisplayObject() -- scaled pose: (%.3f,%.3f,%.3f),(%.3f,%.3f,%.3f,%.3f)", 
      display_pose.pose.position.x,display_pose.pose.position.y,display_pose.pose.position.z,
      display_pose.pose.orientation.x,display_pose.pose.orientation.y,display_pose.pose.orientation.z,display_pose.pose.orientation.w);
  }

  setFrame(obj.name, display_pose);

  // create Interactive Marker for the object
  visualization_msgs::InteractiveMarker int_marker;

  // check to see if the IM server already has a IM with this name and just update it if so 
  if(server_->get(obj.name, int_marker)) {
    ROS_DEBUG("AffordanceTemplate::createDisplayObject() -- updating old object IM %s", obj.name.c_str());
  } else {
    setupObjectMenu(obj);
  }

  // set basic info for IM
  int_marker.header.frame_id = obj_frame;
  int_marker.header.stamp = ros::Time(0);
  int_marker.name = obj.name;
  int_marker.description = obj.name;
  int_marker.scale = obj.controls.scale*object_scale_factor_[obj.name];
  int_marker.pose = frame_store_[obj.name].second.pose;

  // set up the object display and menus.  Will be a "clickable" hand in RViz.
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;    
  control.markers.clear();

  // get the shape for the object  
  visualization_msgs::Marker marker;
  if(!createDisplayObjectMarker(obj, marker)){
    ROS_ERROR("AffordanceTemplate::createDisplayObject() -- error getting marker");
    return false;
  }
  marker.id = idx;
  control.markers.push_back(marker);

  //  # int_marker = CreateInteractiveMarker(self.frame_id, obj.name, scale)
  int_marker.controls.clear();
  int_marker.controls.push_back(control);
  if(object_controls_display_on_) {
    std::vector<visualization_msgs::InteractiveMarkerControl> dof_controls;
    dof_controls = rit_utils::MarkerHelper::makeCustomDOFControls(obj.controls.translation[0], obj.controls.translation[1], obj.controls.translation[2],
                                                                  obj.controls.rotation[0], obj.controls.rotation[1], obj.controls.rotation[2]);
    for (auto &c: dof_controls)
      int_marker.controls.push_back(c);
  }

  // setup object menu defualts
  setObjectMenuDefaults(obj.name);

  // add interative marker
  addInteractiveMarker(int_marker);

  return true;
}

bool AffordanceTemplate::createDisplayObjectMarker(affordance_template_object::DisplayObject obj, visualization_msgs::Marker &marker) 
{
  marker.text = obj.name;
  marker.ns = obj.name;
  ROS_DEBUG("AffordanceTemplate::createDisplayObjectMarker() -- obj=%s, scale=%.3f", obj.name.c_str(), object_scale_factor_[obj.name]);
  
  if(obj.shape.type == "mesh") {
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = obj.shape.mesh;
    marker.scale.x = obj.shape.size[0]*object_scale_factor_[obj.name];
    marker.scale.y = obj.shape.size[1]*object_scale_factor_[obj.name];
    marker.scale.z = obj.shape.size[2]*object_scale_factor_[obj.name];
    ROS_DEBUG("AffordanceTemplate::createDisplayObjectMarker() -- drawing Mesh for object %s : %s (scale=%.3f)", obj.name.c_str(), marker.mesh_resource.c_str(), object_scale_factor_[obj.name]);
  } else if(obj.shape.type == "box") {
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = obj.shape.size[0]*object_scale_factor_[obj.name];
    marker.scale.y = obj.shape.size[1]*object_scale_factor_[obj.name];
    marker.scale.z = obj.shape.size[2]*object_scale_factor_[obj.name];
  } else if(obj.shape.type == "sphere") {
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = obj.shape.size[0]*object_scale_factor_[obj.name];
    marker.scale.y = obj.shape.size[1]*object_scale_factor_[obj.name];
    marker.scale.z = obj.shape.size[2]*object_scale_factor_[obj.name];
  } else if(obj.shape.type == "cylinder") {
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = obj.shape.radius*object_scale_factor_[obj.name];
    marker.scale.y = obj.shape.length*object_scale_factor_[obj.name];      
  }

  if(obj.shape.type != "mesh") {
    marker.color.r = obj.shape.rgba[0];
    marker.color.g = obj.shape.rgba[1];
    marker.color.b = obj.shape.rgba[2];
    marker.color.a = obj.shape.rgba[3];
  } else {
    marker.mesh_use_embedded_materials = true;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
  }

  return true;

}


bool AffordanceTemplate::createWaypoints() 
{
  ROS_DEBUG("AffordanceTemplate::createWaypoints() -- trajectory: %s", current_trajectory_.c_str());
  // get trajectory info from structure
  Trajectory traj;
  if(!getTrajectory(structure_.ee_trajectories, current_trajectory_, traj)){
    ROS_ERROR("AffordanceTemplate::createWaypoints() -- couldn't get the request trajectory");
    return false;
  }
  // get each the list of waypoints for each EE in the trajectory
  for(auto &wp_list: traj.ee_waypoint_list) {
    int ee_id = wp_list.id;
    ROS_DEBUG("AffordanceTemplate::createWaypoints() creating Trajectory for Waypoint[%d]", ee_id);
    setTrajectoryFlags(traj);  
    // go through the trajectory and set up each waypoint (and corresponding itneractive marker)
    int wp_id = 0;
    for(auto &wp: wp_list.waypoints) {
      if(!createWaypoint(wp, ee_id, wp_id)) {
        ROS_ERROR("AffordanceTemplate::createWaypoints() -- error creating Waypoint: %d.%d for trajectory: %s", ee_id, wp_id, current_trajectory_.c_str());
        return false;
      }
      wp_id++;
    }   
  }
  ROS_DEBUG("AffordanceTemplate::createWaypoints() -- done");
  return true;
}


bool AffordanceTemplate::createWaypoint(affordance_template_object::EndEffectorWaypoint wp, int ee_id, int wp_id) 
{
  // create wp_name of form <ee_id>.<wp_id>:<at_name>:<at_id>
  std::string wp_name = createWaypointID(ee_id, wp_id);
  ROS_DEBUG("AffordanceTemplate::createWaypoint() creating Waypoint: %s", wp_name.c_str());

  // get ee_name from ee_id
  std::map<int, std::string> ee_name_map = robot_interface_->getEENameMap();
  std::string ee_name = ee_name_map[ee_id];

  // get the parent object name and scale
  std::string parent_obj = appendID(wp.display_object);
  double parent_scale = object_scale_factor_[parent_obj]*ee_scale_factor_[parent_obj];  

  // get the WP origin pose 
  geometry_msgs::PoseStamped display_pose;
  display_pose.header.frame_id = parent_obj;
  display_pose.header.stamp = ros::Time(0); 
  display_pose.pose = originToPoseMsg(wp.origin);  

  ROS_DEBUG("AffordanceTemplate::createWaypoint() -- pose: (%.3f,%.3f,%.3f),(%.3f,%.3f,%.3f,%.3f)", 
    display_pose.pose.position.x,display_pose.pose.position.y,display_pose.pose.position.z,
    display_pose.pose.orientation.x,display_pose.pose.orientation.y,display_pose.pose.orientation.z,display_pose.pose.orientation.w);

  // scale the WP location according to parent object scale
  if(parent_scale != 1.0) {
    display_pose.pose.position.x *= parent_scale;
    display_pose.pose.position.y *= parent_scale;
    display_pose.pose.position.z *= parent_scale;
    ROS_DEBUG("AffordanceTemplate::createWaypoint() -- scaled pose: (%.3f,%.3f,%.3f),(%.3f,%.3f,%.3f,%.3f)", 
      display_pose.pose.position.x,display_pose.pose.position.y,display_pose.pose.position.z,
      display_pose.pose.orientation.x,display_pose.pose.orientation.y,display_pose.pose.orientation.z,display_pose.pose.orientation.w);
  }
  
  // set up robot-specific frames not stored in AT structure 

  // get EE link frame (planning frame of arm/manipulator)
  geometry_msgs::PoseStamped ee_pose;
  ee_pose.header.frame_id = wp_name;
  ee_pose.pose = robot_interface_->getManipulatorOffsetPose(ee_name);
  std::string ee_frame_name = getEEFrameName(wp_name);

  // get control point for EE (a robot specific offset from planning frame (i.e., the palm rather then the wrist link))
  geometry_msgs::PoseStamped control_pose;
  control_pose.header.frame_id = ee_frame_name;
  control_pose.pose = robot_interface_->getToolOffsetPose(ee_name);
  std::string cp_frame_name = getControlPointFrameName(wp_name);

  // set up FrameStore frames 
  setFrame(wp_name, display_pose);
  setFrame(ee_frame_name, ee_pose);
  setFrame(cp_frame_name, control_pose);

  // create Interactive Marker for waypoint  
  visualization_msgs::InteractiveMarker int_marker;

  // check to see if the IM server already has a IM with this name and just update it if so 
  if(server_->get(wp_name, int_marker)) {
    ROS_DEBUG("AffordanceTemplate::createWaypoint() -- updating old waypoint IM %s", wp_name.c_str());
  } else {
    // only setup menu the first time
    setupWaypointMenu(wp_name);
  }

  // set basic info for IM
  int_marker.header.frame_id = parent_obj;
  int_marker.header.stamp = ros::Time(0);
  int_marker.name = wp_name;
  int_marker.description = wp_name;
  int_marker.scale = wp.controls.scale;
  int_marker.pose = frame_store_[wp_name].second.pose;

  // set up the EE display and menus.  Will be a "clickable" hand in RViz.
  visualization_msgs::InteractiveMarkerControl menu_control;
  menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;    
  menu_control.always_visible = true;
  menu_control.markers.clear();

  // get the markers for the EE and add them to the IM
  visualization_msgs::MarkerArray markers;
  if(!createWaypointMarkers(ee_id, wp_id, wp.ee_pose, markers)) {
    ROS_ERROR("AffordanceTemplate::createWaypoint() -- error creating markers for %s", wp_name.c_str());
    return false;
  }
  for(auto &m : markers.markers) {
    menu_control.markers.push_back( m );
  }

  // set up WP controls
  int_marker.controls.clear();
  int_marker.controls.push_back(menu_control);
  if(waypoint_flags_[current_trajectory_].controls_on[wp_name]) {
    std::vector<visualization_msgs::InteractiveMarkerControl> dof_controls;
    dof_controls = rit_utils::MarkerHelper::makeCustomDOFControls(wp.controls.translation[0], wp.controls.translation[1], wp.controls.translation[2],
                                                                  wp.controls.rotation[0], wp.controls.rotation[1], wp.controls.rotation[2]);
    for (auto &c: dof_controls) {
      int_marker.controls.push_back(c);
    }
  }

  // set default menu checkbox states
  setWaypointMenuDefaults(wp_name);

  // finally, add the IM and correspodning menu to the server
  addInteractiveMarker(int_marker);

  // setup the toolpoint and cooresponding IM
  if(!createToolpoint(wp, ee_id, wp_id)) {
    ROS_WARN("AffordanceTemplate::createWaypoint() -- problem setting up ToolPoint for %s", wp_name.c_str());
  }

  return true;
}


bool AffordanceTemplate::createToolpoint(affordance_template_object::EndEffectorWaypoint wp, int ee_id, int wp_id) 
{
 
  // create wp_name of form <ee_id>.<wp_id>:<at_name>:<at_id>
  std::string wp_name = createWaypointID(ee_id, wp_id);
  ROS_DEBUG("AffordanceTemplate::createToolpoint() creating Toolpoint for: %s", wp_name.c_str());

  // get the parent object name and scale
  std::string parent_obj = appendID(wp.display_object);
  double parent_scale = object_scale_factor_[parent_obj]*ee_scale_factor_[parent_obj];  

  // get waypoint (tool) offset point and setup the corresponding frame (<wp_name>/tp)
  geometry_msgs::PoseStamped tool_pose;
  tool_pose.header.frame_id = wp_name;
  tool_pose.header.stamp = ros::Time(0);
  std::string tp_frame_name = getToolPointFrameName(wp_name);
  tool_pose.pose = originToPoseMsg(wp.tool_offset); 

   // scale the TP location according to parent object scale
  if(parent_scale != 1.0) {
    tool_pose.pose.position.x *= parent_scale;
    tool_pose.pose.position.y *= parent_scale;
    tool_pose.pose.position.z *= parent_scale;
    ROS_DEBUG("AffordanceTemplate::createToolpoint() -- scaled tool pose: (%.3f,%.3f,%.3f),(%.3f,%.3f,%.3f,%.3f)", 
      tool_pose.pose.position.x,tool_pose.pose.position.y,tool_pose.pose.position.z,
      tool_pose.pose.orientation.x,tool_pose.pose.orientation.y,tool_pose.pose.orientation.z,tool_pose.pose.orientation.w);
  }

  // add tp frame to FrameStore
  setFrame(tp_frame_name, tool_pose);

  if(waypoint_flags_[current_trajectory_].adjust_offset[wp_name] || waypoint_flags_[current_trajectory_].move_offset[wp_name]) {

    // set up the EE display and menus.  Will be a "clickable" hand in RViz.
    visualization_msgs::InteractiveMarkerControl menu_control;
    menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;    
    menu_control.always_visible = true;
    menu_control.markers.clear();

    visualization_msgs::InteractiveMarker tool_offset_marker;
    tool_offset_marker.header.frame_id = tp_frame_name;
    tool_offset_marker.header.stamp = ros::Time(0);
    tool_offset_marker.name = tp_frame_name;
    tool_offset_marker.description = tp_frame_name;      
    tool_offset_marker.scale = 0.1;
    tool_offset_marker.controls.push_back(menu_control);

    std::vector<visualization_msgs::InteractiveMarkerControl> dof_controls;
    dof_controls = rit_utils::MarkerHelper::makeCustomDOFControls(true,true,true,true,true,true);
    for (auto &c: dof_controls) {
      tool_offset_marker.controls.push_back(c);
    }

    // check to see if the IM server already has a IM with this name and just update it if so 
    if(server_->get(tp_frame_name, tool_offset_marker)) {
      ROS_DEBUG("AffordanceTemplate::createToolpoint() -- updating old toolpoint IM %s", tp_frame_name.c_str());
    } else {
      // only setup menu the first time
      setupToolpointMenu(tp_frame_name);
    }

    // set default menu state
    setToolPointMenuDefaults(tp_frame_name);

    // finally add interactive marker
    addInteractiveMarker(tool_offset_marker);
  }

  return true;
}

bool AffordanceTemplate::createWaypointMarkers(int ee_id, int wp_id, int ee_pose, visualization_msgs::MarkerArray &markers)
{

  // return marker array
  markers.markers.clear();

  // create wp_name of form <ee_id>.<wp_id>:<at_name>:<at_id>
  std::string wp_name = createWaypointID(ee_id, wp_id);
  ROS_DEBUG("AffordanceTemplate::createWaypointMarkers() creating MarkersArray for waypoint: %s", wp_name.c_str());

  // get ee_name from ee_id
  std::map<int, std::string> ee_name_map = robot_interface_->getEENameMap();
  std::string ee_name = ee_name_map[ee_id];

   // get the EE pose configuration (hand closed, hand opened, etc.)
  std::string ee_pose_name;;
  try {
    ee_pose_name = robot_interface_->getEEPoseNameMap(ee_name)[ee_pose];
  } catch(...) {
    ee_pose_name = "current";
  }
  ROS_DEBUG("AffordanceTemplate::createWaypointMarkers() -- ee_pose_name[%d]: %s", ee_pose, ee_pose_name.c_str());
     
  // use the EE helper to get the Markers for the EE at the specified ee_pose
  visualization_msgs::MarkerArray ee_markers;
  end_effector_helper::EndEffectorHelperConstPtr ee_link_data;
  if(robot_interface_->getEELinkData(ee_name, ee_link_data)) { 
    if(!ee_link_data->getMarkersForPose(ee_pose_name, ee_markers)) {
      ROS_ERROR("AffordanceTemplate::createWaypointMarkers() -- problem getting pose markers for EE %s, pose: %s", ee_name.c_str(), ee_pose_name.c_str());
      return false;
    } 
  } else {
    ROS_ERROR("AffordanceTemplate::createWaypointMarkers() -- no link data for EE %s", ee_name.c_str());
    return false;        
  }

  // set the EE pose markers
  double N = (double)getNumWaypoints(current_trajectory_, ee_id);
  double current_idx = (double)plan_status_[current_trajectory_][ee_name].current_idx;
  double range = 0.6;
  double inc = range/N;
  double sl = N-current_idx;
  double compact_view_scale = 0.02;
  double adjusted_color = (0.9-range)+inc*(N-1.0-wp_id);

  // set WP color according to whether it has been passed already, as well as how far away it is to end of trajectory 
  std_msgs::ColorRGBA ee_color;
  if(wp_id < current_idx) {
    ee_color = getColorMsg(0.0,0.0,0.25,0.2);
  } else {
    ee_color = getColorMsg(0.0,adjusted_color,adjusted_color,adjusted_color);
  }

  // "compact view" will just draw a small sphere instead of the full EE
  if(waypoint_flags_[current_trajectory_].compact_view[wp_name]) {
    ROS_DEBUG("AffordanceTemplate::createWaypointMarkers() -- displaying %s in COMPACT mode", wp_name.c_str());
    visualization_msgs::Marker m;
    m.header.frame_id = wp_name;
    m.ns = name_;
    m.type = visualization_msgs::Marker::SPHERE;
    m.scale.x = compact_view_scale;
    m.scale.y = compact_view_scale;
    m.scale.z = compact_view_scale;
    m.color = ee_color;
    m.pose.orientation.w = 1;
    m.frame_locked = true;
    markers.markers.push_back( m );
  } else {
    ROS_DEBUG("AffordanceTemplate::createWaypointMarkers() -- displaying %s in FULL mode", wp_name.c_str());
    for(auto &m: ee_markers.markers) {
      visualization_msgs::Marker ee_m = m;
      ee_m.header.frame_id = getControlPointFrameName(wp_name);
      ee_m.ns = name_;
      ee_m.pose = m.pose;
      ee_m.frame_locked = true;
      ee_m.color = ee_color;
      markers.markers.push_back( ee_m );
    }
  }

  return true;
}

bool AffordanceTemplate::insertWaypointInTrajectory(affordance_template_object::EndEffectorWaypoint wp, int ee_id, int wp_id, std::string traj_name)
{

  // set traj to current if not specified
  if(traj_name.empty()) {
    traj_name = current_trajectory_;
  }

  // go through and get the right waypoint list in the traj for the specified EE 
  for (auto& traj : structure_.ee_trajectories) {
    if (traj.name == traj_name) {
      for(auto &ee_list : traj.ee_waypoint_list) {
        if(ee_list.id == ee_id) {
          insertWaypointInList(wp, wp_id, ee_list);
          return true;
        }
      }
      // add a new waypoint list for the ee_id, cause it wasnt found in the current set
      ROS_DEBUG("AffordanceTemplate::insertWaypointInTrajectory() -- creating new EE list for %d", ee_id); 
      affordance_template_object::EndEffectorWaypointList new_ee_list;
      new_ee_list.id = ee_id;
      new_ee_list.waypoints.push_back(wp);
      traj.ee_waypoint_list.push_back(new_ee_list);
      return true;
    }
  }

  ROS_ERROR("AffordanceTemplate::insertWaypointInTrajectory() -- couldn't set wp at %d for ee[%d] in traj[%s]", wp_id, ee_id, traj_name.c_str());
  return false;

}


bool AffordanceTemplate::insertWaypointInList(affordance_template_object::EndEffectorWaypoint wp, int id, affordance_template_object::EndEffectorWaypointList &wp_list) {
  
  ROS_DEBUG("AffordanceTemplate::insertWaypointInList() -- inserting new waypoint at position %d", id);
  
  // insert new waypoint into the list
  wp_list.waypoints.insert(wp_list.waypoints.begin()+id, wp);

  // need to go and move all waypoints after insertion point forward.  
  // will do this from the end to preserve flags
  // wp_name_1 will move into slot wp_name_2.
  std::string wp_name_1, wp_name_2; 
  for(size_t k=wp_list.waypoints.size()-1; (int)k>(int)id; --k) {

    wp_name_1 = createWaypointID(wp_list.id, k-1);
    wp_name_2 = createWaypointID(wp_list.id, k);
    ROS_DEBUG("AffordanceTemplate::insertWaypointInList() -- ID=%d, k=%d -- moving \'%s\' data to \'%s\'", id, (int)k, wp_name_1.c_str(), wp_name_2.c_str());

    // copy waypoint flags
    waypoint_flags_[current_trajectory_].controls_on[wp_name_2]   = waypoint_flags_[current_trajectory_].controls_on[wp_name_1];
    waypoint_flags_[current_trajectory_].compact_view[wp_name_2]  = waypoint_flags_[current_trajectory_].compact_view[wp_name_1];
    waypoint_flags_[current_trajectory_].adjust_offset[wp_name_2] = waypoint_flags_[current_trajectory_].adjust_offset[wp_name_1];
    waypoint_flags_[current_trajectory_].move_offset[wp_name_2]   = waypoint_flags_[current_trajectory_].move_offset[wp_name_1];

    // remove the waypoint
    removeWaypoint(wp_name_1);
    
  }

  // set default flags for new waypoint
  std::string wp_name = createWaypointID(wp_list.id, id);
  ROS_DEBUG("AffordanceTemplate::insertWaypointInList() -- setting waypoint flags for %s",  wp_name.c_str());
  waypoint_flags_[current_trajectory_].controls_on[wp_name] = true;
  waypoint_flags_[current_trajectory_].compact_view[wp_name] = false;
  waypoint_flags_[current_trajectory_].adjust_offset[wp_name] = false;
  waypoint_flags_[current_trajectory_].move_offset[wp_name] = false;           

  return true;
}

bool AffordanceTemplate::deleteWaypointFromTrajectory(int ee_id, int wp_id, std::string traj_name)
{

  // set traj to current if not specified
  if(traj_name.empty()) {
    traj_name = current_trajectory_;
  }

  // go through and get the right waypoint list in the traj for the specified EE 
  for (auto& traj : structure_.ee_trajectories) {
    if (traj.name == traj_name) {
      for(auto &ee_list : traj.ee_waypoint_list) {
        if(ee_list.id == ee_id) {
          deleteWaypointFromList(ee_id, wp_id, ee_list);
          return true;
        }
      }
    }
  }

  ROS_ERROR("AffordanceTemplate::deleteWaypointFromTrajectory() -- couldn't delete wp at %d for ee[%d] in traj[%s]", wp_id, ee_id, traj_name.c_str());
  return false;

}


bool AffordanceTemplate::deleteWaypointFromList(int ee_id, int wp_id, affordance_template_object::EndEffectorWaypointList &wp_list) {

  std::string wp_name = createWaypointID(ee_id, wp_id);
  ROS_DEBUG("AffordanceTemplate::deleteWaypointFromList() -- deleting wp %s (%d)",  wp_name.c_str(), wp_id);
 
  // erase the waypoint from the list
  wp_list.waypoints.erase(wp_list.waypoints.begin() + wp_id);
  
  // need to go and move all waypoints after deletion point backwards.  
  // will do this from the end to preserve flags
  // wp_name_2 will move into slot wp_name_1.
  std::string wp_name_1, wp_name_2;
  for(size_t k=wp_list.waypoints.size(); (int)k>=(int)wp_id; --k) {

    wp_name_1 = createWaypointID(wp_list.id, k);
    wp_name_2 = createWaypointID(wp_list.id, k+1);
    ROS_DEBUG("AffordanceTemplate::deleteWaypointFromList() -- ID=%d, k=%d -- moving \'%s\' data to \'%s\'", wp_id, (int)k, wp_name_2.c_str(), wp_name_1.c_str());

    waypoint_flags_[current_trajectory_].controls_on[wp_name_1] = waypoint_flags_[current_trajectory_].controls_on[wp_name_2];
    waypoint_flags_[current_trajectory_].compact_view[wp_name_1] = waypoint_flags_[current_trajectory_].compact_view[wp_name_2];
    waypoint_flags_[current_trajectory_].adjust_offset[wp_name_1] = waypoint_flags_[current_trajectory_].adjust_offset[wp_name_2];
    waypoint_flags_[current_trajectory_].move_offset[wp_name_1] = waypoint_flags_[current_trajectory_].move_offset[wp_name_2];

    // remove the waypoints
    removeWaypoint(wp_name_1);
    removeWaypoint(wp_name_2);
 
  }

}

bool AffordanceTemplate::getWaypoint(std::string trajectory, int ee_id, int wp_id, affordance_template_object::EndEffectorWaypoint &wp)
{
  for (auto& traj : structure_.ee_trajectories) {
    if (traj.name == trajectory) {
      for(auto &ee_list : traj.ee_waypoint_list) {
        if(ee_list.id == ee_id) {
          if(wp_id < ee_list.waypoints.size()) {
            wp = ee_list.waypoints[wp_id];
            return true;
          } else {
            ROS_WARN("AffordanceTemplate::getWaypoint() -- no wp[%d] found for ee[%d] in traj[%s]", wp_id, ee_id, trajectory.c_str());
            return false;
          }  
        }
      }
    }
  }
  ROS_WARN("AffordanceTemplate::getWaypoint() -- no traj[%s] found with ee[%d]", trajectory.c_str(), ee_id);
  return false;
}

void AffordanceTemplate::setupObjectMenu(DisplayObject obj)
{
  for(auto& o : object_menu_options_) {
    if(o.first == "Choose Trajectory") 
      setupTrajectoryMenu(obj.name);
    else if (o.first.find("Add Waypoint") != std::string::npos)
      setupAddWaypointMenuItem(obj.name, o.first);
    else 
      setupSimpleMenuItem(obj.name, o.first, o.second);
  }
}

void AffordanceTemplate::setupWaypointMenu(std::string name)
{
  for(auto& o : waypoint_menu_options_) {
    if(o.first == "Change End-Effector Pose")
      setupEndEffectorPoseMenu(name);
    else
      setupSimpleMenuItem(name, o.first, o.second);
  }
}

void AffordanceTemplate::setupToolpointMenu(std::string name)
{
  for(auto& o : toolpoint_menu_options_) {
    setupSimpleMenuItem(name, o.first, o.second);
  }
}


void AffordanceTemplate::setupSimpleMenuItem(const std::string& name, const std::string& menu_text, bool has_check_box)
{
  MenuHandleKey key;
  key[name] = {menu_text};  
  group_menu_handles_[key] = marker_menus_[name].insert( menu_text, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ) );  
  if(has_check_box) {
    marker_menus_[name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
  }
}

void AffordanceTemplate::setupAddWaypointMenuItem(std::string name, std::string menu_text)
{
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = marker_menus_[name].insert( menu_text);
  for(auto& ee: robot_interface_->getEENameMap()) {
    std::string ee_readable = robot_interface_->getReadableEEName(ee.second);
    MenuHandleKey key;
    key[name] = {menu_text, ee.second};
    group_menu_handles_[key] = marker_menus_[name].insert( sub_menu_handle, ee_readable, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ) );     
    ROS_DEBUG("[AffordanceTemplate::setupAddWaypointMenuItem] adding submenu text %s to menu item %s", ee_readable.c_str(), menu_text.c_str());
  }
}

void AffordanceTemplate::setupTrajectoryMenu(const std::string& name)
{
  std::string menu_text = "Choose Trajectory";
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = marker_menus_[name].insert( menu_text );
  for(auto &traj: structure_.ee_trajectories) {
    MenuHandleKey key;
    key[name] = {menu_text, traj.name};
    group_menu_handles_[key] = marker_menus_[name].insert( sub_menu_handle, traj.name, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ) );   
    if(traj.name == current_trajectory_) 
      marker_menus_[name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
    else
      marker_menus_[name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );       
    ROS_DEBUG("[AffordanceTemplate::setupTrajectoryMenu] adding submenu text %s to menu item %s", traj.name.c_str(), menu_text.c_str());
  }
}

void AffordanceTemplate::setupEndEffectorPoseMenu(const std::string& name)
{
  std::string menu_text = "Change End-Effector Pose";
  int ee_id = getEEIDfromWaypointName(name);
  std::string ee_name = robot_interface_->getEEName(ee_id);
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = marker_menus_[name].insert( menu_text );
  for(auto &pose_name: robot_interface_->getEEPoseNames(ee_name)) {
    MenuHandleKey key;
    key[name] = {menu_text, pose_name};
    group_menu_handles_[key] = marker_menus_[name].insert( sub_menu_handle, pose_name, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ) ); 
  }
}

int AffordanceTemplate::getEEIDfromWaypointName(const std::string wp_name) 
{
  std::string delimiter = ".";
  size_t pos = 0;
  std::string token;
  if ((pos = wp_name.find(delimiter)) != std::string::npos) {
    token = wp_name.substr(0, pos);
    return std::stoi(token);
  }
  ROS_ERROR("AffordanceTemplate::getEEIDfromWaypointName() -- could not find EE ID from %s", wp_name.c_str());
  return -1;
}

void AffordanceTemplate::addInteractiveMarker(visualization_msgs::InteractiveMarker m)
{
  ROS_DEBUG("AffordanceTemplate::addInteractiveMarker() -- %s with frame: %s", m.name.c_str(), m.header.frame_id.c_str());
  std::string name = m.name;

  visualization_msgs::InteractiveMarker im;
  if(server_->get(name, im)) {
    im = m;
  } else {
    server_->insert(m);
    server_->setCallback(m.name, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ));
  }
  int_markers_[m.name] = m;

  // add the menu too
  marker_menus_[m.name].apply( *server_, m.name );

  // apply the changes to the IM server
  server_->applyChanges();
}

void AffordanceTemplate::removeWaypoint(std::string wp_name)
{
  if(!isWaypoint(wp_name)) {
    ROS_WARN("AffordanceTemplate::removeWaypoint() -- %s is not a waypoint name", wp_name.c_str());
    return;
  }
  ROS_DEBUG("AffordanceTemplate::removeWaypoint() -- %s", wp_name.c_str());
 
  // remove interactive markers
  removeInteractiveMarker(wp_name);
  removeInteractiveMarker(getToolPointFrameName(wp_name));

  // remove frames
  frame_store_.erase(wp_name);
  frame_store_.erase(getControlPointFrameName(wp_name));
  frame_store_.erase(getEEFrameName(wp_name));
  frame_store_.erase(getToolPointFrameName(wp_name));
}

void AffordanceTemplate::removeAllWaypoints()
{
  for(auto &m: int_markers_)
    if(isWaypoint(m.first)) 
      removeWaypoint(m.first);
}

void AffordanceTemplate::removeInteractiveMarker(std::string marker_name) 
{
  ROS_DEBUG("[AffordanceTemplate::removeInteractiveMarker] removing marker %s", marker_name.c_str());
  server_->erase(marker_name);
  marker_menus_.erase(marker_name);
  server_->applyChanges();
}

void AffordanceTemplate::removeAllMarkers() 
{
  for(auto &m: int_markers_)
    removeInteractiveMarker(m.first);
  group_menu_handles_.clear();
  int_markers_.clear();
  marker_menus_.clear();
}

bool AffordanceTemplate::removeMarkerAndRebuild(std::string marker_name)
{
  removeInteractiveMarker(marker_name);
  if(!buildTemplate()) {
    ROS_ERROR("AffordanceTemplate::removeMarkerAndRebuild() -- error rebuilding template after removing marker %s", marker_name.c_str());
    return false;
  }
  return true;  
}

bool AffordanceTemplate::updatePoseFrames(std::string name, geometry_msgs::PoseStamped ps) 
{
  // Most frames will just require updating the FrameStore with the given pose (in the right frame).
  // If it is a toolPoint frame and we are moving it (not adjusting it), however, we need to update 
  // the correspodning waypoint frame, and then reset the toolPoint frame back to 0

  // sanity check that we are trying to adjust a frame that has IM controls
  if(!hasControls(name))
    return false;

  // get the cooresponding waypoint name if its a toolPoint frame
  std::string wp_name;
  if(isToolPointFrame(name)) {
   wp_name = getWaypointFrame(name);
  } 

  if(isToolPointFrame(name) && waypoint_flags_[current_trajectory_].move_offset[wp_name]) {
      
    ROS_DEBUG("AffordanceTemplate::updatePoseFrames() -- moving tool frame: %s", name.c_str());
    std::string obj_frame = frame_store_[wp_name].second.header.frame_id;       
    geometry_msgs::PoseStamped tool_pose = ps;
    geometry_msgs::Pose wp_pose_new;
    tf::Transform origTtp_new, origTwp_new, wpTtp;
    geometry_msgs::PoseStamped wp_pose, tp_in_obj_frame;
    
    try {
      
      // get the toolpoint in the waypoint's base (object) frame
      tf_listener_.transformPose (obj_frame, tool_pose, tp_in_obj_frame);
      
      // calculate new waypoint location, keeping the toolPoint (wpTtp) fixed
      tf::poseMsgToTF(frame_store_[name].second.pose,wpTtp);
      tf::poseMsgToTF(tp_in_obj_frame.pose,origTtp_new);
      origTwp_new = origTtp_new*wpTtp.inverse();
      tf::poseTFToMsg(origTwp_new, wp_pose_new);        
      frame_store_[wp_name].second.pose = wp_pose_new;

      // reset the toolPoint IM back to 0, now that the frames have been updated
      geometry_msgs::PoseStamped fresh_pose;
      fresh_pose.pose.orientation.w = 1.0;
      server_->setPose(name, fresh_pose.pose);
      server_->applyChanges();

    } catch(...) {
      ROS_WARN("AffordanceTemplate::updatePoseFrames(%s) -- %s transform error while moving tool frame", robot_name_.c_str(), name.c_str());
    }

  } else {

    // this is most cases, just update the frame store with the input pose
    if(ps.header.frame_id != frame_store_[name].second.header.frame_id) {
      try {
        tf_listener_.transformPose (frame_store_[name].second.header.frame_id, ps, ps);
      } catch (...) {
        ROS_WARN("AffordanceTemplate::updatePoseFrames() -- %s transform error while adjusting feedback frame", name.c_str());
      }
    }
    ROS_DEBUG("AffordanceTemplate::updatePoseFrames() -- storing pose for %s", name.c_str());
    frame_store_[name].second = ps;  

  }

  return true;
}

bool AffordanceTemplate::updatePoseInStructure(std::string name, geometry_msgs::Pose p) 
{
  // check the objects for the name
  for (auto& d : structure_.display_objects) {
    if (name == d.name) {
      ROS_DEBUG("AffordanceTemplate::updatePoseInStructure() -- saving pose for object %s", name.c_str());
      d.origin = poseMsgToOrigin(p);
      return true;
    }
  }

  // check the waypoints
  for (auto& traj : structure_.ee_trajectories) {
    if (traj.name == current_trajectory_) {
      // look for the object the user selected in our waypoint list
      for (auto& wp_list: traj.ee_waypoint_list) {
        int wp_id = -1; // init to -1 because we pre-add
        for (auto& wp: wp_list.waypoints) {
          std::string wp_name = createWaypointID(wp_list.id, ++wp_id);
          if (wp_name == name) {
            ROS_DEBUG("AffordanceTemplate::updatePoseInStructure() -- saving pose for EE waypoint %s", name.c_str());
            wp.origin = poseMsgToOrigin(p);
            return true;
          } else if(isToolPointFrame(name)) {
            if (wp_name == getWaypointFrame(name)) {
              wp.origin = poseMsgToOrigin(frame_store_[wp_name].second.pose);
              wp.tool_offset = poseMsgToOrigin(frame_store_[name].second.pose);
              ROS_WARN("AffordanceTemplate::updatePoseInStructure() -- saving toolPoint and pose for EE waypoint %s", wp_name.c_str());
              return true;
            }
          }
        }
      }
    }
  }

  // didn't find anything that matches the input name
  return false;
}

bool AffordanceTemplate::addWaypointBeforeHandler(std::string wp_name) {

  affordance_template_object::EndEffectorWaypoint wp, wp_new, wp_prev;

  // get the current waypoint
  if(!isWaypoint(wp_name)) {
    ROS_ERROR("AffordanceTemplate::addWaypointBeforeHandler() -- %s not a waypoint, can't add a new Waypoint before it", wp_name.c_str());
    return false;
  }
  int wp_id = getWaypointIDFromName(wp_name);
  int ee_id = getEEIDFromName(wp_name);
  getWaypoint(current_trajectory_, ee_id, wp_id, wp);

  // set basic stuff for new wp based on the one we are adding it off of 
  wp_new.ee_pose = wp.ee_pose;
  wp_new.display_object = wp.display_object;
  wp_new.controls = wp.controls;
  wp_new.planner_type = wp.planner_type;
  wp_new.bounds = wp.bounds;
  wp_new.conditioning_metric = wp.conditioning_metric;
  wp_new.task_compatibility = wp.task_compatibility;
  wp_new.origin = wp.origin; // copy the orienation, will adjust position below
  wp_new.tool_offset.position[0] = wp_new.tool_offset.position[1] = wp_new.tool_offset.position[2] = 0.0; 
  wp_new.tool_offset.orientation[0] = wp_new.tool_offset.orientation[1] = wp_new.tool_offset.orientation[2] = 0.0; 

  // set the position to be halfway between selected wp and the previous wp (or just off it, if selected is the first in traj)
  if(wp_id > 0) {
    ROS_DEBUG("AffordanceTemplate::addWaypointBeforeHandler() -- averging position with wp %d", wp_id-1);
    getWaypoint(current_trajectory_, ee_id, wp_id-1, wp_prev);
    for(int i=0; i<3; i++) {
      wp_new.origin.position[i] = (wp_prev.origin.position[i] + wp.origin.position[i])/2.0;
    }            
    ROS_DEBUG("wp_%d pose  = (%.3f,%.3f,%.3f)",wp_id-1,wp_prev.origin.position[0],wp_prev.origin.position[1],wp_prev.origin.position[2]);
    ROS_DEBUG("wp_%d pose  = (%.3f,%.3f,%.3f)",wp_id,wp.origin.position[0],wp.origin.position[1],wp.origin.position[2]);
    ROS_DEBUG("new wp pose = (%.3f,%.3f,%.3f)",wp_new.origin.position[0],wp_new.origin.position[1],wp_new.origin.position[2]);
  } else {
    for(int i=0; i<3; i++) {
      wp_new.origin.position[i] = wp.origin.position[i] + 0.025;
    }
  }

  // insert the new waypoint into the structure and rebuild the template IMs
  if(!insertWaypointInTrajectory(wp_new, ee_id, wp_id)) {
    ROS_ERROR("AffordanceTemplate::addWaypointBeforeHandler() -- problem inserting new waypoint before %s", wp_name.c_str());
    return false;
  } 
  buildTemplate();
  return true;

}


bool AffordanceTemplate::addWaypointAfterHandler(std::string wp_name) 
{

  ROS_DEBUG("AffordanceTemplate::addWaypointAfterHandler() -- [Add Waypoint After] for EE waypoint %s", wp_name.c_str());
  affordance_template_object::EndEffectorWaypoint wp, wp_new, wp_next;

  // get the current waypoint
  if(!isWaypoint(wp_name)) {
    ROS_ERROR("AffordanceTemplate::addWaypointAfterHandler() -- %s not a waypoint, can't add a new Waypoint after it", wp_name.c_str());
    return false;
  }
  int wp_id = getWaypointIDFromName(wp_name);
  int ee_id = getEEIDFromName(wp_name);
  getWaypoint(current_trajectory_, ee_id, wp_id, wp);

  // set basic stuff for new wp based on the one we are adding it off of 
  wp_new.ee_pose = wp.ee_pose;
  wp_new.display_object = wp.display_object;
  wp_new.controls = wp.controls;
  wp_new.planner_type = wp.planner_type;
  wp_new.bounds = wp.bounds;
  wp_new.conditioning_metric = wp.conditioning_metric;
  wp_new.task_compatibility = wp.task_compatibility;
  wp_new.origin = wp.origin; // copy the orienation, will adjust position below
  wp_new.tool_offset.position[0] = wp_new.tool_offset.position[1] = wp_new.tool_offset.position[2] = 0.0; 
  wp_new.tool_offset.orientation[0] = wp_new.tool_offset.orientation[1] = wp_new.tool_offset.orientation[2] = 0.0; 

  // set the position to be halfway between selected wp and the previous wp (or just off it, if selected is the first in traj)
  if(wp_id < getNumWaypoints(current_trajectory_,ee_id)-1) {
    ROS_DEBUG("AffordanceTemplate::addWaypointAfterHandler() -- averging position with wp %d", wp_id+1);
    getWaypoint(current_trajectory_, ee_id, wp_id+1, wp_next);
    for(int i=0; i<3; i++) {
      wp_new.origin.position[i] = (wp_next.origin.position[i] + wp.origin.position[i])/2.0;
    }            
    ROS_DEBUG("wp_%d pose  = (%.3f,%.3f,%.3f)",wp_id,wp.origin.position[0],wp.origin.position[1],wp.origin.position[2]);
    ROS_DEBUG("wp_%d pose  = (%.3f,%.3f,%.3f)",wp_id+1,wp_next.origin.position[0],wp_next.origin.position[1],wp_next.origin.position[2]);
    ROS_DEBUG("new wp pose = (%.3f,%.3f,%.3f)",wp_new.origin.position[0],wp_new.origin.position[1],wp_new.origin.position[2]);
  } else {
    for(int i=0; i<3; i++) {
      wp_new.origin.position[i] = wp.origin.position[i] - 0.025;
    }
  }

  // insert the new waypoint into the structure and rebuild the template IMs
  if(!insertWaypointInTrajectory(wp_new, ee_id, wp_id+1)) {
    ROS_ERROR("AffordanceTemplate::addWaypointAfterHandler() -- problem inserting new waypoint after %s", wp_name.c_str());
    return false;
  } 
  buildTemplate();
  return true;

}

bool AffordanceTemplate::addWaypointBeforeTrajectoryHandler(std::string obj_name, int ee_id) 
{
  ROS_DEBUG("AffordanceTemplate::addWaypointBeforeTrajectoryHandler() -- [Add Waypoint BeforeE] for Object %s", obj_name.c_str());
  affordance_template_object::EndEffectorWaypoint wp, wp_new;

  // get the current waypoint
  if(!isObject(obj_name)) {
    ROS_ERROR("AffordanceTemplate::addWaypointBeforeTrajectoryHandler() -- %s not an object, can't add a new Waypoint before it", obj_name.c_str());
    return false;
  }
  int wp_id = getNumWaypoints(current_trajectory_,ee_id);

  ROS_DEBUG("AffordanceTemplate::addWaypointBeforeTrajectoryHandler() -- ee_id: %d, wp_id: %d", ee_id, wp_id);
             
  if(wp_id > 0) {

    // set basic stuff for new wp based on the last one in the trajectory 
    getWaypoint(current_trajectory_, ee_id, 0, wp);
    wp_new.ee_pose = wp.ee_pose;
    wp_new.display_object = wp.display_object;
    wp_new.controls = wp.controls;
    wp_new.planner_type = wp.planner_type;
    wp_new.bounds = wp.bounds;
    wp_new.conditioning_metric = wp.conditioning_metric;
    wp_new.task_compatibility = wp.task_compatibility;
    wp_new.origin = wp.origin; // copy the orienation, will adjust position below
    for(int i=0; i<3; i++) {
      wp_new.origin.position[i] = wp.origin.position[i] + 0.025;
    }
    
  } else {

    // there are no wp's for this ee-id, so just set default basic stuff 
    wp_new.ee_pose = 0;
    wp_new.display_object = removeID(obj_name); // wtf
    wp_new.controls.translation[0] = wp_new.controls.translation[1] = wp_new.controls.translation[2] = true;
    wp_new.controls.rotation[0] = wp_new.controls.rotation[1] = wp_new.controls.rotation[2] = true;
    wp_new.controls.scale = 0.25;             
    wp_new.origin.position[0] = wp_new.origin.position[1] = wp_new.origin.position[2] = 0.1;
    wp_new.origin.orientation[0] = wp_new.origin.orientation[1] = wp_new.origin.orientation[2] = 0.0;
    
    // what to do here?
    wp_new.planner_type = stringToPlannerType("CARTESIAN");
    wp_new.bounds = wp.bounds;
    wp_new.conditioning_metric = wp.conditioning_metric;
    wp_new.task_compatibility = wp.task_compatibility;
    
  }

  wp_new.tool_offset.position[0] = wp_new.tool_offset.position[1] = wp_new.tool_offset.position[2] = 0.0; 
  wp_new.tool_offset.orientation[0] = wp_new.tool_offset.orientation[1] = wp_new.tool_offset.orientation[2] = 0.0; 

  ROS_DEBUG("new wp pose = (%.3f,%.3f,%.3f)",wp_new.origin.position[0],wp_new.origin.position[1],wp_new.origin.position[2]);

  // insert the new waypoint into the structure and rebuild the template IMs
  if(!insertWaypointInTrajectory(wp_new, ee_id, 0)) {
    ROS_ERROR("AffordanceTemplate::addWaypointBeforeTrajectoryHandler() -- problem inserting new waypoint before %s", obj_name.c_str());
    return false;
  } 
  buildTemplate();
  
  return true;

}

bool AffordanceTemplate::addWaypointAfterTrajectoryHandler(std::string obj_name, int ee_id)
{
  
  ROS_DEBUG("AffordanceTemplate::addWaypointAfterTrajectoryHandler() -- [Add Waypoint After] for Object %s", obj_name.c_str());
  affordance_template_object::EndEffectorWaypoint wp, wp_new;

  // get the current waypoint
  if(!isObject(obj_name)) {
    ROS_ERROR("AffordanceTemplate::addWaypointAfterTrajectoryHandler() -- %s not an object, can't add a new Waypoint after it", obj_name.c_str());
    return false;
  }
  int wp_id = getNumWaypoints(current_trajectory_,ee_id);

  ROS_DEBUG("AffordanceTemplate::addWaypointAfterTrajectoryHandler() -- ee_id: %d, wp_id: %d", ee_id, wp_id);
             
  if(wp_id > 0) {

    // set basic stuff for new wp based on the last one in the trajectory 
    getWaypoint(current_trajectory_, ee_id, wp_id-1, wp);
    wp_new.ee_pose = wp.ee_pose;
    wp_new.display_object = wp.display_object;
    wp_new.controls = wp.controls;
    wp_new.planner_type = wp.planner_type;
    wp_new.bounds = wp.bounds;
    wp_new.conditioning_metric = wp.conditioning_metric;
    wp_new.task_compatibility = wp.task_compatibility;
    wp_new.origin = wp.origin; // copy the orienation, will adjust position below
    for(int i=0; i<3; i++) {
      wp_new.origin.position[i] = wp.origin.position[i] - 0.025;
    }
    
  } else {

    // there are no wp's for this ee-id, so just set default basic stuff 
    wp_new.ee_pose = 0;
    wp_new.display_object = removeID(obj_name); // wtf
    wp_new.controls.translation[0] = wp_new.controls.translation[1] = wp_new.controls.translation[2] = true;
    wp_new.controls.rotation[0] = wp_new.controls.rotation[1] = wp_new.controls.rotation[2] = true;
    wp_new.controls.scale = 0.25;             
    wp_new.origin.position[0] = wp_new.origin.position[1] = wp_new.origin.position[2] = 0.1;
    wp_new.origin.orientation[0] = wp_new.origin.orientation[1] = wp_new.origin.orientation[2] = 0.0;
    
    // what to do here?
    wp_new.planner_type = stringToPlannerType("CARTESIAN");
    wp_new.bounds = wp.bounds;
    wp_new.conditioning_metric = wp.conditioning_metric;
    wp_new.task_compatibility = wp.task_compatibility;
    
  }

  wp_new.tool_offset.position[0] = wp_new.tool_offset.position[1] = wp_new.tool_offset.position[2] = 0.0; 
  wp_new.tool_offset.orientation[0] = wp_new.tool_offset.orientation[1] = wp_new.tool_offset.orientation[2] = 0.0; 

  ROS_DEBUG("new wp pose = (%.3f,%.3f,%.3f)",wp_new.origin.position[0],wp_new.origin.position[1],wp_new.origin.position[2]);

  // insert the new waypoint into the structure and rebuild the template IMs
  if(!insertWaypointInTrajectory(wp_new, ee_id, wp_id)) {
    ROS_ERROR("AffordanceTemplate::addWaypointAfterTrajectoryHandler() -- problem inserting new waypoint after %s", obj_name.c_str());
    return false;
  } 
  buildTemplate();
 
  return true;
}
    
bool AffordanceTemplate::deleteWaypointHandler(std::string wp_name)
{
  ROS_DEBUG("AffordanceTemplate::deleteWaypointHandler() -- %s", wp_name.c_str());
  // get the current waypoint
  if(!isWaypoint(wp_name)) {
    ROS_ERROR("AffordanceTemplate::deleteWaypointHandler() -- %s not a waypoint", wp_name.c_str());
    return false;
  }
  int wp_id = getWaypointIDFromName(wp_name);
  int ee_id = getEEIDFromName(wp_name);
  if(!deleteWaypointFromTrajectory(ee_id, wp_id)){
    ROS_ERROR("AffordanceTemplate::deleteWaypointHandler() -- error deleting waypoint %s", wp_name.c_str());
    return false;
  }
  buildTemplate();
  return true;
}

void AffordanceTemplate::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) 
{
  ROS_DEBUG("AffordanceTemplate::processFeedback(%s) -- %s", robot_name_.c_str(), feedback->marker_name.c_str());

  interactive_markers::MenuHandler::CheckState state;

  // set up key maps for easy comparison to menu handler ID
  MenuHandleKey wp_before_key;
  MenuHandleKey wp_after_key;
  MenuHandleKey reset_key;
  MenuHandleKey save_key;
  MenuHandleKey delete_key;
  MenuHandleKey hide_controls_key;
  MenuHandleKey view_mode_key;
  MenuHandleKey play_plan_key;
  MenuHandleKey loop_key;
  MenuHandleKey autoplay_key;
  MenuHandleKey adjust_offset_key;
  MenuHandleKey move_offset_key;

  wp_before_key[feedback->marker_name]             = {"Add Waypoint Before"};
  wp_after_key[feedback->marker_name]              = {"Add Waypoint After"};
  reset_key[feedback->marker_name]                 = {"Reset"};
  save_key[feedback->marker_name]                  = {"Save"};
  delete_key[feedback->marker_name]                = {"Delete Waypoint"};
  hide_controls_key[feedback->marker_name]         = {"Hide Controls"};
  view_mode_key[feedback->marker_name]             = {"Compact View"};
  play_plan_key[feedback->marker_name]             = {"(Re)Play Plan"};
  loop_key[feedback->marker_name]                  = {"Loop Animation"};
  autoplay_key[feedback->marker_name]              = {"Autoplay"};
  adjust_offset_key[feedback->marker_name]         = {"Change Tool Offset"};
  move_offset_key[feedback->marker_name]           = {"Move Tool Point"};

  // update the fames everytime this is called
  geometry_msgs::PoseStamped ps;
  ps.pose = feedback->pose;
  ps.header = feedback->header;
  if(!updatePoseFrames(feedback->marker_name, ps)) {
    ROS_ERROR("AffordanceTemplate::processFeedback() -- error updating pose frames for %s", feedback->marker_name.c_str());
    return;
  }

  switch ( feedback->event_type ) {

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP :
    {
      if(!updatePoseInStructure(feedback->marker_name, feedback->pose)) {
        ROS_ERROR("AffordanceTemplate::processFeedback() -- problem updating pose in structure for %s", feedback->marker_name.c_str());
        break;
      }
      break;
    }
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT : {
      ROS_DEBUG("[AffordanceTemplate::processFeedback] %s selected menu entry: %d", feedback->marker_name.c_str(), feedback->menu_entry_id);
      // ROS_DEBUG("AffordanceTemplate::processFeedback() --   pose: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f), frame_id: %s", 
      //                                                             feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z, 
      //                                                             feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w,
      //                                                             feedback->header.frame_id.c_str());

     
      // 
      // check for 'Add Waypoint Before' for EE objects
      if (group_menu_handles_.find(wp_before_key) != std::end(group_menu_handles_)) {
        if (group_menu_handles_[wp_before_key] == feedback->menu_entry_id) {         
          if(!addWaypointBeforeHandler(feedback->marker_name)) {
            ROS_ERROR("AffordanceTemplate::processFeedback() -- error adding new waypoint before %s", feedback->marker_name.c_str());
          }
          break;
        }
      }

      // check for 'Add Waypoint After' for EE objects
      if (group_menu_handles_.find(wp_after_key) != std::end(group_menu_handles_)) {
        if (group_menu_handles_[wp_after_key] == feedback->menu_entry_id) {
          if(!addWaypointAfterHandler(feedback->marker_name)) {
            ROS_ERROR("AffordanceTemplate::processFeedback() -- error adding new waypoint after %s", feedback->marker_name.c_str());
          }
          break; 
        }
      }

      // 
      // check for 'Add Waypoint Before' for AT object (wheel, door, etc) 
      for (auto& ee: robot_interface_->getEENameMap()) 
      {
        MenuHandleKey key;
        key[feedback->marker_name] = {"Add Waypoint Before", ee.second};
        if (group_menu_handles_.find(key) != std::end(group_menu_handles_)) {
          if (group_menu_handles_[key] == feedback->menu_entry_id) {
            int ee_id = robot_interface_->getEEID(ee.second);
            if(!addWaypointBeforeTrajectoryHandler(feedback->marker_name, ee_id)) {
              ROS_ERROR("AffordanceTemplate::processFeedback() -- error adding new waypoint before object %s", feedback->marker_name.c_str());
            }
            break;
          }
        }
      }

      // 
      // check for 'Add Waypoint After' for AT object (wheel, door, etc) 
      for (auto& ee: robot_interface_->getEENameMap()) 
      {
        MenuHandleKey key;
        key[feedback->marker_name] = {"Add Waypoint After", ee.second};
        if (group_menu_handles_.find(key) != std::end(group_menu_handles_)) {
          if (group_menu_handles_[key] == feedback->menu_entry_id) {
            int ee_id = robot_interface_->getEEID(ee.second);
            if(!addWaypointAfterTrajectoryHandler(feedback->marker_name, ee_id)) {
              ROS_ERROR("AffordanceTemplate::processFeedback() -- error adding new waypoint after object %s", feedback->marker_name.c_str());
            }
            break;
          }
        }
      }

      //
      // delete waypoint
      if (group_menu_handles_.find(delete_key) != group_menu_handles_.end()) {
        if (group_menu_handles_[delete_key] == feedback->menu_entry_id) {
          if(!deleteWaypointHandler(feedback->marker_name)){
            ROS_ERROR("AffordanceTemplate::processFeedback() -- error deleting waypoint %s", feedback->marker_name.c_str());
          }
          break;
        }
      }

      //
      // reset AT
      if(group_menu_handles_.find(reset_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[reset_key] == feedback->menu_entry_id) {
          ROS_DEBUG("AffordanceTemplate::processFeedback::Reset] resetting current structure to the inital structure.");
          structure_ = initial_structure_;      
          appendIDToStructure(structure_);
          removeAllMarkers();
          clearTrajectoryFlags();
          buildTemplate(); 
        }
      }

      //
      // save AT to file
      if (group_menu_handles_.find(save_key) != group_menu_handles_.end()) 
      {
        if (group_menu_handles_[save_key] == feedback->menu_entry_id) 
        {
          ROS_DEBUG("[AffordanceTemplate::processFeedback::Save] saving file");
          std::vector<std::string> keys;
          boost::split(keys, structure_.filename, boost::is_any_of("/"));
          if (keys.size())
            saveToDisk(keys.back(), structure_.image, feedback->marker_name, true);
          else
            ROS_ERROR("[AffordanceTemplate::processFeedback::Save] invalid filename: %s", structure_.filename.c_str());
        }
      }

      //
      // toggle controls
      if(group_menu_handles_.find(hide_controls_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[hide_controls_key] == feedback->menu_entry_id) {
          ROS_DEBUG("AffordanceTemplate::processFeedback() --   CONTROLS TOGGLE");
          if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state ) ) {
            if(state == interactive_markers::MenuHandler::CHECKED) {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED );
              if(isObject(feedback->marker_name)) {
                object_controls_display_on_ = true;
              } else {
                waypoint_flags_[current_trajectory_].controls_on[feedback->marker_name] = true;
              }
            } else {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED );
              if(isObject(feedback->marker_name)) {
                object_controls_display_on_ = false;
              } else {
                waypoint_flags_[current_trajectory_].controls_on[feedback->marker_name] = false;
              }
            }
          }
          if(!removeMarkerAndRebuild(feedback->marker_name)) {
            ROS_ERROR("AffordanceTemplate::processFeedback() -- failed toggling controls");
          }
        }
      }

      //
      // toggle EE compact view 
      if(group_menu_handles_.find(view_mode_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[view_mode_key] == feedback->menu_entry_id) {
          ROS_DEBUG("AffordanceTemplate::processFeedback() --   VIEW MODE TOGGLE");
          if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state ) ) {
            if(state == interactive_markers::MenuHandler::UNCHECKED) {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED );
              if(!isObject(feedback->marker_name)) {
                waypoint_flags_[current_trajectory_].compact_view[feedback->marker_name] = true;
              }
            } else {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED );
              if(!isObject(feedback->marker_name)) {
                waypoint_flags_[current_trajectory_].compact_view[feedback->marker_name] = false;
              }
            }
          }
          if(!removeMarkerAndRebuild(feedback->marker_name)) {
            ROS_ERROR("AffordanceTemplate::processFeedback() -- failed toggling view mode");
          }
        }
      }

      //
      // toggle adjust tool 6dof markers  
      if(group_menu_handles_.find(adjust_offset_key) != std::end(group_menu_handles_)) {
        // figure out if the menu was selected from the wp or the tp 
        std::string wp_name, tp_name;
        if(isWaypoint(feedback->marker_name)) {
          wp_name = feedback->marker_name;
          tp_name = getToolPointFrameName(wp_name);
        } else if(isToolPointFrame(feedback->marker_name)) {
          tp_name = feedback->marker_name;
          wp_name = getWaypointFrame(tp_name);
        }
        if(group_menu_handles_[adjust_offset_key] == feedback->menu_entry_id) {
          ROS_DEBUG("AffordanceTemplate::processFeedback() --   ADJUST TOOL TOGGLE");
          if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state ) ) {
            if(state == interactive_markers::MenuHandler::UNCHECKED) {
              waypoint_flags_[current_trajectory_].adjust_offset[wp_name] = true;
              waypoint_flags_[current_trajectory_].move_offset[wp_name] = false;
              marker_menus_[wp_name].setCheckState( group_menu_handles_[adjust_offset_key], interactive_markers::MenuHandler::CHECKED );
              marker_menus_[tp_name].setCheckState( group_menu_handles_[adjust_offset_key], interactive_markers::MenuHandler::CHECKED );
              marker_menus_[wp_name].setCheckState( group_menu_handles_[move_offset_key], interactive_markers::MenuHandler::UNCHECKED );
              marker_menus_[tp_name].setCheckState( group_menu_handles_[move_offset_key], interactive_markers::MenuHandler::UNCHECKED );                          
            } else {           
              waypoint_flags_[current_trajectory_].adjust_offset[wp_name] = false;
              waypoint_flags_[current_trajectory_].move_offset[wp_name] = false;
              marker_menus_[wp_name].setCheckState( group_menu_handles_[adjust_offset_key], interactive_markers::MenuHandler::UNCHECKED );
              marker_menus_[tp_name].setCheckState( group_menu_handles_[adjust_offset_key], interactive_markers::MenuHandler::UNCHECKED );
              marker_menus_[wp_name].setCheckState( group_menu_handles_[move_offset_key], interactive_markers::MenuHandler::UNCHECKED );
              marker_menus_[tp_name].setCheckState( group_menu_handles_[move_offset_key], interactive_markers::MenuHandler::UNCHECKED );
            }
            removeInteractiveMarker(wp_name);
            removeInteractiveMarker(tp_name);                   
            if(!buildTemplate()) {
              ROS_ERROR("AffordanceTemplate::processFeedback() -- failed toggling adust tool controls");
            }
          }
        }
      }

      //
      // toggle move tool 6dof markers  
      if(group_menu_handles_.find(move_offset_key) != std::end(group_menu_handles_)) {
        // figure out if the menu was selected from the wp or the tp 
        std::string wp_name, tp_name;
        if(isWaypoint(feedback->marker_name)) {
          wp_name = feedback->marker_name;
          tp_name = getToolPointFrameName(wp_name);
        } else if(isToolPointFrame(feedback->marker_name)) {
          tp_name = feedback->marker_name;
          wp_name = getWaypointFrame(tp_name);
        }  
        if( group_menu_handles_[move_offset_key] == feedback->menu_entry_id ) {
          ROS_DEBUG("AffordanceTemplate::processFeedback() --   MOVE TOOL TOGGLE");
          if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state ) ) {
            if(state == interactive_markers::MenuHandler::UNCHECKED) {
              waypoint_flags_[current_trajectory_].adjust_offset[wp_name] = false;
              waypoint_flags_[current_trajectory_].move_offset[wp_name] = true;
              marker_menus_[wp_name].setCheckState( group_menu_handles_[move_offset_key], interactive_markers::MenuHandler::CHECKED );
              marker_menus_[tp_name].setCheckState( group_menu_handles_[move_offset_key], interactive_markers::MenuHandler::CHECKED );
              marker_menus_[wp_name].setCheckState( group_menu_handles_[adjust_offset_key], interactive_markers::MenuHandler::UNCHECKED );
              marker_menus_[tp_name].setCheckState( group_menu_handles_[adjust_offset_key], interactive_markers::MenuHandler::UNCHECKED );          
            } else {
              waypoint_flags_[current_trajectory_].move_offset[wp_name] = false;
              waypoint_flags_[current_trajectory_].adjust_offset[wp_name] = false;
              marker_menus_[wp_name].setCheckState( group_menu_handles_[move_offset_key], interactive_markers::MenuHandler::UNCHECKED );
              marker_menus_[tp_name].setCheckState( group_menu_handles_[move_offset_key], interactive_markers::MenuHandler::UNCHECKED );
              marker_menus_[wp_name].setCheckState( group_menu_handles_[adjust_offset_key], interactive_markers::MenuHandler::UNCHECKED );
              marker_menus_[tp_name].setCheckState( group_menu_handles_[adjust_offset_key], interactive_markers::MenuHandler::UNCHECKED );
            }
            removeInteractiveMarker(wp_name);
            removeInteractiveMarker(tp_name);
            if(!buildTemplate()) {
              ROS_ERROR("AffordanceTemplate::processFeedback() -- failed toggling adust tool controls");
            }
          }
        }
      }

      //
      // switch trajectories using the context menu
      for (auto &traj: structure_.ee_trajectories) {
        MenuHandleKey key;
        key[feedback->marker_name] = {"Choose Trajectory", traj.name}; // FIXME -- can this be static like this??
        if (group_menu_handles_.find(key) != group_menu_handles_.end()) {
          marker_menus_[feedback->marker_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED);
          if (group_menu_handles_[key] == feedback->menu_entry_id)  {
            ROS_DEBUG("[AffordanceTemplate::processFeedback::Choose Trajectory] found matching trajectory name %s", traj.name.c_str());
            setTrajectory(traj.name);
            marker_menus_[feedback->marker_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED);
            removeAllMarkers();//FIXME here
            if(!buildTemplate()) {
              ROS_ERROR("AffordanceTemplate::processFeedback() -- failed switching trajectory");
            }
          }
        }
      }

      //
      // switch EE pose of a WP
      if(isWaypoint(feedback->marker_name)) {
        int ee_id = getEEIDfromWaypointName(feedback->marker_name);
        std::string ee_name = robot_interface_->getEEName(ee_id);
        for(auto &pn : robot_interface_->getEEPoseNames(ee_name)) {
          MenuHandleKey key;
          key[feedback->marker_name] = {"Change End-Effector Pose", pn};
          if (group_menu_handles_.find(key) != std::end(group_menu_handles_)) {
            if (group_menu_handles_[key] == feedback->menu_entry_id) {
              ROS_DEBUG("AffordanceTemplate::processFeedback() -- changing EE[%s] pose to \'%s\'", ee_name.c_str(), pn.c_str());
              for (auto& traj : structure_.ee_trajectories) {
                if (traj.name == current_trajectory_) {
                  // look for the object the user selected in our waypoint list
                  for (auto& wp_list: traj.ee_waypoint_list) {
                    int wp_id = -1; // init to -1 because we pre-add
                    for (auto& wp: wp_list.waypoints) {
                      std::string wp_name = createWaypointID(wp_list.id, ++wp_id);
                      if (wp_name == feedback->marker_name) {
                        wp.ee_pose = robot_interface_->getEEPoseIDMap(ee_name)[pn];
                        if(!removeMarkerAndRebuild(feedback->marker_name)) {
                          ROS_ERROR("AffordanceTemplate::processFeedback() -- failed creating structure with new EE pose");
                        }
                        break;
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }     

      if (group_menu_handles_.find(play_plan_key) != std::end(group_menu_handles_)) {
        if (group_menu_handles_[play_plan_key] == feedback->menu_entry_id) {
          ROS_DEBUG("[AffordanceTemplate::processFeedback] playing available plan");
          robot_interface_->getPlanner()->playAnimation();
        }
      }

      if (group_menu_handles_.find(loop_key) != std::end(group_menu_handles_)) {
        if (group_menu_handles_[loop_key] == feedback->menu_entry_id) {
          ROS_DEBUG("[AffordanceTemplate::processFeedback] changing looping functionality");
          MenuHandleKey key;
          key[feedback->marker_name] = {"Loop Animation"};
          bool loop = false;
          if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state ) ) {
            if(state == interactive_markers::MenuHandler::CHECKED) {
              marker_menus_[feedback->marker_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
            } else {
              marker_menus_[feedback->marker_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
              loop = true; // transitioning from not looping to looping
            }
            marker_menus_[feedback->marker_name].apply( *server_, feedback->marker_name );
            robot_interface_->getPlanner()->loopAnimation(feedback->marker_name, loop);
          } else {
            ROS_ERROR("[AffordanceTemplate::processFeedback] can't get the loop state!!");
          }
        }
      }

      if (group_menu_handles_.find(autoplay_key) != std::end(group_menu_handles_)) {
        if (group_menu_handles_[autoplay_key] == feedback->menu_entry_id) {
          ROS_WARN("[AffordanceTemplate::processFeedback] flipping autoplay functionality");
          MenuHandleKey key;
          key[feedback->marker_name] = {"Autoplay"};
          if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state ) )  {
            if(state == interactive_markers::MenuHandler::CHECKED)  {
              marker_menus_[feedback->marker_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
              autoplay_display_ = false;
            } else {
              marker_menus_[feedback->marker_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
              autoplay_display_ = true;
            }
            marker_menus_[feedback->marker_name].apply( *server_, feedback->marker_name );
          } else {
            ROS_ERROR("[AffordanceTemplate::processFeedback] can't get the autoplay state!!");
          }
        }
      }
      break;
    }

    default : 
      ROS_DEBUG("[AffordanceTemplate::processFeedback] got unrecognized or unmatched menu event: %d", feedback->event_type);
      break;
  }
  server_->applyChanges();
  marker_menus_[feedback->marker_name].apply( *server_, feedback->marker_name );
}

bool AffordanceTemplate::isObject(const std::string& obj) {
  for(auto &o : structure_.display_objects) {
    if(o.name == obj) {
      return true;
    }
  }
  return false;
}

bool AffordanceTemplate::isWaypoint(const std::string& wp) {
  return (!isObject(wp) && !isToolPointFrame(wp) && !isControlPointFrame(wp) && !isEEFrame(wp));
}

bool AffordanceTemplate::isToolPointFrame(const std::string& tp) {
  return tp.find("/tp")!=std::string::npos;
}

bool AffordanceTemplate::isControlPointFrame(const std::string& cp) {
  return cp.find("/cp")!=std::string::npos;
}

bool AffordanceTemplate::isEEFrame(const std::string& ee) {
  return ee.find("/ee")!=std::string::npos;
}

bool AffordanceTemplate::hasObjectFrame(std::string obj) {
  return isObject(obj) && (frame_store_.find(obj) != std::end(frame_store_)); 
}

bool AffordanceTemplate::hasWaypointFrame(std::string wp) {
  return isWaypoint(wp) && (frame_store_.find(wp) != std::end(frame_store_)); 
}

bool AffordanceTemplate::hasToolPointFrame(std::string tp) {
  return isToolPointFrame(tp) && (frame_store_.find(tp) != std::end(frame_store_)); 
}

bool AffordanceTemplate::hasControlPointFrame(std::string cp) {
  return isControlPointFrame(cp) && (frame_store_.find(cp) != std::end(frame_store_)); 
}

bool AffordanceTemplate::hasFrame(std::string frame_name) {
  return frame_store_.find(frame_name) != std::end(frame_store_); 
}

void AffordanceTemplate::setFrame(std::string frame_name, geometry_msgs::PoseStamped ps) {

  if(!hasFrame(frame_name))
    frame_store_[frame_name] = FrameInfo(frame_name, ps);
  else
    frame_store_[frame_name].second = ps;
}

bool AffordanceTemplate::hasControls(std::string name) {
  return hasObjectFrame(name) || hasWaypointFrame(name) || hasToolPointFrame(name);
}

std::string AffordanceTemplate::getWaypointFrame(std::string frame) {
  std::size_t pos = frame.find("/");
  return frame.substr(0,pos);  
} 

int AffordanceTemplate::getNumWaypoints(const std::string traj_name, const int ee_id) {
  for(auto &traj : structure_.ee_trajectories) {
    if(traj.name == traj_name) {
      for(auto &ee_list : traj.ee_waypoint_list) {
        if(ee_list.id == ee_id) {
          return ee_list.waypoints.size();
        }
      }
    }
  }
  return 0;
}

bool AffordanceTemplate::computePathSequence(std::string traj_name, 
                                             int ee_id, int idx, int steps, 
                                             bool direct, bool backwards, 
                                             std::vector<int> &sequence_ids, 
                                             int &next_path_idx)
{ 
  ROS_WARN_STREAM("computing path seq for traj "<<traj_name<<" for ee "<<ee_id<<" at index "<<idx<<" with "<<steps<<" steps and will "<<(direct?"be ":"not be ")<<"direct motion");
  sequence_ids.clear();
  if (direct) {
    sequence_ids.push_back(steps-1);
    next_path_idx = steps-1;
    return true;
  } else if (steps == 0) {
    sequence_ids.push_back(idx);
    next_path_idx = idx;
    return true;
  } else {
    int max_idx = getNumWaypoints(traj_name, ee_id)-1;
    int cap = max_idx+1;
    int inc = 1;
    if(backwards) {
      inc = -1;
    }
    if(idx == -1) {
      if(backwards) {
        sequence_ids.push_back(max_idx);
        idx = max_idx;
      } else {
        sequence_ids.push_back(0);
        idx = 0;
      }
      steps--;
    }
    if (steps == -1) {
      sequence_ids.push_back(idx);
    }
    for(int s=0; s<steps; s++) {
      idx += inc;
      if(idx == -1) {
        if(backwards) {
          idx = max_idx;
        } else {
          idx = 0;
        }
      } else {
        idx = idx%cap;
      }
      sequence_ids.push_back(idx);      
    }
    next_path_idx = sequence_ids.back();
  }
  return true;
}

void AffordanceTemplate::planRequest(const PlanGoalConstPtr& goal)
{
  ROS_INFO("[AffordanceTemplate::planRequest] request to plan %sfor \"%s\" trajectory...", (goal->execute?"and execute ":""), goal->trajectory.c_str());

  PlanResult result;
  PlanFeedback planning;

  planning.progress = 1;
  planning_server_.publishFeedback(planning);

  robot_interface_->getPlanner()->resetAnimation(true);

  for (auto ee : goal->groups) {

    ++planning.progress;
    planning_server_.publishFeedback(planning);

    plan_status_[goal->trajectory][ee].plan_valid = false;
    plan_status_[goal->trajectory][ee].exec_valid = false;
    plan_status_[goal->trajectory][ee].direct     = goal->direct;
    plan_status_[goal->trajectory][ee].backwards  = goal->backwards;
    plan_status_[goal->trajectory][ee].sequence_ids.clear();
    plan_status_[goal->trajectory][ee].sequence_poses.clear();
    
    bool skip = true;
    // get our waypoints for this trajectory so we can get the EE pose IDs
    int ee_id = robot_interface_->getEEID(ee);
    std::vector<affordance_template_object::EndEffectorWaypoint> wp_vec;
    for(auto &traj : structure_.ee_trajectories) {
      if(traj.name == goal->trajectory) {
        for(auto &ee_list : traj.ee_waypoint_list) {
          if(ee_list.id == ee_id) {
            wp_vec = ee_list.waypoints;
            skip = false;
            break;
          }
        }
      }
    }

    // make sure the ee_id (thus, the EE) is in the WP list, if not move onto next EE
    if (skip)
      continue;

    int max_idx = getNumWaypoints(goal->trajectory, ee_id);
    int current_idx = plan_status_[goal->trajectory][ee].current_idx;
    std::string manipulator_name = robot_interface_->getManipulator(ee);
    std::map<std::string, std::vector<geometry_msgs::PoseStamped> > goals;
    std::map<std::string, planner_interface::PlanningGoal> goals_full;

    // make sure it matches our max idx number because that is max num waypoints
    if (wp_vec.size() != max_idx) {
      ROS_ERROR("[AffordanceTemplate::planRequest] waypoint vector size does not match up!!");
      planning.progress = -1;
      planning_server_.publishFeedback(planning);
      result.succeeded = false;
      planning_server_.setSucceeded(result);
      return;
    }

    // get list of EE pose names related to ID
    std::map<int, std::string> ee_pose_map = robot_interface_->getEEPoseNameMap(ee);

    // find our sequence IDs first - will use these to loop on
    if (!computePathSequence(goal->trajectory, ee_id, 
                            plan_status_[goal->trajectory][ee].current_idx,
                            goal->steps, goal->direct,
                            plan_status_[goal->trajectory][ee].backwards, 
                            plan_status_[goal->trajectory][ee].sequence_ids, 
                            plan_status_[goal->trajectory][ee].goal_idx)) {
      ROS_ERROR("[AffordanceTemplate::planRequest] failed to get path sequence!!");
      planning.progress = -1;
      planning_server_.publishFeedback(planning);
      result.succeeded = false;
      planning_server_.setSucceeded(result);
      return;
    }

    // use to set the seed states during planning
    sensor_msgs::JointState manipulator_state;
    sensor_msgs::JointState gripper_state;
    std::map<std::string, sensor_msgs::JointState> group_start_states;

    // now loop through waypoints setting new start state to the last planned joint values
    for (auto plan_seq : plan_status_[goal->trajectory][ee].sequence_ids) {

      ++planning.progress;
      planning_server_.publishFeedback(planning);

      std::string next_path_str = createWaypointID(ee_id, plan_seq);
      
      std::string wp_frame_name = next_path_str;
      std::string tp_frame_name = getToolPointFrameName(wp_frame_name);       
      std::string cp_frame_name = getControlPointFrameName(wp_frame_name);
      std::string ee_frame_name = getEEFrameName(wp_frame_name);
      
      // get helper transforms
      tf::Transform wpTee, eeTcp, wpTtp, cpTtp,wpTcp;
      tf::poseMsgToTF(frame_store_[ee_frame_name].second.pose,wpTee);
      tf::poseMsgToTF(frame_store_[cp_frame_name].second.pose,eeTcp);
      tf::poseMsgToTF(frame_store_[tp_frame_name].second.pose,wpTtp); 
      
      // create goal
      planner_interface::PlanningGoal pg;
      goals[manipulator_name].clear();

      geometry_msgs::PoseStamped pt = frame_store_[tp_frame_name].second;    
      goals[manipulator_name].push_back(pt);
      plan_status_[goal->trajectory][ee].sequence_poses.push_back(pt);
      pg.goal = pt;
      
      // transform frame offset back to EE frame
      wpTcp = wpTee*eeTcp;
      cpTtp = wpTcp.inverse()*wpTtp;
      geometry_msgs::Pose tp_offset;
      tf::poseTFToMsg(cpTtp, tp_offset);
      pg.offset = tp_offset;
      robot_interface_->getPlanner()->setToolOffset(manipulator_name, pg.offset);

      // get the rest of the waypoint infor for goal
      affordance_template_object::EndEffectorWaypoint wp;
      if(!getWaypoint(goal->trajectory, ee_id, plan_seq, wp)) {
        ROS_ERROR("[AffordanceTemplate::planRequest] waypoint vector size does not match up!!");
        planning.progress = -1;
        planning_server_.publishFeedback(planning);
        result.succeeded = false;
        planning_server_.setSucceeded(result);
        return;
      }

      pg.task_compatibility = taskCompatibilityToPoseMsg(wp.task_compatibility);  
      pg.conditioning_metric = wp.conditioning_metric;
      pg.type = stringToPlannerType(wp.planner_type);
      
      for(int i = 0; i < 3; ++i)  {
        for(int j = 0; j < 2; ++j) {
          pg.tolerance_bounds[i][j] = wp.bounds.position[i][j];
          pg.tolerance_bounds[i+3][j] = wp.bounds.orientation[i][j];
        }
      }

      ROS_INFO("[AffordanceTemplate::planRequest] configuring plan goal for waypoint %s [%d/%d] for %s[%d] on manipulator %s, type: %s", next_path_str.c_str(), plan_seq+1, max_idx, ee.c_str(), ee_id, manipulator_name.c_str(), wp.planner_type.c_str());

      // give start states
      if (gripper_state.position.size()) 
        group_start_states[ee] = gripper_state;
      if (manipulator_state.position.size()) 
        group_start_states[manipulator_name] = manipulator_state;

      // do plan
      goals_full[manipulator_name] = pg;
      if (robot_interface_->getPlanner()->plan(goals_full, false, false, group_start_states)) {

        ROS_INFO("[AffordanceTemplate::planRequest] planning for %s succeeded", next_path_str.c_str());
        
        ++planning.progress;
        planning_server_.publishFeedback(planning);

        plan_status_[goal->trajectory][ee].plan_valid = true;
        
        moveit::planning_interface::MoveGroup::Plan plan;
        if (!robot_interface_->getPlanner()->getPlan(manipulator_name, plan)) {
          ROS_FATAL("[AffordanceTemplate::planRequest] couldn't find stored plan for %s waypoint!! this shouldn't happen, something is wrong!.", next_path_str.c_str());
          planning.progress = -1;
          planning_server_.publishFeedback(planning);
          result.succeeded = false;
          planning_server_.setSucceeded(result);
          return;
        }

        ContinuousPlan cp;
        cp.step = plan_seq;
        cp.group = manipulator_name;
        cp.type = PlanningGroup::MANIPULATOR;
        cp.start_state = manipulator_state;
        cp.plan = plan;
        setContinuousPlan(goal->trajectory, cp);

        // set the start state for ee planning
        manipulator_state.header = plan.trajectory_.joint_trajectory.header;
        manipulator_state.name = plan.trajectory_.joint_trajectory.joint_names;
        if (plan.trajectory_.joint_trajectory.points.size()) {
          manipulator_state.position = plan.trajectory_.joint_trajectory.points.back().positions;
          manipulator_state.velocity = plan.trajectory_.joint_trajectory.points.back().velocities;
          manipulator_state.effort = plan.trajectory_.joint_trajectory.points.back().effort;
          group_start_states[manipulator_name] = manipulator_state;
        } else {
          ROS_ERROR("[AffordanceTemplate::planRequest] the resulting plan generated 0 joint trajectory points!!");
        }
        
        // find and add EE joint state to goal
        if (ee_pose_map.find(wp_vec[plan_seq].ee_pose) == ee_pose_map.end()) {
          ROS_WARN("[AffordanceTemplate::planRequest] couldn't find EE Pose ID %d in robot interface map!!", plan_seq);
        } else {
          ++planning.progress;
          planning_server_.publishFeedback(planning);

          ROS_INFO("[AffordanceTemplate::planRequest] setting EE goal pose to %s", ee_pose_map[wp_vec[plan_seq].ee_pose].c_str());
          sensor_msgs::JointState ee_js;
          try {
            if (!robot_interface_->getPlanner()->getRDFModel()->getGroupState( ee, ee_pose_map[wp_vec[plan_seq].ee_pose], ee_js)) { // this is the reason for the try{} block, TODO should put null pointer detection in
              ROS_ERROR("[AffordanceTemplate::planRequest] couldn't get group state!!");
              planning.progress = -1;
              planning_server_.publishFeedback(planning);
              result.succeeded = false;
              planning_server_.setSucceeded(result);
              return;
            } else {
              ++planning.progress;
              planning_server_.publishFeedback(planning);

              std::map<std::string, std::vector<sensor_msgs::JointState> > ee_goals;
              ee_goals[ee].push_back(ee_js);

              if (!robot_interface_->getPlanner()->planJointPath( ee_goals, false, false, group_start_states)) {
                ROS_ERROR("[AffordanceTemplate::planRequest] couldn't plan for gripper joint states!!");
                planning.progress = -1;
                planning_server_.publishFeedback(planning);
                result.succeeded = false;
                planning_server_.setSucceeded(result);
                return;
              }

              if (!robot_interface_->getPlanner()->getPlan(ee, plan)) {
                ROS_FATAL("[AffordanceTemplate::planRequest] couldn't find stored plan for %s waypoint!! this shouldn't happen, something is wrong!.", next_path_str.c_str());
                planning.progress = -1;
                planning_server_.publishFeedback(planning);
                result.succeeded = false;
                planning_server_.setSucceeded(result);
                return;
              }

              cp.step = plan_seq;
              cp.group = ee;
              cp.type = PlanningGroup::EE;
              cp.start_state = manipulator_state;
              cp.plan = plan;
              setContinuousPlan(goal->trajectory, cp);

              // doing this lets us append the grasp pose without having the arm go back to init start state
              ContinuousPlan p;
              getContinuousPlan( goal->trajectory, plan_seq, ee, PlanningGroup::EE, p);
              p.plan.trajectory_.joint_trajectory.points.push_back(plan.trajectory_.joint_trajectory.points.back());
              gripper_state.header = plan.trajectory_.joint_trajectory.header;
              gripper_state.name = plan.trajectory_.joint_trajectory.joint_names;

              if (plan.trajectory_.joint_trajectory.points.size()) {
                gripper_state.position = plan.trajectory_.joint_trajectory.points.back().positions;
                gripper_state.velocity = plan.trajectory_.joint_trajectory.points.back().velocities;
                gripper_state.effort = plan.trajectory_.joint_trajectory.points.back().effort;
                group_start_states[ee] = gripper_state;
              } else {
                ROS_ERROR("[AffordanceTemplate::planRequest] the resulting plan generated 0 joint trajectory points!!");
              }
            }
          } catch(...) {
            ROS_FATAL("[AffordanceTemplate::planRequest] couldn't get planner or RDF model -- bad pointer somewhere!!");
            planning.progress = -1;
            planning_server_.publishFeedback(planning);
            result.succeeded = false;
            planning_server_.setSucceeded(result);
            return;
          }
        }
      } else {
        ROS_ERROR("[AffordanceTemplate::planRequest] planning failed for waypoint %s", next_path_str.c_str());
        planning.progress = -1;
        planning_server_.publishFeedback(planning);
        result.succeeded = false;
        planning_server_.setSucceeded(result);
        return;
      }
    } // waypoint loop
  } // ee loop

  if (autoplay_display_)
    robot_interface_->getPlanner()->playAnimation();

  if (goal->execute) {
    ROS_INFO("[AffordanceTemplate::planRequest] planning complete. executing generated plans!");

    for (auto ee : goal->groups) {
      if ( plan_status_[goal->trajectory][ee].plan_valid) {
        if (!continuousMoveToWaypoints(goal->trajectory, ee)) {
          
          ROS_ERROR("[AffordanceTemplate::planRequest] execution of plan failed!!");
          
          continuous_plans_[goal->trajectory].clear();
          robot_interface_->getPlanner()->resetAnimation(true);
      
          return;
        }

        plan_status_[goal->trajectory][ee].current_idx = plan_status_[goal->trajectory][ee].goal_idx;
        buildTemplate(goal->trajectory);
      }
    }

    // clear out whatever plans we may have
    continuous_plans_[goal->trajectory].clear();
    robot_interface_->getPlanner()->resetAnimation(true);
  }

  ++planning.progress;
  planning_server_.publishFeedback(planning);
  
  result.succeeded = true;
  planning_server_.setSucceeded(result);
}


void AffordanceTemplate::executeRequest(const ExecuteGoalConstPtr& goal)
{
  ROS_INFO("[AffordanceTemplate::executeRequest] request to execute any stored plans...");

  ExecuteResult result;
  ExecuteFeedback exe;

  exe.progress = 1;
  execution_server_.publishFeedback(exe);
  
  for (auto ee : goal->groups) {
    if ( plan_status_[goal->trajectory][ee].plan_valid) {
      if (!continuousMoveToWaypoints(goal->trajectory, ee)) {
        ROS_ERROR("[AffordanceTemplate::executeRequest] execution of plan failed!!");
        exe.progress = -1;
        execution_server_.publishFeedback(exe);
        result.succeeded = false;
        execution_server_.setSucceeded(result);
        
        continuous_plans_[goal->trajectory].clear();
        robot_interface_->getPlanner()->resetAnimation(true);
    
        return;
      }
      plan_status_[goal->trajectory][ee].current_idx = plan_status_[goal->trajectory][ee].goal_idx;
    }
  }

  // clear out whatever plans we may have
  continuous_plans_[goal->trajectory].clear();
  robot_interface_->getPlanner()->resetAnimation(true);
  
  result.succeeded = true;
  execution_server_.setSucceeded(result);
}

bool AffordanceTemplate::continuousMoveToWaypoints(const std::string& trajectory, const std::string& ee)
{
  if (continuous_plans_.find(trajectory) == continuous_plans_.end()) {
    ROS_ERROR("[AffordanceTemplate::continuousMoveToWaypoints] no plan found for trajectory %s!!", trajectory.c_str());
    return false;
  }

  if (!plan_status_[trajectory][ee].plan_valid) {
    plan_status_[trajectory][ee].exec_valid = false;
    ROS_ERROR("[AffordanceTemplate::continuousMoveToWaypoints] EE %s in trajectory %s doesn't have a valid plan!!", ee.c_str(), trajectory.c_str());
    return false;
  }

  std::string manipulator_name = robot_interface_->getManipulator(ee);
  std::vector<std::pair<std::string, moveit::planning_interface::MoveGroup::Plan> > plans_to_exe;
  bool ee_wp = false, man_wp = false;
  for ( auto& p : continuous_plans_[trajectory]) {
    if (p.group == ee && !ee_wp) {
      plans_to_exe.push_back(std::make_pair(p.group, p.plan));
      ee_wp = true;
    }

    if (p.group == manipulator_name && !man_wp) {
      plans_to_exe.push_back(std::make_pair(p.group, p.plan));
      man_wp = true; 
    }

    if (man_wp && ee_wp) {
      if (!robot_interface_->getPlanner()->executeContinuousPlans(plans_to_exe)) {
        ROS_ERROR("[AffordanceTemplate::continuousMoveToWaypoints] execution failed");
        return false;
      }

      man_wp = ee_wp = false;
      plans_to_exe.clear();
    }
  }

  plan_status_[trajectory][ee].current_idx = plan_status_[trajectory][ee].goal_idx;
  plan_status_[trajectory][ee].plan_valid = false;
  plan_status_[trajectory][ee].exec_valid = true;

  ROS_INFO("AffordanceTemplate::moveToWaypoints() execution of %s to %d succeeded, traj=%s!!", ee.c_str(), plan_status_[trajectory][ee].current_idx, trajectory.c_str());

  buildTemplate(trajectory);
  
  return true;
}

// list of ee waypoints to move to, return true if all waypoints were valid
bool AffordanceTemplate::moveToWaypoints(const std::vector<std::string>& ee_names) 
{
  // ROS_INFO("AffordanceTemplate::moveToWaypoints() with size %d", ee_names.size());
  std::vector<std::string> valid_ee_plans;
  std::vector<std::string> m_names;
  for(auto ee: ee_names) {
    if (plan_status_[current_trajectory_][ee].plan_valid) {
      valid_ee_plans.push_back(ee);
      m_names.push_back(robot_interface_->getManipulator(ee));
    } else {
      plan_status_[current_trajectory_][ee].exec_valid = false;
    }
  }
  if(robot_interface_->getPlanner()->executePlans(m_names)) {
    ROS_INFO("AffordanceTemplate::moveToWaypoints() -- execution succeeded");
    for(auto ee: valid_ee_plans) {
      plan_status_[current_trajectory_][ee].current_idx = plan_status_[current_trajectory_][ee].goal_idx;
      plan_status_[current_trajectory_][ee].plan_valid = false;
      plan_status_[current_trajectory_][ee].exec_valid = true;
    }
    return true;
  }
  ROS_WARN("AffordanceTemplate::moveToWaypoints() -- execution failed");
  return false;
}

bool AffordanceTemplate::getPoseFromFrameStore(const std::string &frame, geometry_msgs::PoseStamped &ps) 
{
  auto search = frame_store_.find(frame);
  if(search != frame_store_.end()) {
    ps = frame_store_[frame].second;
    return true;
  }
  return false;
}

void AffordanceTemplate::run()
{
  ros::Rate loop_rate(loop_rate_);
  tf::Transform transform;
  FrameInfo fi;
  ros::Time t;
  ros::Time start_t = ros::Time::now();
  bool applied = false;
  
  ROS_DEBUG("AffordanceTemplate::run() -- spinning...");
  while(running_ && ros::ok()) {
    
    t = ros::Time::now();
    mutex_.lock();
    for(auto f: frame_store_)  {
      fi = f.second;
      fi.second.header.stamp = t;
      tf::poseMsgToTF(fi.second.pose, transform);
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, t, fi.second.header.frame_id, fi.first));
    
      if(isObject(f.first) || isWaypoint(f.first) )
        server_->setPose(f.first, fi.second.pose);
    }
    mutex_.unlock();

    // apply changes every ~10 seconds so we don't overload the IM server
    if ((int)(ros::Time::now()-start_t).toSec() % 10 == 0 && !applied) {
      server_->applyChanges();
      applied = true;
    } else if (applied && (int)(ros::Time::now()-start_t).toSec()%10 != 0) {
      applied = false;
    }
    
    loop_rate.sleep();
  }
  ROS_DEBUG("AffordanceTemplate::run() -- leaving spin thread. template must be shutting down...");
}

void AffordanceTemplate::stop()
{
  ROS_INFO("[AffordanceTemplate::stop] %s being asked to stop..", name_.c_str());
  running_ = false;
  mutex_.lock();
  frame_store_.clear();
  mutex_.unlock();
  removeAllMarkers();
}

bool AffordanceTemplate::setObjectScaling(const std::string& key, double scale_factor, double ee_scale_factor)
{ 
  ROS_DEBUG("[AffordanceTemplate::setObjectScaling] setting object %s scaling to %g, %g", key.c_str(), scale_factor, ee_scale_factor);
  object_scale_factor_[key] = scale_factor;
  ee_scale_factor_[key] = ee_scale_factor;
  removeInteractiveMarker(key);
  return buildTemplate();
}

bool AffordanceTemplate::setObjectPose(const DisplayObjectInfo& obj)
{
  ROS_INFO("[AffordanceTemplate::setObjectPose] setting pose for object %s in template %s:%d", obj.name.c_str(), obj.type.c_str(), obj.id);
  
  for (auto& d : structure_.display_objects) {
    std::string obj_name = obj.name + ":" + std::to_string(obj.id);
    if (d.name == obj_name) {
      ROS_DEBUG("[AffordanceTemplate::setObjectPose] matched object %s in frame: %s", obj_name.c_str(), obj.stamped_pose.header.frame_id.c_str());
      geometry_msgs::PoseStamped ps;
      try {
        tf_listener_.waitForTransform(frame_store_[obj_name].second.header.frame_id, obj.stamped_pose.header.frame_id, obj.stamped_pose.header.stamp, ros::Duration(3.0));
        tf_listener_.transformPose(frame_store_[obj_name].second.header.frame_id, obj.stamped_pose, ps);
        frame_store_[obj_name].second = ps;
        server_->setPose(obj_name, ps.pose);
        updatePoseInStructure(obj_name, ps.pose);
      } catch(tf::TransformException ex) {
        ROS_ERROR("[AffordanceTemplate::setObjectPose] trouble transforming pose from %s to %s. TransformException: %s",frame_store_[obj_name].second.header.frame_id.c_str(), obj.stamped_pose.header.frame_id.c_str(), ex.what());
        return false;
      }
      break;
    }
  }
  server_->applyChanges();
  return true;
}

bool AffordanceTemplate::getContinuousPlan(const std::string& trajectory, const int step, const std::string& group, const PlanningGroup type, ContinuousPlan& plan)
{
  if (continuous_plans_.find(trajectory) == continuous_plans_.end()) {
    ROS_WARN("[AffordanceTemplate::getContinuousPlan] no plan found for trajectory %s", trajectory.c_str());
    return false;
  }
  
  if (step < 0)
    return false;
  
  for ( auto& p : continuous_plans_[trajectory]) {
    if (p.step == step && p.group == group && p.type == type) {
      plan = p;
      return true;
    }
  }

  return false;
}

void AffordanceTemplate::setContinuousPlan(const std::string& trajectory, const ContinuousPlan& plan)
{
  // ROS_WARN("trajectory %s plan step is %d for group %s", trajectory.c_str(), plan.step, plan.group.c_str());
  if (continuous_plans_.find(trajectory) == continuous_plans_.end()) {
    continuous_plans_[trajectory].push_back(plan);
  } else {
    for (auto& cp : continuous_plans_[trajectory]) {
      if (cp.step == plan.step && cp.group == plan.group && cp.type == plan.type) {
        cp.start_state = plan.start_state;
        cp.plan = plan.plan;
        return;
      }
    }
    // else not in there yet
    continuous_plans_[trajectory].push_back(plan);
  }
  ROS_INFO("[AffordanceTemplate::setContinuousPlan] there are now %d plans", (int)continuous_plans_[trajectory].size());
}
