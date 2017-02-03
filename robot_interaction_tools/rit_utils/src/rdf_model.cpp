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

#include <rit_utils/rdf_model.h>

using namespace rit_utils;

RDFModel::RDFModel(const std::string& _robot_name) :
  robot_name_(_robot_name)
{
  ROS_INFO("RDFModel -- Creating utility class.");

  GroupType type = CARTESIAN;
}
RDFModel::~RDFModel(){}

bool RDFModel::init(const std::string& srdf, const std::string& urdf)
{
  ROS_INFO("[RDFModel::init] Initializing model using SRDF %s and URDF %s", srdf.c_str(), urdf.c_str());

  if (!createModels(srdf, urdf))
  {
    ROS_FATAL("[RDFModel::init] Failed to create models!");
    return false;
  }

  ROS_INFO("[RDFModel::init] Successfully initialized SRDF Model");
  return true;
}

bool RDFModel::createModels(const std::string& srdf, const std::string& urdf)
{ /*
  ros::NodeHandle nh;
  ROS_INFO("[RDFModel::createModels] Creating Robot Model from URDF from %s....", urdf.c_str());
  ROS_ASSERT(nh.hasParam(urdf));
  std::string robot_xml;
  nh.getParam(urdf, robot_xml);
  boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDF(robot_xml);
  ROS_ASSERT(urdf_model != NULL);
  ROS_INFO("[RDFModel::createModels] URDF created");

  ROS_INFO("[RDFModel::createModels] Creating Robot Model from SRDF from %s....", srdf.c_str());
  ROS_ASSERT(nh.hasParam(srdf));
  std::string semantic_xml;
  nh.getParam(srdf, semantic_xml);
  srdf::Model srdf_model = srdf::Model();
  if (!srdf_model.initString(*urdf_model, semantic_xml)) return false;
  ROS_INFO("[RDFModel::createModels] SRDF created");

  if (!parseRDF(srdf_model, urdf_model))
  {
    ROS_FATAL("[RDFModel::init] Failed to parse SRDF!");
    return false;
  }
*/
  return true;
}

bool RDFModel::parseRDF(const srdf::Model &srdf, const boost::shared_ptr<urdf::ModelInterface> &urdf)
{
  // SRDF
  ROS_INFO("[RDFModel::parseRDF] Parsing SRDF file.");
  robot_name_          = srdf.getName();
  groups_              = srdf.getGroups();
  group_states_        = srdf.getGroupStates();
  end_effectors_       = srdf.getEndEffectors();
  collision_spheres_   = srdf.getLinkSphereApproximations();
  passive_joints_      = srdf.getPassiveJoints();
  virtual_joints_      = srdf.getVirtualJoints();
  disabled_collisions_ = srdf.getDisabledCollisionPairs();
  
  ROS_INFO("[RDFModel::parseRDF] parsing URDF file into KDL Tree");
  if (!kdl_parser::treeFromUrdfModel(*urdf, tree)) {
    ROS_FATAL("[RDFModel::parseRDF] failed to extract kdl tree from xml robot description!");
    return false;
  }

  // URDF link info
  urdf->getLinks(urdf_links_);
  for (auto& i : urdf_links_) {
    std::string parent, child;
    parent = i->getParent() == NULL ? "" : i->getParent()->name;
    child = i->child_links.size() == 0 ? "" : i->child_links.front()->name;
    ROS_DEBUG_STREAM("[RDFModel::parseRDF] link "<<i->name<<" has parent: "<<parent<<" and child: "<<child); 
    robot_links_.insert(std::make_pair(i->name, std::make_pair(parent, child)));

    // cast a geometry object to a mesh object
    // mesh inherits geometry
    // so casting a base class to a subclass which is backward
    if (i->visual != NULL) {
      if (i->visual->geometry->type == urdf::Geometry::MESH) {
        urdf::Mesh *m = dynamic_cast<urdf::Mesh*>(i->visual->geometry.get());
        ROS_DEBUG("[RDF:::parseRDF] link %s has mesh file %s", i->name.c_str(), m->filename.c_str());
        mesh_links_[i->name] = m->filename;
      }
    }
  }

  // URDF joint info
  urdf_joints_ = urdf->joints_;
  for (auto &i : urdf_joints_) {
    if (i.second->type == urdf::Joint::FIXED) {
      ROS_DEBUG_STREAM("[RDFModel::parseRDF] found fixed joint "<<i.second->name<<" with child "<<i.second->child_link_name<<" and parent "<<i.second->parent_link_name);
      fixed_joints_[i.second->name] = std::make_pair(i.second->parent_link_name, i.second->child_link_name);
    } else {
      joint_velocities_[i.second->name] = std::abs(i.second->limits->velocity);  //required in URDF
      if ( i.second->type != urdf::Joint::CONTINUOUS ) {
        if (i.second->safety)
          joint_limits_[i.second->name] = std::make_pair(i.second->safety->soft_lower_limit, i.second->safety->soft_upper_limit);
        else 
          joint_limits_[i.second->name] = std::make_pair(i.second->limits->lower, i.second->limits->upper);    
      } else {
        joint_limits_[i.second->name] = std::make_pair(-std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
      }
      
      ROS_DEBUG_STREAM("[RDFModel::parseRDF] joint "<<i.second->name<<" using limits "<<joint_limits_[i.second->name].first<<" <--> "<<joint_limits_[i.second->name].second);
    }
  }

  // get all tip links
  bool found_parent;
  for (auto& l : urdf_links_) {  
    found_parent = false;
    for (auto& j : urdf_joints_) {
      if (j.second->parent_link_name == l->name) {
        found_parent = true;
      }
    }
    if(!found_parent) {
      urdf_tips_.push_back(l->name);
    }
  }

  // create full link list from base->tip if chain exists
  // otherwise create list from joint or link names that
  // show up in group object
  for (auto &g : groups_) {

    ROS_DEBUG("[RDFModel::parseRDF] SRDF parsing for group[%s]", g.name_.c_str()); 

    std::vector<std::string> links;
    if (!getKDLChain(g.name_, links)) {

      if (g.links_.size()) {

        for (auto &i : g.links_)
          links.push_back(i);
      } else if (g.joints_.size()) {

        for (auto &i : g.joints_)
          links.push_back(urdf_joints_[i]->child_link_name);
      }
    }

    for(auto l : links)
      ROS_DEBUG("[RDFModel::parseRDF] group[%s] has link %s", g.name_.c_str(), l.c_str());

    for (auto &i : urdf_links_) {
      for (auto &l : g.links_) {
        std::vector<std::string> new_links;
        if (getChain(l, i->name, new_links)) {
          for (auto &k : new_links) {
            if(std::find(links.begin(), links.end(), k)==links.end()) {
              links.push_back(k);
              ROS_DEBUG("[RDFModel::parseRDF] group %s getting link %s for full group list", i->name.c_str(), k.c_str());
            }
          }
        }
      }
    }
    full_group_links_[g.name_] = links;

    // find full joint links
    for (auto &i : links)
    {
      std::string jnt;
      if(findLinkJoint(i, jnt)) {
        ROS_DEBUG("[RDFModel::parseRDF] group[%s] link[%s] has joint %s ", g.name_.c_str(), i.c_str(), jnt.c_str()); 
        full_group_joints_[g.name_].push_back(jnt);
      }
    }
  }

  return true;
}

bool RDFModel::findGroup(const std::string& group_name, srdf::Model::Group& group)
{
  for (auto &i : groups_)
  {
    if (i.name_ == group_name)
    {
      group = i;
      return true;
    }
  }

  ROS_ERROR("[RDFModel::findGroup] Group %s does not exsist!", group_name.c_str());
  return false;
}

boost::shared_ptr<urdf::Link> RDFModel::getLink(const std::string& link)
{
  for (auto &i : urdf_links_)
  {
    if (i->name == link)
      return i;
  }
}

boost::shared_ptr<urdf::Joint> RDFModel::getJoint(const std::string& joint)
{
  for (auto &i : urdf_joints_)
  {
    if (i.second->name == joint)
      return i.second;
  }
}

bool RDFModel::getKDLLimits(const std::string& group_name, KDL::JntArray& lower, KDL::JntArray& upper) {

  std::vector<double> ll, ul;

  if (!getJointLimits(group_name,ll,ul))
    return false;
  
  lower.resize(ll.size());
  upper.resize(ul.size());

  for (uint i=0; i < ll.size(); i++) {
    lower(i)=ll[i];
    upper(i)=ul[i];
  }

  return true;
}

bool RDFModel::getJointLimits(const std::string& group_name, std::vector<double>& lower_limits, std::vector<double>& upper_limits)
{
  KDL::Chain chain;
  if (!getKDLChain(group_name, chain))
    return false;

  ROS_INFO("[RDFModel::getJointLimits] getting joint limits for %s with chain links size %d", group_name.c_str(), chain.getNrOfJoints());

  lower_limits.clear();
  upper_limits.clear();

  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {

    std::string joint = chain.segments[i].getJoint().getName();
    if (joint_limits_.find(joint) != joint_limits_.end()) {
      
      ROS_DEBUG("[RDFModel::getJointLimits] joint: %s has limits %g -- %g", joint.c_str(), joint_limits_[joint].first, joint_limits_[joint].second);
      lower_limits.push_back(joint_limits_[joint].first);
      upper_limits.push_back(joint_limits_[joint].second);
    }
  }

  return true;
}

bool RDFModel::getJointVelocities(const std::string& group_name, std::vector<double>& velocities)
{
  KDL::Chain chain;
  if (!getKDLChain(group_name, chain))
    return false;

  velocities.clear();

  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {

    std::string joint = chain.segments[i].getJoint().getName();
    if (joint_velocities_.find(joint) != joint_velocities_.end()) {
    
      ROS_DEBUG("[RDFModel::getJointVelocities] joint %s has velocity %g ", joint.c_str(), joint_velocities_[joint]);
      velocities.push_back(joint_velocities_[joint]);
    }
  }

  // for(auto link : chain_links)
  // {
  //   std::string joint;
  //   if (findLinkJoint(link, joint))
  //   {
  //     if (joint_velocities_.find(joint) != joint_velocities_.end())
  //     {
  //       ROS_DEBUG("[RDFModel::getJointVelocities] joint %s has velocity %g ", joint.c_str(), joint_velocities_[joint]);
  //       velocities.push_back(joint_velocities_[joint]);
  //     }
  //   }
  // }

  // std::reverse(velocities.begin(),velocities.end());

  return true;
}

bool RDFModel::findLinkJoint(const std::string& link, std::string& joint)
{
  if ((tree.getSegments()).find(link) != (tree.getSegments()).end()) {
    joint = tree.getSegments().at(link).segment.getJoint().getName();
    ROS_DEBUG("[RDFModel::findLinkJoint] found joint %s for link %s", joint.c_str(), link.c_str());
    return true;
  } else {
    ROS_WARN("[RDFModel::findLinkJoint] no joint for link %s.", link.c_str());
    return false;
  }
}

std::vector<std::string> RDFModel::getGroups()
{
  std::vector<std::string> groups;
  for (auto &i : groups_)
    groups.push_back(i.name_);
  return groups;
}

bool RDFModel::getLinks(const std::string& group, std::vector<std::string>& links)
{
  links.clear();

  srdf::Model::Group g;
  if (!findGroup(group, g))
    return false;

  for (auto &j : g.links_)
    links.push_back(j);

  return true;
}

bool RDFModel::getJoints(const std::string& group, std::vector<std::string>& joints, bool full_joints, bool fixed)
{
  joints.clear();

  srdf::Model::Group g;
  if (!findGroup(group, g)) {
    return false;
  }

  if(full_joints) {
    
    KDL::Chain chain;
    getKDLChain(group, chain); 
    for(auto seg : chain.segments)
      {
        std::string joint = seg.getJoint().getName();
        if(fixed || !isJointFixed(joint)) 
          joints.push_back(joint);
      }
  } else {
    for (auto &j : g.joints_) {
      joints.push_back(j);
    }
  }
  return true;
}

bool RDFModel::isJointFixed(const std::string &name) {

  if (urdf_joints_.find(name) != urdf_joints_.end()) {
     return urdf_joints_[name]->type == urdf::Joint::FIXED;
  } else {
    ROS_DEBUG("RDFModel::isJointFixed() no joint %s found", name.c_str());
    return false;
  }
}

bool RDFModel::isChain(const std::string& group) {
  
  std::string base, tip;
  return getChain(group, base, tip); 
}

bool RDFModel::isChain(const std::string& group, std::string& base, std::string& tip) {
  
  return getChain(group, base, tip); 
}

bool RDFModel::getKDLChain(const std::string& group_name, KDL::Chain& chain) {
  
  std::string base_frame, tip_frame;
  if (!getBaseLink(group_name, base_frame) || !getTipLink(group_name, tip_frame))
    return false;
  
  ROS_INFO("[RDFModel::getKDLChain] getting KDL chain from %s to %s",base_frame.c_str(),tip_frame.c_str());
  
  if(!tree.getChain(base_frame, tip_frame, chain)) {
    ROS_WARN("[RDFModel::getKDLChain] Couldn't find KDL chain %s to %s",base_frame.c_str(),tip_frame.c_str());
    return false;
  }
  
  return true;
}

bool RDFModel::getKDLChain(const std::string& group_name, std::vector<std::string>& chain_links) {
  
  KDL::Chain chain;
  if(!getKDLChain(group_name, chain)) {
    ROS_WARN("[RDFModel::getKDLChain] Couldn't find KDL chain for %s",group_name.c_str());
    return false;
  }

  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
    chain_links.push_back(chain.segments[i].getName());    

  return true;
}

bool RDFModel::getChain(const std::string& group, std::string& base, std::string& tip) {
  
  srdf::Model::Group g;
  if (!findGroup(group, g)) {
    ROS_ERROR("[RDFModel::getChain] can't find group %s", group.c_str());
    return false;
  } else if (g.chains_.size() == 0) {
    ROS_DEBUG("[RDFModel::getChain] group %s chain size is 0!!", group.c_str());
    return false;
  }

  base = g.chains_.front().first;
  tip = g.chains_.front().second;

  return true;
}

bool RDFModel::getChain(std::string base, std::string tip, std::vector<std::string>& links) { //FIXME? breaks if chain longer than joints listed 
  
  links.push_back(tip);
  std::string p = getParentLink(tip);  
  
  if (p == base) { 
    links.push_back(base);
    return true;
  } else if (p == "") {
    return false;
  } else {
    return getChain(base, p, links);
  }
}

bool RDFModel::getBaseLink(const std::string& group, std::string& link) {
  
  srdf::Model::Group g;
  if (!findGroup(group, g)) {
    return false;
  }

  if (!g.chains_.size()) {
    if(!g.links_.size()) {
      return false;
    }
    link = g.links_[0];
    return true;
  }
  link = g.chains_.front().first;

  ROS_DEBUG("[RDFModel::getBaseLink] base link is %s", link.c_str());

  return true;
}

bool RDFModel::getTipLink(const std::string& group, std::string& link)
{
  srdf::Model::Group g;
  if (!findGroup(group, g)) {
    return false;
  }

  try {
    if(g.chains_.size() == 0) {
      return false;
    }
    link = g.chains_.front().second;
  } catch(...) {
    ROS_WARN("RDFModel::getTipLink() -- failed to get tip link for group: %s", group.c_str());
    return false;
  }

  ROS_DEBUG("[RDFModel::getTipLink] tip link is %s", link.c_str());

  return true;
}

bool RDFModel::getSubgroup(const std::string& group, std::vector<std::string>& subgroup)
{
  subgroup.clear();

  srdf::Model::Group g;
  if (!findGroup(group, g))
    return false;

  for (auto &i : g.subgroups_)
    subgroup.push_back(i.c_str());

  return true;
}

std::vector<std::string> RDFModel::getEndEffectors()
{
  std::vector<std::string> ee;

  for (auto &i : end_effectors_)
    ee.push_back(i.name_);

  return ee;
}

bool RDFModel::getEndEffector(const std::string& group, std::string& ee)
{
  for (auto &i : end_effectors_)
  {
    if (group == i.parent_group_)
    {
      ee = i.name_;
      return true;
    }
  }
  return false;
}

bool RDFModel::getEndEffectorPlanningGroup(const std::string& ee, std::string& group)
{
  for (auto &i : end_effectors_)
  {
    if (ee == i.name_) {
      group = i.component_group_;
      return true;
    }
  }
  return false;
}


bool RDFModel::getEndEffectorParentGroup(const std::string& ee, std::string& group)
{
  for (auto &i : end_effectors_)
  {
    if (ee == i.component_group_) {
      group = i.parent_group_;
      return true;
    }
  }
  return false;
}

bool RDFModel::checkValidEndEffector(const std::string& group) {

  srdf::Model::Group g;
  if (!findGroup(group, g)) {
    ROS_INFO("RDFModel::checkValidEndEffector(%s) -- not a valid group", group.c_str());
    return false;
  }

  std::string parent_group, parent_link;
  for (auto &i : end_effectors_)
  {
    if (group == i.component_group_) {
      if(getEndEffectorParentGroup(group, parent_group)) {
        if(getEndEffectorBaseLink(group, parent_link) ) {      
          if(isChain(parent_group)) {
            ROS_INFO("RDFModel::checkValidEndEffector(%s) -- name: %s, parent: %s, link: %s", group.c_str(), i.name_.c_str(), parent_group.c_str(), parent_link.c_str());
            return true;
          } else {
            ROS_INFO("RDFModel::checkValidEndEffector(%s) -- parent: %s is not a valid chain", group.c_str(), parent_group.c_str());
            return false;            
          }
        } else {
          ROS_INFO("RDFModel::checkValidEndEffector(%s) -- doesn't have a valid parent link (needs to match parent group SRDF tip frame)", group.c_str());
          return false;
        }
      } else {
        ROS_INFO("RDFModel::checkValidEndEffector(%s) -- name: %s doesn't have valid parent group, needs to be specified in SRDF", group.c_str(),i.name_.c_str());
        return false;
      }
    }
  }
  ROS_INFO("RDFModel::checkValidEndEffector(%s) -- couldn't find \'end_effector\' tag in SRDF", group.c_str());

  return false;
}

bool RDFModel::isEndEffector(const std::string& group) {

  srdf::Model::Group g;
  if (!findGroup(group, g)) {
    return false;
  }

  std::string parent_group, parent_link;
  for (auto &i : end_effectors_)
  {
    if (group == i.component_group_) {
      if(getEndEffectorParentGroup(i.name_, parent_group)) {
        if(getEndEffectorBaseLink(i.component_group_, parent_link) ) {
          return true;
        } else {
          return false;
        }
      } else {
        return false;
      }
    }
  }
  return false;
}


bool RDFModel::getEndEffectorBaseLink(const std::string& ee, std::string& link) 
{
  for (auto &i : end_effectors_)
  {
    if (ee == i.component_group_) {
      link = i.parent_link_;
      return true;
    }
  }
  return false;
}

bool RDFModel::getGroupState(const std::string& group, const std::string& name, sensor_msgs::JointState& state)
{
  srdf::Model::Group g;
  if (!findGroup(group, g))
    return false;

  for (auto &i : group_states_)
  {
    if (name == i.name_)
    {
      for (auto &j : i.joint_values_)
      {
        state.name.push_back(j.first);
        state.position.push_back(j.second.front()); //FIXME only taking the first value if it has more than 1
      }
      return true;
    }
  }

  return false;
}

bool RDFModel::getGroupStateMap(const std::string& group, std::map<std::string, sensor_msgs::JointState> &group_states)
{
  group_states.clear();

  srdf::Model::Group g;
  if (!findGroup(group, g))
    return false;

  for (auto &i : group_states_)
  {
    if (group == i.group_)
    {
      sensor_msgs::JointState js;
      for (auto &j : i.joint_values_)
      {
        js.name.push_back(j.first);
        js.position.push_back(j.second.front()); //FIXME only taking the first value if it has more than 1
      }
      group_states[i.name_] = js;
    }
  }

  return !group_states.empty() ? true : false;
}

// check for name in urdf links
// if not there check as a joint name
// if not there either return empty string
std::string RDFModel::getParentLink(const std::string& name)
{
  if (robot_links_.find(name) != robot_links_.end()) {
    return robot_links_[name].first;
  } else if (urdf_joints_.find(name) != urdf_joints_.end()) {
    if (urdf_joints_[name]->parent_link_name.empty()) {
      ROS_WARN("[RDFModel::getParentLink] No parent link for %s.", name.c_str());
      return "";
    } else {
      return urdf_joints_[name]->parent_link_name;
    }
  } else {
    ROS_WARN("[RDFModel::getParentLink] Link %s not found in RDF model.", name.c_str());
    return "";
  }
}

// check for name in urdf links
// if not there check as a joint name
// if not there either return empty string
std::string RDFModel::getChildLink(const std::string& name)
{
  if (robot_links_.find(name) != robot_links_.end()) {
    return robot_links_[name].second;
  } else if (urdf_joints_.find(name) != urdf_joints_.end()) {
    if (urdf_joints_[name]->child_link_name.empty()) {
      ROS_WARN("[RDFModel::getChildLink] No child link for %s.", name.c_str());
      return "";
    } else {
      return urdf_joints_[name]->child_link_name;
    }
  } else {
    ROS_WARN("[RDFModel::getChildLink] Link %s not found in RDF model.", name.c_str());
    return "";
  }
}


std::string RDFModel::getLinkJoint(const std::string& name)
{
  if ((tree.getSegments()).find(name) != (tree.getSegments()).end()) {
    return tree.getSegments().at(name).segment.getJoint().getName();
  } else {
    ROS_WARN("[RDFModel::getLinkJoint] no joint for link %s.", name.c_str());
    return "";
  }
}


visualization_msgs::MarkerArray RDFModel::getLinkMarkers(const std::string& link)
{
  visualization_msgs::MarkerArray arr;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = link;
  marker.ns = link;
  marker.text = link;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.r = 0.0;
  marker.color.g = marker.color.b = marker.color.a = 1.0;
  marker.mesh_use_embedded_materials = true;
  marker.frame_locked = true;
  // make sure our visual and geometry actually exists
  boost::shared_ptr<urdf::Link> ptr = getLink(link);
  if (ptr->visual != NULL) // some links don't have a visual
  {
    if (ptr->visual->geometry != NULL)
    {  
      switch (ptr->visual->geometry->type)
      {
        case urdf::Geometry::MESH:
        {
          marker.type = visualization_msgs::Marker::MESH_RESOURCE;
          marker.mesh_resource = mesh_links_[link];
          marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
          break;
        }
        case urdf::Geometry::SPHERE:
        {
          marker.type = visualization_msgs::Marker::SPHERE;
          urdf::Sphere *s = dynamic_cast<urdf::Sphere*>(ptr->visual->geometry.get());
          marker.scale.x = marker.scale.y = marker.scale.z = s->radius;
          break;
        }
        case urdf::Geometry::CYLINDER:
        {
          marker.type = visualization_msgs::Marker::CYLINDER;
          urdf::Cylinder *c = dynamic_cast<urdf::Cylinder*>(ptr->visual->geometry.get());
          marker.scale.x = marker.scale.y = c->radius;
          marker.scale.z = c->length;
          break;
        }
        case urdf::Geometry::BOX:
        {
          marker.type = visualization_msgs::Marker::CUBE;
          urdf::Box *b = dynamic_cast<urdf::Box*>(ptr->visual->geometry.get());
          marker.scale.x = b->dim.x;
          marker.scale.y = b->dim.y;
          marker.scale.z = b->dim.z;
          break;
        }
        default:
          ROS_DEBUG("[RDFModel::getLinkMarkers] Link %s visual doesn't have a type.", link.c_str());
          return arr;
      }
    }
    else
    {
      ROS_DEBUG("[RDFModel::getLinkMarkers] Link %s does not have a visual geometry.", link.c_str());
      return arr;
    }
  }
  else
  {
    ROS_DEBUG("[RDFModel::getLinkMarkers] Link %s does not have a visual.", link.c_str());
    return arr;
  }

  marker.pose.position.x = ptr->visual->origin.position.x;
  marker.pose.position.y = ptr->visual->origin.position.y;
  marker.pose.position.z = ptr->visual->origin.position.z;
  marker.pose.orientation.x = ptr->visual->origin.rotation.x;
  marker.pose.orientation.y = ptr->visual->origin.rotation.y;
  marker.pose.orientation.z = ptr->visual->origin.rotation.z;
  marker.pose.orientation.w = ptr->visual->origin.rotation.w;

  arr.markers.push_back(marker);
  
  return arr;
}

visualization_msgs::MarkerArray RDFModel::getGroupMeshes(const std::string& group)
{
  visualization_msgs::MarkerArray array;

  srdf::Model::Group g;
  if (findGroup(group, g))
  {
    if (full_group_links_.find(group) != full_group_links_.end())
    {
      for (auto &i : full_group_links_[group])
      {
        // get mesh using method call from above
        // if it was successful (marker size > 0)
        // extract the marker and add to our own list
     
        ROS_DEBUG("[RDFModel::getGroupMeshes] Group %s trying to get mesh for link %s", group.c_str(), i.c_str());
        visualization_msgs::MarkerArray link_arr;
        link_arr = getLinkMarkers(i);
        if(link_arr.markers.size() > 0)
          array.markers.push_back(link_arr.markers.front());
      }

      for (auto i = 0; i < array.markers.size(); ++i)
        array.markers[i].id = i;
    }
    else
    {
      ROS_DEBUG("[RDFModel::getGroupMeshes] Group %s does not have an associated full group link list.", group.c_str());
      return array;
    }
  }

  return array;
}
