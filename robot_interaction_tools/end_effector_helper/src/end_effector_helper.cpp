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

#include <end_effector_helper/end_effector_helper.h>

using namespace end_effector_helper;

EndEffectorHelper::EndEffectorHelper(std::string group_name, std::string root_frame, rit_utils::RDFModel *rdf_model) :
  rdf_model_(rdf_model),
  group_name_(group_name),
  root_frame_(root_frame)
{

  js_sub_ = nh_.subscribe("/joint_states", 1, &EndEffectorHelper::jointStateCallback, this);

  setup();
}

EndEffectorHelper::~EndEffectorHelper()
{}

bool EndEffectorHelper::setup()
{

  ROS_DEBUG("Setting up EE Group %s", group_name_.c_str());
  bool ret = true;
  group_states_.clear();
  rdf_model_->getGroupStateMap(group_name_, group_states_);
  // rdf_model_->getJoints(group_name_, joints_, false, false);

  for(auto &s: group_states_) {
    // std::cout << "EndEffectorHelper::setup() -- " << s.first << std::endl;
    group_state_list_.push_back(s.first);
    visualization_msgs::MarkerArray markers;
    //std::cout << "jpos : " << s.second << std::endl;
    getMarkerArrayFromJointState(s.second, root_frame_, markers);//, color=color)
    pose_marker_map_[s.first] = markers;

    for(auto &j: s.second.name) {
      if(std::find(joints_.begin(), joints_.end(), j)==joints_.end()){
        joints_.push_back(j);
        ROS_DEBUG("  -- joint %s", j.c_str());
      }
    }
  }

  return ret;

}

void EndEffectorHelper::jointStateCallback(const sensor_msgs::JointState &msg) 
{
  current_jpos_.name.clear();
  current_jpos_.position.clear();

  int idx=0;
  for(auto &j: msg.name) {
    if(std::find(joints_.begin(), joints_.end(), j)!=joints_.end()){
      current_jpos_.name.push_back(msg.name[idx]);
      current_jpos_.position.push_back(msg.position[idx]);
    }
    idx++;
  }
  //std::cout << current_jpos_ << std::endl;
}


bool EndEffectorHelper::getMarkerArrayFromJointState(sensor_msgs::JointState jpos, std::string root_link, visualization_msgs::MarkerArray& markers) //, color=color)
{

  bool ret = true;

  std::vector<std::string> link_list;
  std::vector<std::string> root_parents;
  std::map<std::string, std::string> link_joints;
  TransformMap T_joints;
  TransformMap T_links;

  // This function is terrible, needs to be rewritten, as its probably/definitely wrong
  if(root_link=="") {
    root_link = root_frame_;
  }

  // add any joints related to group links not in current jpos to models
  rdf_model_->getLinks(group_name_, link_list);
  std::string joint;
  for (auto &l: link_list) {  

    if(rdf_model_->findLinkJoint(l, joint)) {
      //std::cout << "   joint: " << joint << std::endl;

      if (std::find(std::begin(jpos.name), std::end(jpos.name), joint)==jpos.name.end()) {
        jpos.name.push_back(joint);
        jpos.position.push_back(0);
      }
    }
  }
  // make sure the root joint is in there too
  if(rdf_model_->findLinkJoint(root_link, joint)) {
    if (std::find(std::begin(jpos.name), std::end(jpos.name), joint)==jpos.name.end()) {
      jpos.name.push_back(joint);
      jpos.position.push_back(0);
    }
  }

  // go through and get the transform at the joint rotation specified in JointState input 
  int idx = 0;
  double joint_val = 0.0;
  for (auto &joint : jpos.name) {
    
    std::string link = rdf_model_->getChildLink(joint);
    // std::cout << "child link for joint : " << joint << " is " << link << std::endl;
    if (std::find(std::begin(link_list), std::end(link_list), link)==link_list.end()) {
      link_list.push_back(link);
    } 
    link_joints[link] = joint;
    
    if (jpos.name[idx] == joint) {
      joint_val = jpos.position[idx];
      
      if(rdf_model_->getJoint(joint)->type == urdf::Joint::PRISMATIC) {
        urdf::Vector3 axis = rdf_model_->getJoint(joint)->axis;
        tf::Matrix3x3 R; 
        R.setIdentity();
        tf::Vector3 p = rit_utils::getTranslationFromJointVal(axis, joint_val);
        tf::Transform T(R, p);
        T_joints[joint] = T;
      } else if(rdf_model_->getJoint(joint)->type == urdf::Joint::FIXED) {
        ROS_DEBUG("EndEffectorHelper::getMarkerArrayFromJointState() -- found fixed joint");         
        urdf::Vector3 axis = rdf_model_->getJoint(joint)->axis;
        tf::Matrix3x3 R = rit_utils::getRotationFromJointVal(axis, joint_val);
        tf::Transform T(R, tf::Vector3(0.0, 0.0, 0.0));
        T_joints[joint] = T;
      } else if(rdf_model_->getJoint(joint)->type == urdf::Joint::REVOLUTE || rdf_model_->getJoint(joint)->type == urdf::Joint::CONTINUOUS) {
        urdf::Vector3 axis = rdf_model_->getJoint(joint)->axis;
        tf::Matrix3x3 R = rit_utils::getRotationFromJointVal(axis, joint_val);
        tf::Transform T(R, tf::Vector3(0.0, 0.0, 0.0));
        T_joints[joint] = T;
      } else {
        ROS_ERROR("EndEffectorHelper::getMarkerArrayFromJointState() -- unsupported URDF joint type: %d", (int)rdf_model_->getJoint(joint)->type);
        tf::Matrix3x3 R; 
        R.setIdentity();
        tf::Transform T(R, tf::Vector3(0.0, 0.0, 0.0));
        T_joints[joint] = T;
      }

    }
    idx++;

  }



  if (std::find(std::begin(link_list), std::end(link_list), root_link)==link_list.end()) {
    link_list.push_back(root_link);
  }  

  link_joints[root_link] = rdf_model_->getLinkJoint(root_link);
  std::string parent_link = rdf_model_->getParentLink(root_link);
  while(parent_link != "") {
    root_parents.push_back(parent_link);
    parent_link = rdf_model_->getParentLink(parent_link);   
  }
  root_parents.push_back(root_link);


  // go through and make sure we have all the links for the group.  
  // this will catch any fixed joint links, that would be skipped otherwise
  for(auto &link: link_list) {
    if (std::find(std::begin(root_parents), std::end(root_parents), link)==root_parents.end()) { 
      continue;
    }
    parent_link = rdf_model_->getParentLink(link);

    while ((std::find(std::begin(root_parents), std::end(root_parents), parent_link)==root_parents.end()) &&  
           (std::find(std::begin(link_list),    std::end(link_list),    parent_link)==link_list.end()   )) {
      if(parent_link=="") {
        break;
      }
      link_list.push_back(parent_link);
      link_joints[parent_link] = rdf_model_->getLinkJoint(parent_link);
      parent_link = rdf_model_->getParentLink(parent_link);
    } 
  }       

  idx = 0;
  for(auto &link: link_list) {
    //std::cout << "link: " << link << std::endl;

    T_links[link] = getTransformToLink(root_link, link, link_joints[link], jpos, T_joints, T_links);
    visualization_msgs::MarkerArray link_markers = rdf_model_->getLinkMarkers(link);
    for (auto &m: link_markers.markers) {

      geometry_msgs::Pose mp = m.pose;
      geometry_msgs::Pose nmp;
      tf::Transform tp, tmp;
      tf::poseMsgToTF(mp, tp);
      tmp = T_links[link]*tp;
      tf::poseTFToMsg(tmp,nmp);

      m.id = idx;
      m.header.frame_id = root_link;
      m.pose = nmp;

      markers.markers.push_back(m);
      //std::cout << m << std::endl;
      idx++;
    }
  }

  return ret;

}


tf::Transform EndEffectorHelper::getTransformToLink(std::string root_link, std::string link, std::string joint, sensor_msgs::JointState jpos, EndEffectorHelper::TransformMap T_joints, EndEffectorHelper::TransformMap T_map) 
{

  tf::Transform T;

  // if we already have it, just return
  if(T_map.find(link) != std::end(T_map)) {
    return T_map[link];
  } 

  // if its not a fixed joint, not in the JPos msg, look transform up from TF
  if(!rdf_model_->isJointFixed(joint)) {
    if (std::find(std::begin(jpos.name), std::end(jpos.name), joint)==jpos.name.end()) {
      try {
        tf::StampedTransform ts;
        tf_listener_.waitForTransform(link, root_link, ros::Time(0), ros::Duration(2.0) );
        tf_listener_.lookupTransform(link, root_link, ros::Time(0), ts);
        return (tf::Transform)ts.inverse();
      } catch (tf::TransformException ex) {
        ROS_ERROR("EndEffectorHelper::getTransformToLink() problem looking up transform [%s -> %s]: %s", root_link.c_str(), link.c_str(), ex.what());
        return T;  
      }
    }    
  }

  // call recursive function to parent
  std::string parent_link = rdf_model_->getParentLink(link);
  std::string parent_joint = rdf_model_->getLinkJoint(parent_link);
  T = getTransformToLink(root_link, parent_link, parent_joint, jpos, T_joints, T_map);

  // add last transform
  boost::shared_ptr<urdf::Joint> jnt = rdf_model_->getJoint(joint);
  
  double q[4];
  jnt->parent_to_joint_origin_transform.rotation.getQuaternion(q[0],q[1],q[2],q[3]);
  tf::Transform T_kin(
    tf::Quaternion(q[0],q[1],q[2],q[3]), 
    tf::Vector3(jnt->parent_to_joint_origin_transform.position.x, 
                jnt->parent_to_joint_origin_transform.position.y, 
                jnt->parent_to_joint_origin_transform.position.z)
    );


  geometry_msgs::Transform msg;
  
  // std::cout << "---------------------" << std::endl;
  // std::cout << "T" << std::endl;
  // tf::transformTFToMsg(T, msg); 
  // std::cout << msg << std::endl;

  // std::cout << "T_kin" << std::endl;
  // tf::transformTFToMsg(T_kin, msg); 
  // std::cout << msg << std::endl;

  if(T_joints.find(joint) != std::end(T_joints)) {
    // std::cout << "T_joints[ " << joint << "]" << std::endl;
    // tf::transformTFToMsg(T_joints[joint], msg); 
    // std::cout << msg << std::endl;
    T_map[link] = T*T_kin*T_joints[joint];
  } else {
    T_map[link] = T*T_kin;    
  }

  // std::cout << "T_map[ " << link << "]" << std::endl;
  // tf::transformTFToMsg(T_map[link], msg); 
  // std::cout << msg << std::endl;

  return T_map[link];

}


bool EndEffectorHelper::getMarkersForPose(std::string pose_name, visualization_msgs::MarkerArray &markers) 
{
  bool ret = true;
  if (pose_name == "current") {
      // calcuate the markers for the current configuration of the EE
      ret = getCurrentPositionMarkerArray(markers, root_frame_);
  } else {
    markers = pose_marker_map_[pose_name];
    return ret;
  }
}


bool EndEffectorHelper::getCurrentPositionMarkerArray(visualization_msgs::MarkerArray &markers, std::string root_link) 
{
  return getMarkerArrayFromJointState(current_jpos_, root_link, markers);
}


bool EndEffectorHelper::hasPoseMarkers(std::string pose_name) 
{
  return (pose_marker_map_.find(pose_name) != std::end(pose_marker_map_)); 
}
