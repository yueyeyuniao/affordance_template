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

#ifndef RDF_PARSER_UTIL_H_
#define RDF_PARSER_UTIL_H_

#include <ros/ros.h>
#include <srdfdom/model.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <urdf_parser/urdf_parser.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>


#ifndef GEN_UTILS_H_
#define GEN_UTIL_H_
#include <rit_utils/generic_utils.h>
#endif

namespace rit_utils
{
class RDFModel
{
  bool parseRDF(const srdf::Model &srdf, const boost::shared_ptr<urdf::ModelInterface> &urdf);
  
  // SRDF data structures
  std::string robot_name_;
  std::vector<srdf::Model::Group> groups_;
  std::vector<srdf::Model::GroupState> group_states_;
  std::vector<srdf::Model::EndEffector> end_effectors_;
  std::vector<srdf::Model::LinkSpheres> collision_spheres_; // TODO getters - we don't know if we need this info
  std::vector<srdf::Model::PassiveJoint> passive_joints_; // TODO getters - we don't know if we need this info
  std::vector<srdf::Model::VirtualJoint> virtual_joints_; // TODO getters - we don't know if we need this info
  std::vector<srdf::Model::DisabledCollision> disabled_collisions_; // TODO getters - we don't know if we need this info
  // URDF data structures
  std::map<std::string, std::string> mesh_links_;
  std::vector<boost::shared_ptr<urdf::Link> > urdf_links_;
  std::map<std::string, boost::shared_ptr<urdf::Joint> > urdf_joints_;
  std::map<std::string, std::pair<std::string, std::string> > fixed_joints_;
  std::map<std::string, std::pair<std::string, std::string> > robot_links_;
  std::map<std::string, std::vector<std::string> > full_group_joints_;
  std::map<std::string, std::vector<std::string> > full_group_links_;
  std::map<std::string, std::pair<double, double> > joint_limits_;
  std::map<std::string, double > joint_velocities_;
  std::vector<std::string> urdf_tips_;

  KDL::Tree tree;

public:
  RDFModel(){}
  RDFModel(const std::string& _robot_name);
  ~RDFModel();

  bool init(const std::string& srdf="/robot_description_semantic", const std::string& urdf="/robot_description");
  bool createModels(const std::string& srdf, const std::string& urdf);
  bool findGroup(const std::string& group_name, srdf::Model::Group& group);
  bool findLinkJoint(const std::string& link, std::string& joint);

  bool getKDLChain(const std::string& group_name, KDL::Chain& chain);
  bool getKDLChain(const std::string& group_name, std::vector<std::string>&);
  bool getKDLLimits(const std::string& group_name, KDL::JntArray& lower, KDL::JntArray& upper);
  bool getJointVelocities(const std::string& group_name, std::vector<double>& velocities);

  inline const std::string& getName() { return robot_name_; }

  std::vector<std::string> getGroups();
  std::vector<std::string> getEndEffectors();

  bool getJointLimits(const std::string& group, std::vector<double>& lower, std::vector<double>& upper);

  boost::shared_ptr<urdf::Link> getLink(const std::string& link);
  boost::shared_ptr<urdf::Joint> getJoint(const std::string& joint);

  bool getLinks(const std::string& group, std::vector<std::string>& links);
  bool getJoints(const std::string& group, std::vector<std::string>& joints, bool full_joints=true, bool fixed=false);
  bool getChain(const std::string& group, std::string& base, std::string& tip);
  bool getChain(std::string base, std::string tip, std::vector<std::string>& links);
  bool getBaseLink(const std::string& group, std::string& link);
  bool getTipLink(const std::string& group, std::string& link);
  bool getSubgroup(const std::string& group, std::vector<std::string>& subgroup);
  bool getEndEffector(const std::string& group, std::string& ee);
  bool getEndEffectorPlanningGroup(const std::string& ee, std::string& group);
  bool getEndEffectorParentGroup(const std::string& ee, std::string& group);
  bool getGroupState(const std::string& group, const std::string& name, sensor_msgs::JointState& state);
  bool getGroupStateMap(const std::string& group, std::map<std::string, sensor_msgs::JointState> &group_states);
  bool getEndEffectorBaseLink(const std::string& ee, std::string& link);
  bool isJointFixed(const std::string &name);
  bool isChain(const std::string& group);
  bool isChain(const std::string& group, std::string& base, std::string& tip);
  
  bool isEndEffector(const std::string& group);
  bool checkValidEndEffector(const std::string& group);
  
  std::string getParentLink(const std::string& name);
  std::string getChildLink(const std::string& name);
  std::string getLinkJoint(const std::string& name);

  visualization_msgs::MarkerArray getLinkMarkers(const std::string& link);
  visualization_msgs::MarkerArray getGroupMeshes(const std::string& group);

}; // end of class
} // end of rit_utils namespace

#endif
