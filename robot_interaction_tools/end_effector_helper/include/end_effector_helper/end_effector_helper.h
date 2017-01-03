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

#ifndef END_EFFECTOR_HELPER_H_
#define END_EFFECTOR_HELPER_H_

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <rit_utils/rdf_model.h>
#include <rit_utils/generic_utils.h>

namespace end_effector_helper
{
  class EndEffectorHelper
  {

    public:
    
      typedef std::map<std::string, visualization_msgs::MarkerArray > PoseMarkerMap;
      typedef std::map<std::string, tf::Transform > TransformMap;

      EndEffectorHelper(std::string group_name, std::string root_frame, rit_utils::RDFModel *rdf_model);
      ~EndEffectorHelper();

      bool setup();

      // bool populate_data(links, urdf, srdf);
      bool getMarkersForPose(std::string pose_name, visualization_msgs::MarkerArray &markers);
      bool getCurrentPositionMarkerArray(visualization_msgs::MarkerArray &markers, std::string root_link);
      bool getMarkerArrayFromJointState(sensor_msgs::JointState jpos, std::string root_link, visualization_msgs::MarkerArray& markers);
      
      bool hasPoseMarkers(std::string pose_name);

      tf::Transform getTransformToLink(std::string root_link, std::string link, std::string joint, sensor_msgs::JointState jpos, TransformMap T_joints, TransformMap T_map); 


    protected:

      ros::NodeHandle nh_;

      rit_utils::RDFModel *rdf_model_;
      tf::TransformListener tf_listener_;
      ros::Subscriber js_sub_;

      sensor_msgs::JointState current_jpos_;

      std::string group_name_;
      std::string root_frame_;
      std::vector<std::string> links_;
      std::vector<std::string> joints_;

      std::vector<std::string> group_state_list_;
      std::map<std::string, sensor_msgs::JointState> group_states_;

      PoseMarkerMap pose_marker_map_;
 
      void jointStateCallback(const sensor_msgs::JointState &msg);

  };

  typedef boost::shared_ptr<EndEffectorHelper> EndEffectorHelperConstPtr;
};

#endif
