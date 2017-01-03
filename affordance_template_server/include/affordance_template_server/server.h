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

#ifndef _AFFORDANCE_TEMPLATE_SERVER_H_
#define _AFFORDANCE_TEMPLATE_SERVER_H_

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_listener.h>

#include <affordance_template_markers/robot_interface.h>
#include <affordance_template_markers/affordance_template.h>

#include <affordance_template_library/affordance_template_structure.h>
#include <affordance_template_library/affordance_template_parser.h>

#include <affordance_template_msgs/RobotConfig.h>
#include <affordance_template_msgs/AffordanceTemplateConfig.h>

#include <interactive_markers/interactive_marker_server.h>

#include <geometry_msgs/PoseStamped.h>

#include <boost/thread.hpp>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>

namespace affordance_template_server
{
  class AffordanceTemplateServer
  {
    void configureServer();
    bool loadRobot();
    bool loadTemplates();
    int getNextID(const std::string&);
    std::string getPackagePath(const std::string&);

    ros::NodeHandle nh_;
    boost::mutex mutex_;

    tf::TransformListener listener_;
    affordance_template_msgs::RobotConfig robot_config_;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;
    boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface_;

    std::map<std::string, boost::shared_ptr<affordance_template::AffordanceTemplate> > at_map_;
    std::map<std::string, affordance_template_object::AffordanceTemplateStructure> at_structure_map_;

    bool status_;
    std::string pkg_name_;
    std::string robot_yaml_;
    std::string robot_name_;
    
  public:
    AffordanceTemplateServer(){} // default constructor 
    AffordanceTemplateServer(const ros::NodeHandle &, const std::string&);
    ~AffordanceTemplateServer(){}
     
    inline bool getStatus() { return status_; }
    inline void setStatus(bool status) { status_ = status; }
    inline bool findTemplate(const std::string &name) { at_structure_map_.find(name) == at_structure_map_.end() ? false : true; }
    inline affordance_template_msgs::RobotConfig getRobotConfig() { return robot_config_; }
    
    std::vector<affordance_template_msgs::AffordanceTemplateConfig> getAvailableTemplates(const std::string &name="");
    std::vector<std::string> getRunningTemplates(const std::string &name="");

    bool loadRobot(const std::string&); // from file
    bool loadRobot(const affordance_template_msgs::RobotConfig&); // from msg

    bool refreshTemplates() { return loadTemplates(); }

    bool addTemplate(const std::string&, uint8_t&);
    bool addTemplate(const std::string&, uint8_t&, geometry_msgs::PoseStamped&);
    bool removeTemplate(const std::string&, const uint8_t);
    bool updateTemplate(const std::string&, const uint8_t, const geometry_msgs::PoseStamped&);

    bool getTemplateInstance(const std::string&, const uint8_t, boost::shared_ptr<affordance_template::AffordanceTemplate>&);
    bool getTemplateInstance(const std::string&, boost::shared_ptr<affordance_template::AffordanceTemplate>&);
  };
}

#endif 