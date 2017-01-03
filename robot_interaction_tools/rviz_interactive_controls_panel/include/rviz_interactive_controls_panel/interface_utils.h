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

#ifndef INTERACTIVE_CONTROLS_INTERFACE_UTILS_H
#define INTERACTIVE_CONTROLS_INTERFACE_UTILS_H

// #include "robot_interaction_tools/InteractiveControlsInterface.h"
// #include <planner_interface/planner_interface.h>
#include <robot_interaction_tools_msgs/JointMask.h>
#include <robot_interaction_tools_msgs/JointNameMap.h>
#include <robot_interaction_tools_msgs/ToleranceInfo.h>
#include <robot_interaction_tools_msgs/ToleranceInfoArray.h>
#include <robot_interaction_tools_msgs/InteractiveControlsInterface.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <unordered_set>
#include <vector>

namespace rviz_interactive_controls_panel {

    // typedef for using message in Qt signal/slot; registered in .cpp
    typedef robot_interaction_tools_msgs::InteractiveControlsInterface ICInterface;
    //typedef robot_interaction_tools_msgs::InteractiveControlsInterface::Request InteractiveControlsInterfaceRequest;
    //typedef robot_interaction_tools_msgs::InteractiveControlsInterface::Response InteractiveControlsInterfaceResponse;
    typedef robot_interaction_tools_msgs::InteractiveControlsInterfaceRequest ICInterfaceRequest;
    typedef robot_interaction_tools_msgs::InteractiveControlsInterfaceResponse ICInterfaceResponse;
    
    // TODO: need a service call thread object
    
    class InteractiveControlsInterfaceUtils {
        public:
            static
            void inactiveGroups(const ICInterface::Response &srv,
                                std::unordered_set<std::string> &inactive);
                                //std::vector<std::string> &inactive);
            
            static
            std::string srvStr(const ICInterface &srv, bool suppress=true);
            
            static
            std::string requestStr(const ICInterfaceRequest &srv, bool suppress=true);
            
            static
            std::string responseStr(const ICInterfaceResponse &srv, bool suppress=true);
            
            static
            std::string actionStr(char type);
            
        private:
            static
            std::string vecString2Str(const std::vector<std::string> &vec);
            
            static
            std::string vecPoseStamped2Str(const std::vector<geometry_msgs::PoseStamped> &vec);
            
            static
            std::string vecJointMask2Str(const std::vector<robot_interaction_tools_msgs::JointMask> &vec);
            
            static
            std::string vecToleranceInfo2Str(const std::vector<robot_interaction_tools_msgs::ToleranceInfo> &vec);
            
            //static
            //std::string vecToleranceInfoArray2Str(const std::vector<robot_interaction_tools_msgs::ToleranceInfoArray> &vec);
            
            static
            std::string vecBool2Str(const std::vector<uint8_t> &vec);
            
            static
            std::string vecJointNameMap2Str(const std::vector<robot_interaction_tools_msgs::JointNameMap> &vec);
            
    };
     
}
#endif

