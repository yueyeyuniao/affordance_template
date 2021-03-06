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

#include <rviz_interactive_controls_panel/interface_utils.h>
#include <algorithm>
#include <iostream>

namespace rviz_interactive_controls_panel {

void InteractiveControlsInterfaceUtils::inactiveGroups(
                                  const ICInterface::Response &srv,
                                  std::unordered_set<std::string> &inactive) {
                                  //std::vector<std::string> &inactive) {
    std::vector<std::string> gns(srv.group_name), act(srv.active_group_name), inact;
    std::sort(gns.begin(), gns.end());
    std::sort(act.begin(), act.end());
    std::vector<std::string>::iterator it;
    //std::unordered_set<std::string>::iterator it;
    it=std::set_difference(gns.begin(), gns.end(), act.begin(), act.end(), inact.begin());
    inact.resize(it-inact.begin());
    inactive.clear();
    inactive.insert(inact.begin(), inact.end());
}

std::string InteractiveControlsInterfaceUtils::srvStr(
                               const ICInterface &srv, bool suppress) {
    std::stringstream oss;
    oss << requestStr(srv.request, suppress) << std::endl;
    oss << "---" << std::endl;
    oss << responseStr(srv.response, suppress) << std::endl;
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::requestStr(
                           const ICInterface::Request &srv, bool suppress) {
    std::stringstream oss;
    oss << "action_type: " << actionStr(srv.action_type) << std::endl;
    oss << "group_name[]: " << vecString2Str(srv.group_name) << std::endl;
    oss << "group_type[]: " << vecString2Str(srv.group_type) << std::endl;
    oss << "stored_pose_name[]: " << vecString2Str(srv.stored_pose_name) << std::endl;
    oss << "path_visualization_mode[]: " << vecString2Str(srv.path_visualization_mode) << std::endl;
    oss << "goal_pose[]: ";
    if (!suppress) {
        oss << std::endl << vecPoseStamped2Str(srv.goal_pose) << std::endl;
    } else {
        oss << "*output suppressed*" << std::endl;
    }
    oss << "joint_mask[]: ";
    if (!suppress) {
        oss << vecJointMask2Str(srv.joint_mask);
    } else {
        oss << "*output suppressed*" << std::endl;
    }
    oss << "tolerance[]: ";
    if (!suppress) {
        oss << vecToleranceInfo2Str(srv.tolerance);
    } else {
        oss << "*output suppressed*" << std::endl;
    }
    oss << "execute_on_plan[]: " << vecBool2Str(srv.execute_on_plan) << std::endl;
    oss << "plan_on_move[]: " << vecBool2Str(srv.plan_on_move) << std::endl;
    oss << "navigation_waypoint_name[]: " << vecString2Str(srv.navigation_waypoint_name) << std::endl;
    oss << "plan_footsteps: " << (srv.plan_footsteps ? "true" : "false") << std::endl;
    oss << "accommodate_terrain_in_navigation: " << (srv.accommodate_terrain_in_navigation ? "true" : "false") << std::endl;
    oss << "navigation_mode: " << srv.navigation_mode << std::endl;
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::responseStr(
                           const ICInterface::Response &srv, bool suppress) {
    std::stringstream oss;
    oss << "action_type: " << actionStr(srv.action_type) << std::endl;
    oss << "group_name[]: " << vecString2Str(srv.group_name) << std::endl;
    oss << "group_type[]: " << vecString2Str(srv.group_type) << std::endl;
    oss << "active_group_name[]: " << vecString2Str(srv.active_group_name) << std::endl;
    oss << "plan_found[]: " << vecBool2Str(srv.plan_found) << std::endl;
    oss << "joint_mask[]: ";
    if (!suppress) {
        oss << vecJointMask2Str(srv.joint_mask);
    } else {
        oss << "*output suppressed*" << std::endl;
    }
    oss << "joint_names[]: ";
    if (!suppress) {
        oss << vecJointNameMap2Str(srv.joint_names);
    } else {
        oss << "*output suppressed*" << std::endl;
    }
    oss << "path_visualization_mode[]: " << vecString2Str(srv.path_visualization_mode) << std::endl;
    oss << "tolerance[]: ";
    if (!suppress) {
        oss << vecToleranceInfo2Str(srv.tolerance);
    } else {
        oss << "*output suppressed*" << std::endl;
    }
    oss << "tolerance_setting[]: *skipping output*" << std::endl;
    //oss << "tolerance_setting[]: " << vecToleranceInfoArray2Str(srv.response.tolerance_setting) << std::endl;
    oss << "execute_on_plan[]: " << vecBool2Str(srv.execute_on_plan) << std::endl;
    oss << "plan_on_move[]: " << vecBool2Str(srv.plan_on_move) << std::endl;
    oss << "stored_pose_list[]: *skipping output*" << std::endl;
    //oss << "stored_pose_list[]: " << vecStringArray2Str(srv.response.stored_pose_list) << std::endl;
    oss << "has_navigation_controls: " << (srv.has_navigation_controls ? "true" : "false") << std::endl;
    oss << "plan_footsteps: " << (srv.plan_footsteps ? "true" : "false") << std::endl;
    oss << "navigation_waypoint_name[]: " << vecString2Str(srv.navigation_waypoint_name) << std::endl;
    oss << "navigation_mode: " << srv.navigation_mode << std::endl;
    oss << "accommodate_terrain_in_navigation: " << (srv.accommodate_terrain_in_navigation ? "true" : "false") << std::endl;
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::actionStr(char type) {
    switch (type) {
    case ICInterface::Request::GET_INFO:
        return "GET_INFO";
    case ICInterface::Request::ADD_GROUP:
        return "ADD_GROUP";
    case ICInterface::Request::REMOVE_GROUP:
        return "REMOVE_GROUP";
    case ICInterface::Request::SET_MARKER_POSE:
        return "SET_MARKER_POSE";
    case ICInterface::Request::SET_SHOW_PATH:
        return "SET_SHOW_PATH";
    case ICInterface::Request::SET_PLAN_ON_MOVE:
        return "SET_PLAN_ON_MOVE";
    case ICInterface::Request::SET_EXECUTE_ON_PLAN:
        return "SET_EXECUTE_ON_PLAN";
    case ICInterface::Request::SET_TOLERANCES:
        return "SET_TOLERANCES";
    case ICInterface::Request::SET_JOINT_MAP:
        return "SET_JOINT_MAP";
    case ICInterface::Request::EXECUTE_STORED_POSE:
        return "EXECUTE_STORED_POSE";
    case ICInterface::Request::TOGGLE_POSTURE_CONTROLS:
        return "TOGGLE_POSTURE_CONTROLS";
    case ICInterface::Request::SYNC_MARKERS_TO_ROBOT:
        return "SYNC_MARKERS_TO_ROBOT";
    case ICInterface::Request::PLAN_TO_MARKER:
        return "PLAN_TO_MARKER";
    case ICInterface::Request::EXECUTE_PLAN:
        return "EXECUTE_PLAN";
    case ICInterface::Request::ADD_NAVIGATION_WAYPOINT:
        return "ADD_NAVIGATION_WAYPOINT";
    case ICInterface::Request::DELETE_NAVIGATION_WAYPOINT:
        return "DELETE_NAVIGATION_WAYPOINT";
    case ICInterface::Request::PLAN_NAVIGATION_PATH:
        return "PLAN_NAVIGATION_PATH";
    case ICInterface::Request::EXECUTE_NAVIGATION_PATH:
        return "EXECUTE_NAVIGATION_PATH";
    case ICInterface::Request::PLAN_FOOTSTEPS_IN_PATH:
        return "PLAN_FOOTSTEPS_IN_PATH";
    case ICInterface::Request::SET_FIRST_FOOT:
        return "SET_FIRST_FOOT";
    default:
        return "unknown";
    }
}

std::string InteractiveControlsInterfaceUtils::vecString2Str(
                                   const std::vector<std::string> &vec) {
    std::stringstream oss;
    oss << "[ ";
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << vec[i] << " ";
    }
    oss << "]";
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::vecPoseStamped2Str(
                        const std::vector<geometry_msgs::PoseStamped> &vec) {
    std::stringstream oss;
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << "PoseStamped[" << i << "]:" << std::endl << vec[i];
    }
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::vecJointMask2Str(
                        const std::vector<robot_interaction_tools_msgs::JointMask> &vec) {
    std::stringstream oss;
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << "JointMask[" << i << "]:" << std::endl << vec[i];
    }
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::vecJointNameMap2Str(
                    const std::vector<robot_interaction_tools_msgs::JointNameMap> &vec) {
    std::stringstream oss;
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << "JointNameMap[" << i << "]:" << std::endl << vec[i];
    }
    return oss.str();
}

std::string InteractiveControlsInterfaceUtils::vecToleranceInfo2Str(
                   const std::vector<robot_interaction_tools_msgs::ToleranceInfo> &vec) {
    std::stringstream oss;
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << "ToleranceInfo[" << i << "]:" << std::endl << vec[i];
    }
    return oss.str();
}

/*
std::string InteractiveControlsInterfaceUtils::vecToleranceInfoArray2Str(
             const std::vector<robot_interaction_tools_msgs::ToleranceInfoArray> &vec) {
    std::stringstream oss;
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << "ToleranceInfoArray[" << i << "]:" << std::endl;
        oss << vecToleranceInfo2Str(vec[i]);
    }
    return oss.str();
}
*/

std::string InteractiveControlsInterfaceUtils::vecBool2Str(
                        const std::vector<uint8_t> &vec) {
    std::stringstream oss;
    oss << "[ ";
    for (unsigned int i=0; i<vec.size(); ++i) {
        oss << (vec[i] ? "true" : "false") << " ";
    }
    oss << "]";
    return oss.str();
}

}
