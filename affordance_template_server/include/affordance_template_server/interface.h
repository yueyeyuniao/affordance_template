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

#ifndef _AFFORDANCE_TEMPLATE_INTERFACE_H_
#define _AFFORDANCE_TEMPLATE_INTERFACE_H_

#include <affordance_template_server/server.h>
#include <affordance_template_library/affordance_template_structure.h>

#include <affordance_template_msgs/GetRobotConfigInfo.h>
#include <affordance_template_msgs/GetAffordanceTemplateConfigInfo.h>
#include <affordance_template_msgs/LoadRobotConfig.h>
#include <affordance_template_msgs/AddAffordanceTemplate.h>
#include <affordance_template_msgs/DeleteAffordanceTemplate.h>
#include <affordance_template_msgs/GetRunningAffordanceTemplates.h>
#include <affordance_template_msgs/AffordanceTemplatePlanCommand.h>
#include <affordance_template_msgs/AffordanceTemplateExecuteCommand.h>
#include <affordance_template_msgs/SaveAffordanceTemplate.h>
#include <affordance_template_msgs/AddAffordanceTemplateTrajectory.h>
#include <affordance_template_msgs/ScaleDisplayObject.h>
#include <affordance_template_msgs/ScaleDisplayObjectInfo.h>
#include <affordance_template_msgs/GetAffordanceTemplateStatus.h>
#include <affordance_template_msgs/GetAffordanceTemplateServerStatus.h>
#include <affordance_template_msgs/SetAffordanceTemplateTrajectory.h>
#include <affordance_template_msgs/SetAffordanceTemplatePose.h>
#include <affordance_template_msgs/GetObjectPose.h>
#include <affordance_template_msgs/SetObjectPose.h>
#include <affordance_template_msgs/DisplayObjectInfo.h>
#include <affordance_template_msgs/ObjectInfo.h>
#include <affordance_template_msgs/WaypointInfo.h>
#include <affordance_template_msgs/WaypointViewMode.h>
#include <affordance_template_msgs/SetWaypointViewModes.h>

using namespace affordance_template_msgs;

namespace affordance_template_server
{
    typedef boost::shared_ptr<affordance_template::AffordanceTemplate> ATPointer;

    inline std::string boolToString(bool b) { return (b ? "true" : "false"); }
    inline std::string successToString(bool b) { return (b ? "succeeded" : "failed"); }

    class AffordanceTemplateInterface
    {
        // srv handlers
        bool handleRobotRequest(GetRobotConfigInfo::Request&, GetRobotConfigInfo::Response&);
        bool handleTemplateRequest(GetAffordanceTemplateConfigInfo::Request&, GetAffordanceTemplateConfigInfo::Response&);
        bool handleLoadRobot(LoadRobotConfig::Request&, LoadRobotConfig::Response&);
        bool handleAddTemplate(AddAffordanceTemplate::Request&, AddAffordanceTemplate::Response&);
        bool handleDeleteTemplate(DeleteAffordanceTemplate::Request&, DeleteAffordanceTemplate::Response&);
        bool handleRunning(GetRunningAffordanceTemplates::Request&, GetRunningAffordanceTemplates::Response&);
        bool handlePlanCommand(AffordanceTemplatePlanCommand::Request&, AffordanceTemplatePlanCommand::Response&);
        bool handleExecuteCommand(AffordanceTemplateExecuteCommand::Request&, AffordanceTemplateExecuteCommand::Response&);
        bool handleSaveTemplate(SaveAffordanceTemplate::Request&, SaveAffordanceTemplate::Response&);
        bool handleAddTrajectory(AddAffordanceTemplateTrajectory::Request&, AddAffordanceTemplateTrajectory::Response&);
        bool handleObjectScale(ScaleDisplayObject::Request&, ScaleDisplayObject::Response&);
        bool handleTemplateStatus(GetAffordanceTemplateStatus::Request&, GetAffordanceTemplateStatus::Response&);
        bool handleServerStatus(GetAffordanceTemplateServerStatus::Request&, GetAffordanceTemplateServerStatus::Response&);
        bool handleSetTrajectory(SetAffordanceTemplateTrajectory::Request&, SetAffordanceTemplateTrajectory::Response&);
        bool handleSetPose(SetAffordanceTemplatePose::Request&, SetAffordanceTemplatePose::Response&);
        bool handleSetObject(SetObjectPose::Request&, SetObjectPose::Response&);
        bool handleGetObject(GetObjectPose::Request&, GetObjectPose::Response&);
        bool handleSetWaypointViews(SetWaypointViewModes::Request&, SetWaypointViewModes::Response&);

        void runPlanAction();
        void runExecuteAction();
        void handleObjectScaleCallback(const ScaleDisplayObjectInfo&);
        bool doesTrajectoryExist(const ATPointer&, const std::string&);
        bool doesEndEffectorExist(const ATPointer&, const std::string&);
        AffordanceTemplateStatus getTemplateStatus(const std::string& template_name, const int template_id, std::string& traj_name, const std::string& frame_id="");
        
        ros::Subscriber scale_stream_sub_;
        boost::shared_ptr<AffordanceTemplateServer> at_server_;
        tf::TransformListener listener_;
        std::map<std::string, ros::ServiceServer> at_srv_map_;
        ros::NodeHandle nh_;

        // P&E thread stuffs
        std::deque<affordance_template_msgs::AffordanceTemplatePlanCommand::Request> plan_stack_;
        std::deque<affordance_template_msgs::AffordanceTemplateExecuteCommand::Request> exe_stack_;
        boost::scoped_ptr<boost::thread> plan_thread_, exe_thread_;
        boost::mutex plan_mutex_, exe_mutex_;

    public:
        AffordanceTemplateInterface(const ros::NodeHandle&, const std::string&);
        ~AffordanceTemplateInterface();
    };       
}

#endif