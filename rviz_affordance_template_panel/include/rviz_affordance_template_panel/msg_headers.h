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

#ifndef AFFORDANCE_TEMPLATE_MSG_HEADERS_HPP
#define AFFORDANCE_TEMPLATE_MSG_HEADERS_HPP

// messages
#include <affordance_template_msgs/AffordanceTemplateConfig.h>
#include <affordance_template_msgs/EndEffectorConfig.h>
#include <affordance_template_msgs/EndEffectorPoseData.h>
#include <affordance_template_msgs/RobotConfig.h>
#include <affordance_template_msgs/WaypointInfo.h>
#include <affordance_template_msgs/WaypointTrajectory.h>
#include <affordance_template_msgs/GripperActionMap.h>

// services
#include <affordance_template_msgs/AddAffordanceTemplate.h>
#include <affordance_template_msgs/AddAffordanceTemplateTrajectory.h>
#include <affordance_template_msgs/SaveAffordanceTemplate.h>
#include <affordance_template_msgs/AffordanceTemplatePlanCommand.h>
#include <affordance_template_msgs/AffordanceTemplateExecuteCommand.h>
#include <affordance_template_msgs/DeleteAffordanceTemplate.h>
#include <affordance_template_msgs/GetAffordanceTemplateConfigInfo.h>
#include <affordance_template_msgs/GetRobotConfigInfo.h>
#include <affordance_template_msgs/GetRunningAffordanceTemplates.h>
#include <affordance_template_msgs/GetAffordanceTemplateStatus.h>
#include <affordance_template_msgs/GetAffordanceTemplateServerStatus.h>
#include <affordance_template_msgs/LoadRobotConfig.h>
#include <affordance_template_msgs/ScaleDisplayObjectInfo.h>
#include <affordance_template_msgs/ScaleDisplayObject.h>
#include <affordance_template_msgs/SetAffordanceTemplateTrajectory.h>
#include <affordance_template_msgs/SetWaypointViewModes.h>

#endif // AFFORDANCE_TEMPLATE_MSG_HEADERS_HPP