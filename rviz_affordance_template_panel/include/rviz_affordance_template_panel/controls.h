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

#ifndef CONTROLS_HPP
#define CONTROLS_HPP

#include <ros/ros.h>

#include <iostream>
#include <boost/shared_ptr.hpp>

#include <QTableWidgetItem>

#include <rviz_affordance_template_panel/robot_config.h>
#include <rviz_affordance_template_panel/util.h>
#include <rviz_affordance_template_panel/template_status_info.h>

#include "ui_rviz_affordance_template_panel.h"

#include <affordance_template_msgs/AffordanceTemplatePlanCommand.h>
#include <affordance_template_msgs/AffordanceTemplateExecuteCommand.h>

namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel
{
    class Controls
    {
    public:

        enum CommandType 
        {
            START,
            END,
            STEP_FORWARD,
            STEP_BACKWARD,
            CURRENT,
            PAUSE
        };

        typedef boost::shared_ptr<RobotConfig> RobotConfigSharedPtr;

        Controls();
        ~Controls() {};

        inline void setServices(ros::ServiceClient plan_srv, ros::ServiceClient execute_srv) { planService_ = plan_srv; executeService_ = execute_srv; }
        inline void setRobotMap(std::map<std::string, RobotConfigSharedPtr> map) { robotMap_ = map; }
        inline void setRobotName(std::string name) { robotName_ = name; }
        inline void setTemplateStatusInfo(AffordanceTemplateStatusInfo *template_status) { template_status_ = template_status; }
        inline void setUI(Ui::RVizAffordanceTemplatePanel* ui) { ui_ = ui; }
        inline AffordanceTemplateStatusInfo * getTemplateStatusInfo() { return template_status_; }
        
        bool requestPlan(Controls::CommandType command_type, bool exe_on_plan=false);
        bool executePlan();
        std::vector<std::pair<std::string,int> > getSelectedEndEffectorInfo();

    private:

        Ui::RVizAffordanceTemplatePanel* ui_;
    
        ros::ServiceClient planService_;
        ros::ServiceClient executeService_;
    
        std::map<std::string, RobotConfigSharedPtr> robotMap_;
        std::string robotName_;
        AffordanceTemplateStatusInfo *template_status_;

    };
}

#endif // CONTROLS_HPP