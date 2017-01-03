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

#ifndef WAYPOINT_DISPLAY_HPP
#define WAYPOINT_DISPLAY_HPP

#include <ros/ros.h>

#include <iostream>
#include <boost/shared_ptr.hpp>

#include <QTableWidgetItem>
#include <QTreeWidgetItem>

#include <rviz_affordance_template_panel/robot_config.h>
#include <rviz_affordance_template_panel/util.h>
#include <rviz_affordance_template_panel/template_status_info.h>

#include "ui_rviz_affordance_template_panel.h"

#include <affordance_template_msgs/SetWaypointViewModes.h>

namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel
{
    class WaypointDisplay : public QObject
    {

    Q_OBJECT
    public:

        typedef boost::shared_ptr<RobotConfig> RobotConfigSharedPtr;

        WaypointDisplay(QObject *_parent=0);
        ~WaypointDisplay() {};

        void setService(ros::ServiceClient set_srv) { setWaypointDisplayService_ = set_srv; };
        void setRobotMap(std::map<std::string, RobotConfigSharedPtr> map) { robotMap_ = map; };
        void setRobotName(std::string name) { robotName_ = name; };
        void setupWaypointDisplayInfo(AffordanceTemplateStatusInfo::EndEffectorInfo wp_info);

        void setUI(Ui::RVizAffordanceTemplatePanel* ui) { 
            ui_ = ui;
            QObject::connect(ui_->waypointDisplayTree, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(displayEventChecked(QTreeWidgetItem*, int)), Qt::UniqueConnection);
         }

        void setTemplateStatusInfo(AffordanceTemplateStatusInfo *template_status) { template_status_ = template_status; }
        AffordanceTemplateStatusInfo * getTemplateStatusInfo() { return template_status_; }

        void callWaypointDisplayService(std::vector<std::string> wp_names, std::vector<bool> modes);

    public Q_SLOTS:
        void displayEventChecked(QTreeWidgetItem *item,int i);

    private:

        Ui::RVizAffordanceTemplatePanel* ui_;
    
        ros::ServiceClient setWaypointDisplayService_;
    
        std::map<std::string, RobotConfigSharedPtr> robotMap_;
        std::string robotName_;
        AffordanceTemplateStatusInfo *template_status_;

        std::map<std::pair<std::string,int>, QTreeWidgetItem*> waypointDisplayItems_;
        std::map<std::string,int> eeNameMap_;

    };
}

#endif // WAYPOINT_DISPLAY_HPP