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

#ifndef RVIZ_INTERACTIVE_CONTROLS_PANEL_HPP
#define RVIZ_INTERACTIVE_CONTROLS_PANEL_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <rviz/panel.h>
#include <ros/package.h>

#include <robot_interaction_tools_msgs/GetGroupConfiguration.h>
#include <robot_interaction_tools_msgs/ConfigureGroup.h>

#include "ui_rviz_panel.h"
#include <rviz_interactive_controls_panel/interface_utils.h>
#include <rviz_interactive_controls_panel/group_controls_widget.h>
#include <rviz_interactive_controls_panel/multi_group_controls_widget.h>

#include <robot_interaction_tools_msgs/AddGroup.h>
#include <robot_interaction_tools_msgs/RemoveGroup.h>
#include <robot_interaction_tools_msgs/ExecuteCommand.h>

namespace Ui {
class RVizInteractiveControlsPanel;
}

namespace rviz_interactive_controls_panel {

    class RVizInteractiveControlsPanel : public rviz::Panel
    {
        Q_OBJECT

    public:
        explicit RVizInteractiveControlsPanel(QWidget *parent = 0);
        ~RVizInteractiveControlsPanel();
        
      public Q_SLOTS:  
         bool getConfigData();
         bool popupParamData();
         bool addGroupRequest();
         bool removeGroupRequest();
         void groupDoubleClicked(QListWidgetItem*);

    private:
        
        // pointer to maink UI panel
        Ui::RVizInteractiveControlsPanel *ui;

        // service client to get/set info
        ros::ServiceClient interactive_control_configure_client_;
        ros::ServiceClient interactive_control_get_info_client_;

        // ros node handle
        ros::NodeHandle nh_;

        // topic base
        std::string topic_base_;

        // init flag
        bool initialized;
        
        // array of group widgets
        std::map<std::string, GroupControlsWidget *> group_widgets;
        std::map<std::string, int> previous_groups;
        MultiGroupControlsWidget *multi_group_widget;

        // setup widget function
        void setupWidgets();
        bool selectTab(const std::string &text);

        // function add a new group controls tab
        bool addGroupControls(std::string group_name);

        // multi-group controls
        void updateMultiGroupControls(std::vector<robot_interaction_tools_msgs::PlanGroupConfiguration> &group_configurations);
        bool addMultiGroupControls();
        bool removeMultiGroupControls();

        bool setupFromConfigResponse(std::vector<robot_interaction_tools_msgs::PlanGroupConfiguration> &group_configurations);

    };

}

#endif // RVIZ_INTERACTIVE_CONTROLS_PANEL_HPP

