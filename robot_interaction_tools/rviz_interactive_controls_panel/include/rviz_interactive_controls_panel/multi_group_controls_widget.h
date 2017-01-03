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

#ifndef MULTI_GROUP_CONTROLS_WIDGET_HPP
#define MULTI_GROUP_CONTROLS_WIDGET_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <ros/package.h>

// #include "robot_interaction_tools/InteractiveControlsInterface.h"
// #include <planner_interface/planner_interface.h>
#include <robot_interaction_tools_msgs/InteractiveControlsInterface.h>
#include <robot_interaction_tools_msgs/GetGroupConfiguration.h>
#include <robot_interaction_tools_msgs/ConfigureGroup.h>

#include "ui_multi_group_controls_widget.h"

namespace Ui {
class MultiGroupControls;
}

namespace rviz_interactive_controls_panel {

    class GroupControlsWidget;

    class MultiGroupControlsWidget : public QWidget
    {
        Q_OBJECT

    public:
        explicit MultiGroupControlsWidget(QWidget *parent = 0);
         ~MultiGroupControlsWidget();

        void setNodeHandle(ros::NodeHandle &nh) {
            nh_ = nh;
        }
        
        void setServiceClient(const std::map<int8_t, ros::ServiceClient> &map)
        {
          service_client_map_ = map;
        }

        void setServiceClient(ros::ServiceClient *client_) { 
            service_client_ = client_;
        }

        void resetGroups() {
            group_map.clear();
            ui->group_list->clear();
        }

        bool addGroup(const std::string &group_name,
                      GroupControlsWidget* group_widget);

        void removeGroup(const std::string &group_name);

        //void setupDisplay();
        bool setDataFromResponse(robot_interaction_tools_msgs::ConfigureGroupResponse &resp);

    public Q_SLOTS:

        bool planRequest();
        bool executeRequest();

        void planOnMoveClicked(int d);
        void executeOnPlanClicked(int d);
        
    private:

        // the ui
        Ui::MultiGroupControls *ui;
        void setupWidgets();

        // ros node handle
        ros::NodeHandle nh_;
        ros::ServiceClient *service_client_;
        std::map<int8_t, ros::ServiceClient> service_client_map_;

        std::map<std::string, GroupControlsWidget*> group_map;
        // NOTE: gcvec is vector of bools; for use with ROS message types,
        //   use unsigned char
        void getCheckedGroups(bool andVal, std::vector<std::string> &gnvec,
                              std::vector<unsigned char> &gcvec);
        bool getChecked(const QString &gn);
        QListWidgetItem* getListItem(const QString &gn);

        bool initialized;

     public:
        bool plan_on_move;
        bool execute_on_plan;
        bool plan_found;
    };

}

#endif // MULTI_GROUP_CONTROLS_WIDGET_HPP
