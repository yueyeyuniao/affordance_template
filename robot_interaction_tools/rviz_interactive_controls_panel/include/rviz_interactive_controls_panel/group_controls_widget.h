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

#ifndef GROUP_CONTROLS_WIDGET_HPP
#define GROUP_CONTROLS_WIDGET_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <ros/package.h>

#include <robot_interaction_tools_msgs/ToleranceInfo.h>
#include <robot_interaction_tools_msgs/PlanGroupConfiguration.h>
#include <robot_interaction_tools_msgs/ConfigureGroup.h>

#include "ui_group_controls_widget.h"

namespace Ui {
class GroupControls;
}

namespace rviz_interactive_controls_panel {

    class GroupControlsWidget : public QWidget
    {
        Q_OBJECT

    public:
        explicit GroupControlsWidget(QWidget *parent = 0);
         ~GroupControlsWidget();

        void setNodeHandle(ros::NodeHandle &nh) {
            nh_ = nh;
        }
        
        void setServiceClient(ros::ServiceClient *client_) { 
            service_client_ = client_;
        }

        bool setGroupDataFromConfig(robot_interaction_tools_msgs::PlanGroupConfiguration config, QString from = "");
        bool setGroupDataFromResponse(robot_interaction_tools_msgs::ConfigureGroupResponse resp, QString from = "");

        void setupDisplay(QString from = "");
        void fillPlanRequest(robot_interaction_tools_msgs::ConfigureGroup &srv);

    public Q_SLOTS:

        bool planRequest();
        bool executeRequest();
        // bool toggleJointControlRequest();
        bool storedPoseRequest();
        void setToolOffsetClicked();
        void clearToolOffsetClicked();

        void planOnMoveClicked(int d);
        void executeOnPlanClicked(int d);
        void planCartesianClicked(int d);
        void jointMaskChanged(QListWidgetItem* item);
        bool positionToleranceChanged(const QString& text);
        bool rotationToleranceChanged(const QString& text);
        bool conditioningMetricChanged(const QString& text);

    private:

        // the ui
        Ui::GroupControls *ui;

        // setup widget function
        void setupWidgets();

        // ros node handle
        ros::NodeHandle nh_;

        ros::ServiceClient *service_client_;

        bool initialized;

     public:

        // group storage info
        std::string group_name;
        std::string group_type;
        std::string path_visualization_mode;
        
        std::vector<std::string> joint_names;
        std::vector<bool> joint_mask;
        std::vector<bool> last_sent_joint_mask;
        
        std::vector<std::string> position_tolerances;
        std::vector<std::string> orientation_tolerances;

        std::string position_tolerance;
        std::string orientation_tolerance;

        std::vector<std::string> conditioning_metrics;
        std::string conditioning_metric;

        std::vector<std::string> stored_poses;
        
        bool plan_on_move;
        bool execute_on_plan;
        bool plan_cartesian;

        bool plan_found;
    };

}

#endif // GROUP_CONTROLS_WIDGET_HPP
