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

#ifndef NAVIGATION_CONTROLS_WIDGET_HPP
#define NAVIGATION_CONTROLS_WIDGET_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <ros/package.h>

// #include "robot_interaction_tools/InteractiveControlsInterface.h"
// #include <planner_interface/planner_interface.h>
#include <robot_interaction_tools_msgs/InteractiveControlsInterface.h>

#include "ui_navigation_controls_widget.h"


namespace Ui {
class NavigationControls;
}

namespace rviz_interactive_controls_panel
{

    class NavigationControlsWidget : public QWidget
    {
        Q_OBJECT

    public:
        explicit NavigationControlsWidget(QWidget *parent = 0);
         ~NavigationControlsWidget();

        void setNodeHandle(ros::NodeHandle &nh) {
            nh_ = nh;
        }
        
        void setServiceClient(ros::ServiceClient *client_) { 
            service_client_ = client_;
        }

        void setupDisplay();
        bool setDataFromResponse(robot_interaction_tools_msgs::InteractiveControlsInterfaceResponse &resp);

    public Q_SLOTS:

        bool planRequest();
        bool executeRequest();
        bool directMoveRequest();
        bool syncToRobotOrientationRequest();
        bool syncToRobotLocationRequest();
        bool syncToPathOrientationRequest();
        bool saveFootstepPathRequest();
        
        bool addWaypointRequest();
        bool deleteWaypointRequest();

        void accommodateTerrainClicked(int d);
        bool navModeChanged(const QString&);

    private:

        // the ui
        Ui::NavigationControls *ui;

        // setup widget function
        void setupWidgets();

        // ros node handle
        ros::NodeHandle nh_;

        ros::ServiceClient *service_client_;

        bool initialized;


     public:

        // group storage info
        bool accommodate_terrain;

        std::vector<std::string> waypoint_list;
        std::vector<std::string> navigation_modes;
        std::string navigation_mode;
        
        bool plan_found;
    };

}

#endif // NAVIGATION_CONTROLS_WIDGET_HPP
