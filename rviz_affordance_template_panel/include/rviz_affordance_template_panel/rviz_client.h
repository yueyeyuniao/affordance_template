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

#ifndef AFFORDANCE_TEMPLATE_RVIZ_CLIENT_HPP
#define AFFORDANCE_TEMPLATE_RVIZ_CLIENT_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <rviz/panel.h>
#include <ros/package.h>

/* stuff for getting urdf */
#include <urdf/model.h>
#include <kdl/frames.hpp>

/* qt */
#include <QGraphicsScene>
#include <QTableWidgetItem>
#include <QSlider>

/* Project Include */
#include <rviz_affordance_template_panel/affordance.h>
#include <rviz_affordance_template_panel/robot_config.h>
#include <rviz_affordance_template_panel/controls.h>
#include <rviz_affordance_template_panel/waypoint_display.h>
#include <rviz_affordance_template_panel/util.h>
#include "ui_rviz_affordance_template_panel.h"

#include <geometry_msgs/Pose.h>

#include <rviz_affordance_template_panel/msg_headers.h>
#include <rviz_affordance_template_panel/template_status_info.h>
#include <rviz_affordance_template_panel/server_status_monitor.h>

namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel
{
    class AffordanceTemplateRVizClient 
    {

    public:

        // typedefs
        typedef boost::shared_ptr<Affordance> AffordanceSharedPtr;
        typedef boost::shared_ptr<RobotConfig> RobotConfigSharedPtr;
        typedef boost::shared_ptr<EndEffectorConfig> EndEffectorConfigSharedPtr;
        typedef boost::shared_ptr<EndEffectorPoseConfig> EndEffectorPoseIDConfigSharedPtr;
        typedef boost::shared_ptr<Controls> ControlsSharedPtr;
        typedef boost::shared_ptr<WaypointDisplay> WaypointDisplaySharedPtr;
        typedef std::pair<std::string, int> TemplateInstanceID;

        // Constructors
        AffordanceTemplateRVizClient(ros::NodeHandle &nh, Ui::RVizAffordanceTemplatePanel* ui, QGraphicsScene *at_scene);
        ~AffordanceTemplateRVizClient();

        // init function
        void init();

        // thread functions
        void start();
        void stop();

        // print stored template status info
        void printTemplateStatus();
        
        // helper functions for front-end widgets
        void getAvailableInfo();
        void getAvailableTemplates();
        void getAvailableRobots();
        void getRunningItems();

        void addAffordanceDisplayItem();
        void addTrajectory();

        void selectAffordanceTemplate(QListWidgetItem* item);
        void deleteAffordanceTemplate();
        void killAffordanceTemplate(QListWidgetItem* item);

        void saveAffordanceTemplate();
        void refreshCallback();
        void loadConfig();
        void safeLoadConfig();

        void changeRobot(int id);
        void changeSaveInfo(int id);
        void changeEndEffector(int id);

        void goToStart();
        void goToEnd();
        void stepBackward();
        void stepForward();
        void goToCurrentWaypoint();
        void executePlan();

        void updateRobotConfig(const QString& text);
        void updateEndEffectorGroupMap(const QString&);
        void updateObjectScale(int value);
        void updateEndEffectorScaleAdjustment(int value);
        void selectScaleObject(const QString& object_name);
        void scaleSliderReleased();
        void controlStatusUpdate();
        void resetScale();      
        void enableConfigPanel(int state);
        void selectTemplateTrajectory(const QString& text);
        
    protected:

        void run_function();

        void setupRobotPanel(const string& key);
        void setupEndEffectorConfigPanel(const string& key);

        bool tryToLoadRobotFromYAML();

        void removeAffordanceTemplates();
        int  sendAffordanceTemplateAdd(const string& class_name);
        void sendAffordanceTemplateKill(const string& class_name, int id);
        void sendSaveAffordanceTemplate();
        void sendAddTrajectory();

        void sendObjectScale(affordance_template_msgs::ScaleDisplayObjectInfo scale_info);
        void streamObjectScale(affordance_template_msgs::ScaleDisplayObjectInfo scale_info);

        bool addAffordance(const AffordanceSharedPtr& obj);
        bool removeAffordance(const AffordanceSharedPtr& obj);
        bool checkAffordance(const AffordanceSharedPtr& obj);

        bool addRobot(const RobotConfigSharedPtr& obj);
        bool removeRobot(const RobotConfigSharedPtr& obj);
        bool checkRobot(const RobotConfigSharedPtr& obj);

        void updateServerStatus();
        void setLabelText(QColor color, std::string text);

        void updateStatusFromControls();   

        void updateTables(std::string name, std::string trajectory);
        void updateControlsTable(std::string name, std::string trajectory);
        void updateWaypointDisplayTable(std::string name, std::string trajectory);

        void doCommand(Controls::CommandType command_type);
        void sendScaleInfo();
        void setupDisplayObjectSliders(TemplateInstanceID template_instance);
        bool endEffectorInTrajectory(AffordanceTemplateStatusInfo::EndEffectorInfo ee_info);

        std::string createShortName(const std::string&);
        std::string getLongName(const std::string&);
        std::map<std::string, std::string> robot_name_map_; 
                
        std::string getRobotFromDescription();
        std::vector<std::string> getSelectedEndEffectors();

        // boost thread
        boost::thread *thread_;
        boost::mutex mutex;

        // status flag
        bool running_;
        bool busy_flag_;
        int server_status_;
        
        // ros node handle
        ros::NodeHandle nh_;

        // UI reference
        Ui::RVizAffordanceTemplatePanel* ui_;

        // GUI Widgets
        QGraphicsScene* affordanceTemplateGraphicsScene_;

        // server status monitor thread
        AffordanceTemplateServerStatusMonitor *server_monitor_;

        // map to track instantiated object templates
        std::map<std::string, AffordanceSharedPtr> affordanceMap_;
        std::map<std::string, RobotConfigSharedPtr> robotMap_;
        std::string descriptionRobot_;
        std::string robot_name_;
        bool robot_configured_;
        bool waypoint_display_configured_;

        // affordance template services
        ros::ServiceClient add_template_client_;
        ros::ServiceClient delete_template_client_;
        ros::ServiceClient add_trajectory_client_;
        ros::ServiceClient plan_command_client_;
        ros::ServiceClient execute_command_client_;
        ros::ServiceClient get_robots_client_;
        ros::ServiceClient get_running_client_;
        ros::ServiceClient get_templates_client_;
        ros::ServiceClient load_robot_client_;
        ros::ServiceClient save_template_client_;
        ros::ServiceClient scale_object_client_;
        ros::ServiceClient get_template_status_client_;
        ros::ServiceClient set_template_trajectory_client_;
        ros::ServiceClient set_waypoint_view_client_;

        // affordance template publishers
        ros::Publisher scale_object_streamer_;

        // control helper class
        ControlsSharedPtr controls_;

        // waypoint display helper class
        WaypointDisplaySharedPtr waypointDisplay_;
        std::map<std::string,bool> waypointExpansionStatus_;

        // template bookkeeping
        TemplateInstanceID selected_template;
        std::map<std::pair<TemplateInstanceID, std::string>, int> display_object_scale_map;
        std::map<std::pair<TemplateInstanceID, std::string>, int> end_effector_adjustment_map;
        std::map<std::string, AffordanceTemplateStatusInfo*> template_status_info; 

        // extra graphics stuff
        QPalette *label_palette_;
        QColor red, blue, green;

    };
}
#endif // AFFORDANCE_TEMPLATE_RVIZ_CLIENT_HPP