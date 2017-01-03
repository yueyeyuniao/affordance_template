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

#include <rviz_interactive_controls_panel/rviz_panel.h>
#include <rviz_interactive_controls_panel/rosparam_dialog.h>

using namespace rviz_interactive_controls_panel;
using namespace std;

typedef robot_interaction_tools_msgs::ConfigureGroupRequest ConfigReq;

RVizInteractiveControlsPanel::RVizInteractiveControlsPanel(QWidget *parent)
    : rviz::Panel(parent)
    , ui(new Ui::RVizInteractiveControlsPanel)
    , topic_base_("/interactive_controls")
    , initialized(false)
    , multi_group_widget(NULL)
    // , navigation_widget(NULL)
{
    ui->setupUi(this);

    // setup service clients
    interactive_control_get_info_client_ = nh_.serviceClient<robot_interaction_tools_msgs::GetGroupConfiguration>(topic_base_ +"/get_info");
    interactive_control_configure_client_ = nh_.serviceClient<robot_interaction_tools_msgs::ConfigureGroup>(topic_base_ +"/configure");
    
    // setup QT widgets
    setupWidgets();

    ui->GroupTabs->removeTab(1);
    ui->GroupTabs->show();
    
    getConfigData();
}

RVizInteractiveControlsPanel::~RVizInteractiveControlsPanel()
{
    delete ui;
    group_widgets.clear();
}

void RVizInteractiveControlsPanel::setupWidgets() {
    QObject::connect(ui->refresh_button, SIGNAL(clicked()), this, SLOT(getConfigData()));
    QObject::connect(ui->param_button, SIGNAL(clicked()), this, SLOT(popupParamData()));
    QObject::connect(ui->add_group_button, SIGNAL(clicked()), this, SLOT(addGroupRequest()));
    QObject::connect(ui->remove_group_button, SIGNAL(clicked()), this, SLOT(removeGroupRequest()));
    QObject::connect(ui->active_group_list, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(groupDoubleClicked(QListWidgetItem*)));
}

bool RVizInteractiveControlsPanel::setupFromConfigResponse(std::vector<robot_interaction_tools_msgs::PlanGroupConfiguration> &group_configurations) {
    ROS_DEBUG("RVizInteractiveControlsPanel::setupFromConfigResponse()");
    // ROS_INFO("RVizInteractiveControlsPanel::setupFromConfigResponse() have [%lu] previous groups", previous_groups.size());

    ui->all_group_list->clear();
    ui->active_group_list->clear();

    for(auto &gc : group_configurations) {
        ROS_DEBUG("Found Group: %s, active: %d", gc.group_name.c_str(), (int)gc.is_active);
        ui->all_group_list->addItem(QString(gc.group_name.c_str()));
 
        std::string group_name = gc.group_name;

        if(gc.is_active) {

            ui->active_group_list->addItem(QString(group_name.c_str()));
            addGroupControls(group_name);
            ROS_DEBUG("RVizInteractiveControlsPanel: added group [%s], ptr=%p",
                    group_name.c_str(), group_widgets[group_name]);

            group_widgets[group_name]->group_name = group_name;
            group_widgets[group_name]->joint_names.clear();
            group_widgets[group_name]->joint_mask.clear();
            group_widgets[group_name]->last_sent_joint_mask.clear();

            int jdx=0;
            for (auto& j: gc.joint_mask.joint_names) {
                group_widgets[group_name]->joint_names.push_back(j);
                //std::cout << "joint name: " << j << ", mask: " << (int)gc.joint_mask.mask[jdx] << std::endl;
                group_widgets[group_name]->joint_mask.push_back(gc.joint_mask.mask[jdx]);
                group_widgets[group_name]->last_sent_joint_mask.push_back(gc.joint_mask.mask[jdx]);
                jdx++;  
            }
            group_widgets[group_name]->group_type = gc.group_type;
            group_widgets[group_name]->plan_on_move = gc.plan_on_move;
            group_widgets[group_name]->execute_on_plan = gc.execute_on_plan;
            group_widgets[group_name]->plan_cartesian = gc.compute_cartesian_plans;


            group_widgets[group_name]->stored_poses.clear();
            for (auto& stored_pose: gc.stored_pose_list) {  
                group_widgets[group_name]->stored_poses.push_back(stored_pose);
            }

            group_widgets[group_name]->conditioning_metrics.clear();
            for (auto& m: gc.conditioning_metrics) {
                group_widgets[group_name]->conditioning_metrics.push_back(m);
            }

            group_widgets[group_name]->conditioning_metric = gc.conditioning_metric;
            
            for (auto& tol_mode: gc.tolerances) {
                std::cout << "found tolerance mode (outside): " << tol_mode.mode << std::endl;

                if(tol_mode.mode == "position") {
                    group_widgets[group_name]->position_tolerances.clear();
                    for (auto& tol_type: tol_mode.types) {
                        group_widgets[group_name]->position_tolerances.push_back(tol_type);
                    }
                }
                if(tol_mode.mode == "orientation") {
                    group_widgets[group_name]->orientation_tolerances.clear();
                    for (auto& tol_type: tol_mode.types) {
                        group_widgets[group_name]->orientation_tolerances.push_back(tol_type);
                    }
                }
            }

            if (gc.tolerance_settings.size() > 0) {
                for (auto& tol_info: gc.tolerance_settings) {
                    if(tol_info.mode == "position") {
                        group_widgets[group_name]->position_tolerance = tol_info.types[0];
                    } 
                    if(tol_info.mode == "orientation") {
                        group_widgets[group_name]->orientation_tolerance = tol_info.types[0];
                    }
                }
            }

            group_widgets[group_name]->setupDisplay();

        } else {
            auto search = group_widgets.find(group_name);
            if(search != group_widgets.end())  {
                ROS_INFO("RVizInteractiveControlsPanel: removing old group tab for %s", group_name.c_str());
                int idx = ui->GroupTabs->indexOf(group_widgets[group_name]);
                ui->GroupTabs->removeTab(idx);
                ui->GroupTabs->show();
                group_widgets.erase(group_name);
                if (multi_group_widget) {
                   multi_group_widget->removeGroup(group_name);
                }
            }
        }
        
    }


    // for(uint idx=0; idx<resp.active_group_name.size(); idx++) {
    //     std::string group_name = resp.active_group_name[idx];
    //     group_widgets[group_name]->setupDisplay();
    // }
    ROS_DEBUG("RVizInteractiveControlsPanel: group controls updated");

    //updateMultiGroupControls(resp);
    //ROS_DEBUG("RVizInteractiveControlsPanel: multi-group controls updated");

    initialized = true;

    return true;
}
        

bool RVizInteractiveControlsPanel::addGroupControls(std::string group_name) {
    previous_groups[group_name] = 0; // TODO: should be tab index
    auto search = group_widgets.find(group_name);
    if(search == group_widgets.end()) {
        ROS_WARN("RVizInteractiveControlsPanel::addGroupControls(%s)", group_name.c_str());    
        const QString label(group_name.c_str());
        group_widgets[group_name] = new GroupControlsWidget();
        group_widgets[group_name]->setNodeHandle(nh_);
        group_widgets[group_name]->setServiceClient(&interactive_control_configure_client_); 
        ui->GroupTabs->addTab((QWidget *)group_widgets[group_name], label);
        ui->GroupTabs->show();
    } else {
        return false;
    }
    return true;
}

void RVizInteractiveControlsPanel::updateMultiGroupControls(std::vector<robot_interaction_tools_msgs::PlanGroupConfiguration> &group_configurations) {
    // ROS_DEBUG("RVizInteractiveControlsPanel::updateMultiGroupControls()");
    // if (resp.active_group_name.size() > 1) {
    //     if (addMultiGroupControls()) {
    //         // slightly wasteful: just reset and refill, as we're only
    //         // processing a string/pointer per group
    //         multi_group_widget->resetGroups();
    //         for (uint idx=0; idx<resp.active_group_name.size(); ++idx) {
    //             std::string group_name = resp.active_group_name[idx];
    //             if (group_widgets.count(group_name) > 0) {
    //                 multi_group_widget->addGroup(group_name, group_widgets[group_name]);
    //             }
    //         }
    //     }
    // } else {
    //     removeMultiGroupControls();
    // }
}

bool RVizInteractiveControlsPanel::addMultiGroupControls() {
    ROS_DEBUG("RVizInteractiveControlsPanel::addMultiGroupControls()");
    // assume always called after setting up group tabs
    // removing the tab for non-NULL puts it at the end
    const QString label("MultiGroup");
    if (multi_group_widget == NULL) {
        ROS_DEBUG("RVizInteractiveControlsPanel: creating MultiGroupControls");
        multi_group_widget = new MultiGroupControlsWidget();
        multi_group_widget->setNodeHandle(nh_);
        multi_group_widget->setServiceClient(&interactive_control_configure_client_); //@Seth added 9/4
    } else {
        ROS_DEBUG("RVizInteractiveControlsPanel: remove MultiGroupControls tab");
        int index = ui->GroupTabs->indexOf(multi_group_widget);
        if (index >= 0) {
            ui->GroupTabs->removeTab(index);
        }
    }
    if (multi_group_widget != NULL) {
        ROS_DEBUG("RVizInteractiveControlsPanel: add MultiGroupControls tab");
        ui->GroupTabs->addTab((QWidget *)multi_group_widget, label);
        ui->GroupTabs->show();
    }
    return (multi_group_widget != NULL);
}

bool RVizInteractiveControlsPanel::removeMultiGroupControls() {
    ROS_DEBUG("RVizInteractiveControlsPanel::removeMultiGroupControls()");
    if (multi_group_widget != NULL) {
        int index = ui->GroupTabs->indexOf(multi_group_widget);
        if (index >= 0) {
            ui->GroupTabs->removeTab(index);
        }
        delete multi_group_widget;
        multi_group_widget = NULL;
        ui->GroupTabs->show();
    }
    return true;
}

bool RVizInteractiveControlsPanel::getConfigData() {
    ROS_DEBUG("RVizInteractiveControlsPanel::getConfigData() -- querying all group info");    
    robot_interaction_tools_msgs::GetGroupConfiguration srv;
    if (interactive_control_get_info_client_.call(srv)) {
        ROS_DEBUG("RVizInteractiveControlsPanel::getConfigData() -- success");
        setupFromConfigResponse(srv.response.group_configurations);
        for(auto &gc : srv.response.group_configurations) {
            ROS_DEBUG("Found Group: %s, active: %d", gc.group_name.c_str(), (int)gc.is_active);
        }
    } else {
        ROS_ERROR("RVizInteractiveControlsPanel::getConfigData() -- failed to call service to get group info");
        return false;
    }
    return true;
}

bool RVizInteractiveControlsPanel::popupParamData() {
    std::string param(topic_base_);
    RosparamDialog rpd(param + "/param", this);
    if (rpd.exec()) {
        getConfigData();
    }    
    return true;
}


bool RVizInteractiveControlsPanel::addGroupRequest() {
    ROS_DEBUG("RVizInteractiveControlsPanel::addGroupRequest()");    

    std::string group_name;
    std::string group_type = ui->group_type_combo_box->currentText().toStdString();
    QList<QListWidgetItem *> items = ui->all_group_list->selectedItems();

    if(items.size() > 1) {
        ROS_ERROR("RVizInteractiveControlsPanel::addGroupRequest() -- failed to call service");
        return false;
    }

    for (auto& g: items) {
        group_name = g->text().toStdString(); 
    }

    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::ADD_GROUP;
    srv.request.group_configuration.group_name = group_name;
    srv.request.group_configuration.group_type = group_type;
    
    if (interactive_control_configure_client_.call(srv)) {
        ROS_DEBUG("RVizInteractiveControlsPanel::addGroupRequest() -- success");
        setupFromConfigResponse(srv.response.group_configurations);
    } else {
        ROS_ERROR("RVizInteractiveControlsPanel::addGroupRequest() -- failed to call service");
        return false;
    }

    return true;
}

bool RVizInteractiveControlsPanel::removeGroupRequest() {
    ROS_DEBUG("RVizInteractiveControlsPanel::removeGroupRequest()");    
    
    std::string group_name;
    QList<QListWidgetItem *> items = ui->active_group_list->selectedItems();

    if(items.size() > 1) {
        ROS_ERROR("RVizInteractiveControlsPanel::removeGroupRequest() -- failed to call service");
        return false;
    }

    for (auto& g: items) {
        group_name = g->text().toStdString();    
        int idx = ui->GroupTabs->indexOf(group_widgets[group_name]);
        ui->GroupTabs->removeTab(idx);
        ui->GroupTabs->show();
        group_widgets.erase(group_name);
        if (multi_group_widget) {
           multi_group_widget->removeGroup(group_name);
        }
    }

    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::REMOVE_GROUP;
    srv.request.group_configuration.group_name = group_name;
    
    if (interactive_control_configure_client_.call(srv)) {
        ROS_DEBUG("RVizInteractiveControlsPanel::removeGroupRequest() -- success");
        setupFromConfigResponse(srv.response.group_configurations);
    } else {
        ROS_ERROR("RVizInteractiveControlsPanel::removeGroupRequest() -- failed to call service");
        return false;
    }

    return true;
}

void RVizInteractiveControlsPanel::groupDoubleClicked(QListWidgetItem* item) {
    selectTab(item->text().toStdString());
}

bool RVizInteractiveControlsPanel::selectTab(const std::string &gn) {
    bool retval = false;
    if (group_widgets.count(gn) > 0) {
        int idx = ui->GroupTabs->indexOf(group_widgets[gn]);
        if (idx >= 0) {
            ui->GroupTabs->setCurrentIndex(idx);
            ui->GroupTabs->show();
            retval = true;
        }
    }
    return retval;
}


#include <pluginlib/class_list_macros.h>
#if ROS_VERSION_MINIMUM(1,9,41)
    PLUGINLIB_EXPORT_CLASS(rviz_interactive_controls_panel::RVizInteractiveControlsPanel, rviz::Panel)
#else
    PLUGINLIB_DECLARE_CLASS(rviz_interactive_controls_panel, RVizInteractiveControlsPanel, rviz_interactive_controls_panel::RVizInteractiveControlsPanel, rviz::Panel)
#endif


