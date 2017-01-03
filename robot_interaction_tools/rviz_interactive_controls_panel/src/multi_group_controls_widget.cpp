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

#include <rviz_interactive_controls_panel/group_controls_widget.h>
#include <rviz_interactive_controls_panel/multi_group_controls_widget.h>
#include <rviz_interactive_controls_panel/interface_utils.h>
#include <iostream>

using namespace rviz_interactive_controls_panel;
using namespace std;

MultiGroupControlsWidget::MultiGroupControlsWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MultiGroupControls)
    , initialized(false)
{
    ui->setupUi(this);
    setupWidgets();
}

MultiGroupControlsWidget::~MultiGroupControlsWidget() {
    delete ui;
}

void MultiGroupControlsWidget::setupWidgets() {
    QObject::connect(ui->plan_button, SIGNAL(clicked()),
                     this, SLOT(planRequest()));
    QObject::connect(ui->execute_button, SIGNAL(clicked()),
                     this, SLOT(executeRequest()));

    QObject::connect(ui->plan_on_move, SIGNAL(stateChanged(int)),
                     this, SLOT(planOnMoveClicked(int)));
    QObject::connect(ui->execute_on_plan, SIGNAL(stateChanged(int)),
                     this, SLOT(executeOnPlanClicked(int)));
}

bool MultiGroupControlsWidget::addGroup(const std::string &group_name,
                                        GroupControlsWidget* group_widget) {
    auto search = group_map.find(group_name);
    if (search == group_map.end()) {
        ROS_DEBUG("MultiGroupControlsWidget::addGroup(%s) ptr=%p, type=%s",
                group_name.c_str(), group_widget, group_widget->group_type.c_str());
        const QString label(group_name.c_str());
        if(group_widget->group_type == "cartesian") {
            ROS_DEBUG("MultiGroupControlsWidget::addGroup(%s) ptr=%p, type=%s",
                group_name.c_str(), group_widget, group_widget->group_type.c_str());
            group_map[group_name] = group_widget;
            ui->group_list->addItem(label);
            QListWidgetItem* item = getListItem(label);
            if (item != NULL) {
                item->setCheckState(Qt::Checked);
            } else {
                item->setCheckState(Qt::Unchecked);
            }
        } else {
            return false;
        }
    } else {
        return false;
    }
    return true;
}

void MultiGroupControlsWidget::removeGroup(const std::string &group_name) {
    if (group_map.count(group_name) > 0) {
        std::map<std::string, GroupControlsWidget*>::iterator it;
        it = group_map.find(group_name);
        group_map.erase(it);
    }
    for (int gdx=0; gdx<ui->group_list->count(); ++gdx) {
        QListWidgetItem* listItem = ui->group_list->item(gdx);
        if (group_name == listItem->text().toStdString()) {
            ui->group_list->removeItemWidget(listItem);
            break;
        }
    }
}

bool MultiGroupControlsWidget::setDataFromResponse(robot_interaction_tools_msgs::ConfigureGroupResponse &resp) {
    bool retval = false;
    //std::cout << "MultiGroupControlsWidget: response:" << std::endl;
    //std::cout << ConfigureGroupUtils::responseStr(resp) << std::endl;
    
    plan_found = (resp.group_configurations.size() > 0 ? true : false);
    for (uint idx=0; idx<resp.group_configurations.size(); ++idx) {
        std::string group_name(resp.group_configurations[idx].group_name);
        if (group_map.count(group_name) > 0) {
            // only consider checked groups for plan status display
            if (!getChecked(QString::fromStdString(group_name))) {
                group_map[group_name]->setGroupDataFromConfig(resp.group_configurations[idx]);
            } else {
                retval = retval &&
                    group_map[group_name]->setGroupDataFromConfig(resp.group_configurations[idx], QString(" (from Multigroup)"));
                plan_found = plan_found && group_map[group_name]->plan_found;
            }
        } else {
            ROS_INFO("    missing in group_map!");
        }
    }
    
    if (plan_found) {
        ui->plan_label->setText(QString("PLAN FOUND"));
    } else {
        ui->plan_label->setText(QString("NO PLAN"));
    }
    return retval;
}

bool MultiGroupControlsWidget::planRequest() {
    ROS_INFO("MultiGroupControlsWidget::planRequest()");
    bool retval = false;
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::PLAN_TO_MARKER;
    for (auto git=group_map.cbegin(); git!=group_map.cend(); ++git) {
        if (getChecked(QString::fromStdString(git->first))) {
            git->second->fillPlanRequest(srv);
        }
    }
    //std::cout << "MultiGroupControlsWidget: request:" << std::endl;
    //std::cout << rviz_interactive_controls_panel::ConfigureGroupUtils::requestStr(srv.request) << std::endl;

    if (service_client_->call(srv)) {
        ROS_INFO("MultiGroupControlsWidget::planRequest() -- success");
        // return value is boolean && of all groups' setGroupData
        retval = setDataFromResponse(srv.response);
    } else {
        ROS_ERROR("MultiGroupControlsWidget::planRequest() -- failed to call service");
    }
    return retval;
}

bool MultiGroupControlsWidget::executeRequest() {
    ROS_INFO("MultiGroupControlsWidget::executeRequest()");
    bool retval = false;
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::EXECUTE_PLAN;
    std::vector<unsigned char> junk;
    // FIXME
    //getCheckedGroups(true, srv.request.group_configuration.group_name, junk);

    if (service_client_->call(srv)) {
        ROS_INFO("MultiGroupControlsWidget::executeRequest() -- success");
        retval = setDataFromResponse(srv.response);
    } else {
        ROS_ERROR("MultiGroupControlsWidget::executeRequest() -- failed to call service");
    }
    return retval;
}

void MultiGroupControlsWidget::planOnMoveClicked(int d) {
    ROS_INFO("MultiGroupControlsWidget::planOnMoveClicked()");    
    plan_on_move = (d == Qt::Checked);
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::SET_PLAN_ON_MOVE;

    //getCheckedGroups(plan_on_move, srv.request.group_configuration.group_name, srv.request.group_configuration.plan_on_move);
    for (auto git=group_map.cbegin(); git!=group_map.cend(); ++git) {
        srv.request.group_configuration.group_name = git->first;
        srv.request.group_configuration.plan_on_move = getChecked(QString::fromStdString(git->first)) && plan_on_move;
    }
    //std::cout << "MultiGroupControlsWidget: request:" << std::endl;
    //std::cout << ConfigureGroupUtils::requestStr(srv.request) << std::endl;

    if (service_client_->call(srv)) {
        ROS_INFO("MultiGroupControlsWidget::planOnMoveClicked() -- success");
        setDataFromResponse(srv.response);
    } else {
        ROS_ERROR("MultiGroupControlsWidget::planOnMoveClicked() -- failed to call service");
    }
}

void MultiGroupControlsWidget::executeOnPlanClicked(int d) {
    ROS_INFO("MultiGroupControlsWidget::executeOnPlanClicked()");    
    execute_on_plan = (d == Qt::Checked);
    robot_interaction_tools_msgs::ConfigureGroup srv;

    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::SET_EXECUTE_ON_PLAN;
    //getCheckedGroups(execute_on_plan, srv.request.group_configuration.group_name, srv.request.group_configuration.execute_on_plan);
    for (auto git=group_map.cbegin(); git!=group_map.cend(); ++git) {
        srv.request.group_configuration.group_name = git->first;
        srv.request.group_configuration.execute_on_plan = getChecked(QString::fromStdString(git->first)) && execute_on_plan;
    }
    //std::cout << "MultiGroupControlsWidget: request:" << std::endl;
    //std::cout << ConfigureGroupUtils::requestStr(srv.request) << std::endl;

    if (service_client_->call(srv)) {
        ROS_INFO("MultiGroupControlsWidget::executeOnPlanClicked() -- success");
        setDataFromResponse(srv.response);
    } else {
        ROS_ERROR("MultiGroupControlsWidget::executeOnPlanClicked() -- failed to call service");
    }
}

void MultiGroupControlsWidget::getCheckedGroups(bool andVal,
                std::vector<std::string> &gnvec, std::vector<unsigned char> &gcvec) {
    ROS_DEBUG("MultiGroupControlsWidget::getCheckedGroups()");
    gnvec.clear(); gcvec.clear(); // just to be sure
    
    for (auto git=group_map.cbegin(); git!=group_map.cend(); ++git) {
        gnvec.push_back(git->first);
        gcvec.push_back((unsigned char)(andVal &&
                                        getChecked(QString::fromStdString(git->first))));
    }
}

bool MultiGroupControlsWidget::getChecked(const QString &gn) {
    bool retval = false;
    QListWidgetItem* listItem = getListItem(gn);
    if (listItem != NULL) {
        retval = (listItem->checkState() == Qt::Checked);
    }
    return retval;
}

QListWidgetItem* MultiGroupControlsWidget::getListItem(const QString &gn) {
    QListWidgetItem* retval = NULL;
    // holy schmoley, either me or (Qt) is dumb in this respect...there's
    // really no way to get an item via it's text? fine, loop over all...
    // TODO: set up a map to keep track of the check state
    for (int gdx=0; gdx<ui->group_list->count(); ++gdx) {
        QListWidgetItem* listItem = ui->group_list->item(gdx);
        if (gn == listItem->text()) {
            retval = listItem;
            break;
        }
    }
    return retval;
}

