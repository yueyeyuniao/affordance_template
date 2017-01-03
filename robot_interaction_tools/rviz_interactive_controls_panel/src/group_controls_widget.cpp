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

using namespace rviz_interactive_controls_panel;
using namespace std;

GroupControlsWidget::GroupControlsWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::GroupControls)
    , initialized(false)
{
    ui->setupUi(this);
    setupWidgets();
}

GroupControlsWidget::~GroupControlsWidget()
{
    delete ui;
}

void GroupControlsWidget::setupWidgets() {

    QObject::connect(ui->plan_button, SIGNAL(clicked()), this, SLOT(planRequest()));
    QObject::connect(ui->execute_button, SIGNAL(clicked()), this, SLOT(executeRequest()));
    //QObject::connect(ui->toggle_joint_control_button, SIGNAL(clicked()), this, SLOT(toggleJointControlRequest()));
    QObject::connect(ui->go_to_stored_pose_button, SIGNAL(clicked()), this, SLOT(storedPoseRequest()));
    QObject::connect(ui->set_tool_offset_button, SIGNAL(clicked()), this, SLOT(setToolOffsetClicked()));
    QObject::connect(ui->clear_tool_offset_button, SIGNAL(clicked()), this, SLOT(clearToolOffsetClicked()));

    QObject::connect(ui->plan_on_move, SIGNAL(stateChanged(int)), this, SLOT(planOnMoveClicked(int)));
    QObject::connect(ui->execute_on_plan, SIGNAL(stateChanged(int)), this, SLOT(executeOnPlanClicked(int)));
    QObject::connect(ui->use_cartesian_planning, SIGNAL(stateChanged(int)), this, SLOT(planCartesianClicked(int)));

    QObject::connect(ui->pos_tol, SIGNAL(activated(const QString&)), this, SLOT(positionToleranceChanged(const QString&)));
    QObject::connect(ui->rot_tol, SIGNAL(activated(const QString&)), this, SLOT(rotationToleranceChanged(const QString&)));
    
    QObject::connect(ui->conditioning_metric, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(conditioningMetricChanged(const QString&)));

    QObject::connect(ui->joint_list, SIGNAL(itemClicked(QListWidgetItem *)), this, SLOT(jointMaskChanged(QListWidgetItem*)));

}

void GroupControlsWidget::setupDisplay(QString from) {

    ui->joint_list->clear();
    
    ui->type_label->setText(QString(group_type.c_str()));

    int index;
    int jdx = 0;

    for (auto& j: joint_names) {
        ui->joint_list->addItem(j.c_str());
        QListWidgetItem *item = ui->joint_list->item(jdx);
        if(joint_mask[jdx]) {
            item->setCheckState(Qt::Checked);
        } else {
            item->setCheckState(Qt::Unchecked);
        }
        jdx++;
    }
    
    if(plan_found) {
        ui->plan_label->setText(QString("PLAN FOUND")+ from);
    } else {
        ui->plan_label->setText(QString("NO PLAN")+ from);
    }

    ui->stored_pose_list->clear();
    for (auto& sp: stored_poses) {
        ui->stored_pose_list->addItem(QString(sp.c_str()));
    }

    if(execute_on_plan) {
        ui->execute_on_plan->setCheckState(Qt::Checked);
    } else {
        ui->execute_on_plan->setCheckState(Qt::Unchecked);
    }


    if(group_type=="cartesian") {
        if(plan_on_move) {
            ui->plan_on_move->setCheckState(Qt::Checked);
        } else {
            ui->plan_on_move->setCheckState(Qt::Unchecked);
        }

        if(plan_cartesian) {
            ui->use_cartesian_planning->setCheckState(Qt::Checked);
        } else {
            ui->use_cartesian_planning->setCheckState(Qt::Unchecked);
        }

        for (auto& m: conditioning_metrics) {
            index = ui->conditioning_metric->findText(QString(m.c_str()));
            if( index == -1 ) {
                ui->conditioning_metric->addItem(QString(m.c_str()));
            }
        }

        for (auto& pt: position_tolerances) {
            index = ui->pos_tol->findText(QString(pt.c_str()));
            if( index == -1 ) {
                ui->pos_tol->addItem(QString(pt.c_str()));
            }
        }
            
        for (auto& rt: orientation_tolerances) {
            index = ui->rot_tol->findText(QString(rt.c_str()));
            if( index == -1 ) {
                ui->rot_tol->addItem(QString(rt.c_str()));
            }
        }

        initialized = true;

        index = ui->pos_tol->findText(QString(position_tolerance.c_str()));
        if ( index != -1 ) { // -1 for not found
            ui->pos_tol->setCurrentIndex(index);
            positionToleranceChanged(QString(position_tolerance.c_str()));
        }
        index = ui->rot_tol->findText(QString(orientation_tolerance.c_str()));
        if ( index != -1 ) { // -1 for not found
            ui->rot_tol->setCurrentIndex(index);
            rotationToleranceChanged(QString(orientation_tolerance.c_str()));
        }

        index = ui->conditioning_metric->findText(QString(conditioning_metric.c_str()));
        if ( index != -1 ) { // -1 for not found
            ui->conditioning_metric->setCurrentIndex(index);
            conditioningMetricChanged(QString(conditioning_metric.c_str()));
        }


    } else {
        ui->plan_on_move->setEnabled(false);
        ui->pos_tol->setEnabled(false);
        ui->rot_tol->setEnabled(false);
        ui->conditioning_metric->setEnabled(false);
        ui->use_cartesian_planning->setEnabled(false);
        // ui->viz_type->setEnabled(false);
        initialized = true;
    }


}

bool GroupControlsWidget::setGroupDataFromResponse(robot_interaction_tools_msgs::ConfigureGroupResponse resp, QString from) {
    if(resp.group_configurations.size() != 1) {
        ROS_ERROR("GroupControlsWidget::setGroupDataFromResponse() -- found %d group configurations (not 1), it's got me confused.", (int)resp.group_configurations.size());
        return false;
    }
    return setGroupDataFromConfig(resp.group_configurations[0], from);
}

bool GroupControlsWidget::setGroupDataFromConfig(robot_interaction_tools_msgs::PlanGroupConfiguration config, QString from) {

    if(group_name != config.group_name) {
        ROS_ERROR("GroupControlsWidget::setGroupDataFromResponse(%s) got response for %s. What did you break?!", group_name.c_str(), config.group_name.c_str());
        return false;
    }
    ROS_INFO("GroupControlsWidget::setGroupDataFromResponse(%s) got response", group_name.c_str());

    joint_names.clear();
    joint_mask.clear();
    int jdx=0;
    for (auto& j: config.joint_mask.joint_names) {
        joint_names.push_back(j);
        joint_mask.push_back(config.joint_mask.mask[jdx]);
        last_sent_joint_mask.push_back(config.joint_mask.mask[jdx]);
        jdx++;  
    }
    group_type = config.group_type;
    plan_on_move = config.plan_on_move;
    execute_on_plan = config.execute_on_plan;
    plan_cartesian = config.compute_cartesian_plans;
    //plan_found = config.plan_found;

    conditioning_metrics.clear();
    for (auto& m: config.conditioning_metrics) {
        conditioning_metrics.push_back(m);
        ROS_INFO("GroupControlsWidget::setGroupDataFromResponse(%s) found metric %s", group_name.c_str(), m.c_str());
    }

    for (auto& tol_mode: config.tolerances) {
        std::cout << "found tolerance mode (inside): " << tol_mode.mode << std::endl;
        if(tol_mode.mode == "position") {
            position_tolerances.clear();
            for (auto& tol_type: tol_mode.types) {
                position_tolerances.push_back(tol_type);
            }
        }
        if(tol_mode.mode == "orientation") {
            orientation_tolerances.clear();
            for (auto& tol_type: tol_mode.types) {
                orientation_tolerances.push_back(tol_type);
            }
        }
    }
   
    if (config.tolerance_settings.size() > 0) {
        for (auto& tol_info: config.tolerance_settings) {
            if(tol_info.mode == "position") {
                position_tolerance = tol_info.types[0];
            } 
            if(tol_info.mode == "orientation") {
                orientation_tolerance = tol_info.types[0];
            }
        }
    }
  
    stored_poses.clear();
    for (auto& stored_pose: config.stored_pose_list) {  
        stored_poses.push_back(stored_pose);
    }       
    setupDisplay(from);

    return initialized;
}


void GroupControlsWidget::jointMaskChanged(QListWidgetItem* item) {
    ROS_DEBUG("GroupControlsWidget::jointMaskChanged()");       
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::SET_JOINT_MASK;
    srv.request.group_configuration.group_name = group_name;
    for (int jdx=0; jdx<ui->joint_list->count(); jdx++) {
        if(item->text() != ui->joint_list->item(jdx)->text()) {
            srv.request.group_configuration.joint_mask.mask.push_back(joint_mask[jdx]);
            srv.request.group_configuration.joint_mask.joint_names.push_back(ui->joint_list->item(jdx)->text().toUtf8().constData());
            continue;
        }
        joint_mask[jdx] = (item->checkState()==Qt::Checked);
        srv.request.group_configuration.joint_mask.mask.push_back(joint_mask[jdx]);
        srv.request.group_configuration.joint_mask.joint_names.push_back(ui->joint_list->item(jdx)->text().toUtf8().constData());
    }
    if (service_client_->call(srv)) {
        ROS_DEBUG("GroupControlsWidget::jointMaskChanged() -- success");
        for (int jdx=0; jdx<ui->joint_list->count(); jdx++) {
            last_sent_joint_mask[jdx] = joint_mask[jdx];
        }
        setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::jointMaskChanged() -- failed to call service");
    }
}

void GroupControlsWidget::planOnMoveClicked(int d) {
    ROS_DEBUG("GroupControlsWidget::planOnMoveClicked()");    
    plan_on_move = (d == Qt::Checked);
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::SET_PLAN_ON_MOVE;
    srv.request.group_configuration.group_name = group_name;
    srv.request.group_configuration.plan_on_move = plan_on_move;
    if (service_client_->call(srv)) {
        ROS_DEBUG("GroupControlsWidget::planOnMoveClicked() -- success");
        setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::planOnMoveClicked() -- failed to call service");
    }
}

void GroupControlsWidget::executeOnPlanClicked(int d) {   
    ROS_DEBUG("GroupControlsWidget::executeOnPlanClicked()");    
    execute_on_plan = (d == Qt::Checked);
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::SET_EXECUTE_ON_PLAN;
    srv.request.group_configuration.group_name = group_name;
    srv.request.group_configuration.execute_on_plan = execute_on_plan;
    if (service_client_->call(srv)) {
        ROS_DEBUG("GroupControlsWidget::executeOnPlanClicked() -- success");
        setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::executeOnPlanClicked() -- failed to call service");
    }
}

void GroupControlsWidget::planCartesianClicked(int d) {   
    ROS_DEBUG("GroupControlsWidget::planCartesianClicked()");    
    plan_cartesian = (d == Qt::Checked);
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::SET_CARTESIAN_PLANNING;
    srv.request.group_configuration.group_name = group_name;
    srv.request.group_configuration.compute_cartesian_plans = plan_cartesian;
    if (service_client_->call(srv)) {
        ROS_DEBUG("GroupControlsWidget::planCartesianClicked() -- success");
        setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::planCartesianClicked() -- failed to call service");
    }
}


bool GroupControlsWidget::conditioningMetricChanged(const QString& text) {
 ROS_DEBUG("GroupControlsWidget::conditioningMetricChanged()");    
    if(!initialized) {
        return true;
    }
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::SET_CONDITIONING_METRIC;
    srv.request.group_configuration.group_name = group_name;
    srv.request.group_configuration.conditioning_metric = ui->conditioning_metric->currentText().toStdString();
    conditioning_metric = srv.request.group_configuration.conditioning_metric;
    if (service_client_->call(srv)) {
        ROS_INFO("GroupControlsWidget::conditioningMetricChanged() -- success");
        return true;//setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::conditioningMetricChanged() -- failed to call service");
        return false;
    }
}

bool GroupControlsWidget::positionToleranceChanged(const QString& text) {
  ROS_INFO_STREAM("GroupControlsWidget::positionToleranceChanged() "<<group_name);    
    if(!initialized) {
        return true;
    }
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::SET_TOLERANCES;
    srv.request.group_configuration.group_name = group_name;
    robot_interaction_tools_msgs::ToleranceInfo pos_tol_info;
    pos_tol_info.mode = "position";
    pos_tol_info.types.push_back(ui->pos_tol->currentText().toStdString());
    srv.request.group_configuration.tolerance_settings.push_back(pos_tol_info);
    position_tolerance = ui->pos_tol->currentText().toStdString();
    if (service_client_->call(srv)) {
        ROS_DEBUG("GroupControlsWidget::positionToleranceChanged() -- success");
        return true;//setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::positionToleranceChanged() -- failed to call service");
        return false;
    }
}
      
bool GroupControlsWidget::rotationToleranceChanged(const QString& text) {
  ROS_INFO_STREAM("GroupControlsWidget::rotationToleranceChanged() "<<group_name);    
    if(!initialized) {
        return true;
    }
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::SET_TOLERANCES;
    srv.request.group_configuration.group_name = group_name;
    robot_interaction_tools_msgs::ToleranceInfo rot_tol_info;
    rot_tol_info.mode = "orientation";
    rot_tol_info.types.push_back(ui->rot_tol->currentText().toStdString());
    srv.request.group_configuration.tolerance_settings.push_back(rot_tol_info);
    orientation_tolerance = ui->rot_tol->currentText().toStdString();
    if (service_client_->call(srv)) {
        ROS_DEBUG("GroupControlsWidget::rotationToleranceChanged() -- success");
        return true;//setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::rotationToleranceChanged() -- failed to call service");
        return false;
    }
 }
          

bool GroupControlsWidget::planRequest() {
    ROS_DEBUG("GroupControlsWidget::planRequest()");    
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::PLAN_TO_MARKER;
    fillPlanRequest(srv);
    if (service_client_->call(srv)) {
        ROS_DEBUG("GroupControlsWidget::planRequest() -- success");
        return setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::planRequest() -- failed to call service");
        return false;
    }
}


void GroupControlsWidget::fillPlanRequest(robot_interaction_tools_msgs::ConfigureGroup &srv) {
    srv.request.group_configuration.group_name = group_name;
    // std::string viz_type = ui->viz_type->currentText().toStdString();
    // srv.request.group_configuration.path_visualization_mode = viz_type;
    srv.request.group_configuration.execute_on_plan = ui->execute_on_plan->checkState()==Qt::Checked;
    srv.request.group_configuration.group_type = group_type;

    // robot_interaction_tools_msgs::ToleranceInfo pos_tol_info;
    // pos_tol_info.mode = "position";
    // pos_tol_info.types.push_back(ui->pos_tol->currentText().toStdString());

    // robot_interaction_tools_msgs::ToleranceInfo rot_tol_info;
    // rot_tol_info.mode = "orientation";
    // rot_tol_info.types.push_back(ui->rot_tol->currentText().toStdString());

    //srv.request.group_configuration.tolerance.push_back(pos_tol_info);
    //srv.request.group_configuration.tolerance.push_back(rot_tol_info);

    srv.request.group_configuration.conditioning_metric = ui->conditioning_metric->currentText().toStdString();

    robot_interaction_tools_msgs::JointMask jm;
    for (int jdx=0; jdx<ui->joint_list->count(); jdx++) {
        QListWidgetItem *item = ui->joint_list->item(jdx);       
        joint_mask[jdx] = (item->checkState()==Qt::Checked);
        last_sent_joint_mask[jdx] = joint_mask[jdx];
        jm.mask.push_back(joint_mask[jdx]);
    }
    srv.request.group_configuration.joint_mask = jm; 
}

bool GroupControlsWidget::executeRequest() {
    ROS_DEBUG("GroupControlsWidget::executeRequest()");    
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::EXECUTE_PLAN;
    srv.request.group_configuration.group_name = group_name;
    if (service_client_->call(srv)) {
        ROS_DEBUG("GroupControlsWidget::executeRequest() -- success");
        return setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::executeRequest() -- failed to call service");
        return false;
    }
}


// bool GroupControlsWidget::toggleJointControlRequest() {
//     ROS_DEBUG("GroupControlsWidget::toggleJointControlRequest()");    
//     robot_interaction_tools_msgs::ConfigureGroup srv;
//     srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::TOGGLE_POSTURE_CONTROLS;
//     srv.request.group_configuration.group_name = group_name;
//     if (service_client_->call(srv)) {
//         ROS_DEBUG("GroupControlsWidget::toggleJointControlRequest() -- success");
//         return setGroupDataFromResponse(srv.response);
//     } else {
//         ROS_ERROR("GroupControlsWidget::toggleJointControlRequest() -- failed to call service");
//         return false;
//     }
// }

void GroupControlsWidget::setToolOffsetClicked() {
    ROS_DEBUG("GroupControlsWidget::setToolOffsetClicked()");    
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::SET_TOOL_OFFSET;
    srv.request.group_configuration.group_name = group_name;
    if (service_client_->call(srv)) {
        ROS_DEBUG("GroupControlsWidget::setToolOffsetClicked() -- success");
        setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::setToolOffsetClicked() -- failed to call service");
    }
}

void GroupControlsWidget::clearToolOffsetClicked() {
    ROS_DEBUG("GroupControlsWidget::clearToolOffsetClicked()");    
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::CLEAR_TOOL_OFFSET;
    srv.request.group_configuration.group_name = group_name;
    if (service_client_->call(srv)) {
        ROS_DEBUG("GroupControlsWidget::clearToolOffsetClicked() -- success");
        setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::clearToolOffsetClicked() -- failed to call service");
    }
}

bool GroupControlsWidget::storedPoseRequest() {
    ROS_DEBUG("GroupControlsWidget::storedPoseRequest()");    
    robot_interaction_tools_msgs::ConfigureGroup srv;
    srv.request.action = robot_interaction_tools_msgs::ConfigureGroupRequest::EXECUTE_STORED_POSE;
    srv.request.group_configuration.group_name = group_name;
    std::string stored_pose = ui->stored_pose_list->currentText().toStdString();
    srv.request.group_configuration.stored_pose = stored_pose;
    if (service_client_->call(srv)) {
        ROS_DEBUG("GroupControlsWidget::storedPoseRequest() -- success");
        setGroupDataFromResponse(srv.response);
    } else {
        ROS_ERROR("GroupControlsWidget::storedPoseRequest() -- failed to call service");
        return false;
    }
    return true;
}

