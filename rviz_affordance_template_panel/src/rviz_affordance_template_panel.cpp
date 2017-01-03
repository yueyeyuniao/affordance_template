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

#include <rviz_affordance_template_panel/rviz_affordance_template_panel.h>

using namespace rviz_affordance_template_panel;
using namespace std;

RVizAffordanceTemplatePanel::RVizAffordanceTemplatePanel(QWidget *parent) :
    rviz::Panel(parent),
    ui_(new Ui::RVizAffordanceTemplatePanel)
{
    // Setup the panel.
    ui_->setupUi(this);

    // setup QT widgets
    setupWidgets();

    // start up client thread
    client_ = new AffordanceTemplateRVizClient(nh_, ui_, affordanceTemplateGraphicsScene_);
    client_->start();
}


RVizAffordanceTemplatePanel::~RVizAffordanceTemplatePanel()
{
    client_->stop();
    delete client_;
    delete ui_;
}

void RVizAffordanceTemplatePanel::setupWidgets() {

    affordanceTemplateGraphicsScene_ = new QGraphicsScene(this);
    ui_->affordanceTemplateGraphicsView->setScene(affordanceTemplateGraphicsScene_);

    QObject::connect(affordanceTemplateGraphicsScene_, SIGNAL(selectionChanged()), this, SLOT(addAffordanceDisplayItem()));

    QObject::connect(ui_->server_output_status, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(selectAffordanceTemplate(QListWidgetItem*)));
    QObject::connect(ui_->delete_template_button, SIGNAL(clicked()), this, SLOT(deleteAffordanceTemplate()));
    QObject::connect(ui_->save_as_button, SIGNAL(clicked()), this, SLOT(saveAffordanceTemplate()));
    QObject::connect(ui_->add_traj_button, SIGNAL(clicked()), this, SLOT(addTrajectory()));

    QObject::connect(ui_->load_config_button, SIGNAL(clicked()), this, SLOT(safeLoadConfig()));
    QObject::connect(ui_->robot_select, SIGNAL(currentIndexChanged(int)), this, SLOT(changeRobot(int)));
    QObject::connect(ui_->save_template_combo_box, SIGNAL(activated(int)), this, SLOT(changeSaveInfo(int)));

    QObject::connect(ui_->go_to_start_button, SIGNAL(clicked()), this, SLOT(goToStart()));
    QObject::connect(ui_->go_to_end_button, SIGNAL(clicked()), this, SLOT(goToEnd()));
    QObject::connect(ui_->step_backwards_button, SIGNAL(clicked()), this, SLOT(stepBackward()));
    QObject::connect(ui_->step_forward_button, SIGNAL(clicked()), this, SLOT(stepForward()));
    QObject::connect(ui_->execute_button, SIGNAL(clicked()), this, SLOT(executePlan()));
    QObject::connect(ui_->status_update_button, SIGNAL(clicked()), this, SLOT(controlStatusUpdate()));
    QObject::connect(ui_->go_to_current_waypoint_button, SIGNAL(clicked()), this, SLOT(goToCurrentWaypoint()));
    QObject::connect(ui_->control_trajectory_box, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(selectTemplateTrajectory(const QString&)));

    QObject::connect(ui_->refresh_button, SIGNAL(clicked()), this, SLOT(refreshCallback()));
    QObject::connect(ui_->robot_lock, SIGNAL(stateChanged(int)), this, SLOT(enableConfigPanel(int)));

    QObject::connect(ui_->robot_name, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->config_package, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->config_file, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->planner_type, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->gripper_action, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    
    QObject::connect(ui_->frame_id, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->robot_tx, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->robot_ty, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->robot_tz, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->robot_rr, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->robot_rp, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->robot_ry, SIGNAL(textEdited(const QString&)), this, SLOT(updateRobotConfig(const QString&)));
    QObject::connect(ui_->ee_name, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_id, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_tx, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_ty, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_tz, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_rr, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_rp, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));
    QObject::connect(ui_->ee_ry, SIGNAL(textEdited(const QString&)), this, SLOT(updateEndEffectorGroupMap(const QString&)));

    // object scaling stuff
    QObject::connect(ui_->object_scale_slider, SIGNAL(valueChanged(int)), this, SLOT(updateObjectScale(int)));
    QObject::connect(ui_->object_scale_slider, SIGNAL(sliderReleased()), this, SLOT(scaleSliderReleased()));
    QObject::connect(ui_->end_effector_adjustment_slider, SIGNAL(sliderReleased()), this, SLOT(scaleSliderReleased()));
    QObject::connect(ui_->end_effector_adjustment_slider, SIGNAL(valueChanged(int)), this, SLOT(updateEndEffectorScaleAdjustment(int)));
    QObject::connect(ui_->reset_scale_button, SIGNAL(clicked()), this, SLOT(resetScale()));
    QObject::connect(ui_->object_scale_combo_box, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(selectScaleObject(const QString&)));
}

#include <pluginlib/class_list_macros.h>
#if ROS_VERSION_MINIMUM(1,9,41)
    PLUGINLIB_EXPORT_CLASS(rviz_affordance_template_panel::RVizAffordanceTemplatePanel, rviz::Panel)
#else
    PLUGINLIB_DECLARE_CLASS(rviz_affordance_template_panel, RVizAffordanceTemplatePanel, rviz_affordance_template_panel::RVizAffordanceTemplatePanel, rviz::Panel)
#endif


