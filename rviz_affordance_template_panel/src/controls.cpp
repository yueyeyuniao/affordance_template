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

#include <rviz_affordance_template_panel/controls.h>

using namespace rviz_affordance_template_panel;
using namespace std;

Controls::Controls() {}


bool Controls::requestPlan(Controls::CommandType command_type, bool exe_on_plan) 
{
    affordance_template_msgs::AffordanceTemplatePlanCommand srv;
    string key = ui_->control_template_box->currentText().toUtf8().constData();
    if(key=="") return false;

    vector<string> stuff = util::split(key, ':');
    srv.request.type = stuff[0];
    srv.request.id = int(atoi(stuff[1].c_str()));
    srv.request.trajectory_name = template_status_->getCurrentTrajectory();
    srv.request.backwards = (command_type==CommandType::STEP_BACKWARD);
    srv.request.execute = exe_on_plan;

    vector<pair<string,int> > ee_info = getSelectedEndEffectorInfo();
    for(auto &ee : ee_info) {
        if(!template_status_->endEffectorInTrajectory(template_status_->getCurrentTrajectory(), ee.first)) {
            continue;
        }
        srv.request.end_effectors.push_back(ee.first);
    }
  
    if(command_type==CommandType::CURRENT) {
        for(auto &ee : ee_info) {
            if(!template_status_->endEffectorInTrajectory(template_status_->getCurrentTrajectory(), ee.first)) {
                continue;
            }
            if (template_status_->getTrajectoryInfo().find(srv.request.trajectory_name) == template_status_->getTrajectoryInfo().end()) {
                ROS_ERROR("Controls::requestPlan() -- trajectory \'%s\' not found in template status", srv.request.trajectory_name.c_str());
                return false;
            }

            if (template_status_->getTrajectoryInfo()[srv.request.trajectory_name].find(ee.first) == template_status_->getTrajectoryInfo()[srv.request.trajectory_name].end()) {
                ROS_ERROR("Controls::requestPlan() -- end-effector \'%s\' not found in template status for traj \'%s\'", ee.first.c_str(),srv.request.trajectory_name.c_str());
                return false;
            }
            srv.request.steps.push_back(-1);
        }
    } else if(command_type==CommandType::START || command_type==CommandType::END) {
     
        srv.request.direct = true;
        for(auto &ee : ee_info) {

            if(!template_status_->endEffectorInTrajectory(template_status_->getCurrentTrajectory(), ee.first)) {
                continue;
            }
            
            if (template_status_->getTrajectoryInfo().find(srv.request.trajectory_name) == template_status_->getTrajectoryInfo().end()) {
                ROS_ERROR("Controls::requestPlan() -- trajectory \'%s\' not found in template status", srv.request.trajectory_name.c_str());
                return false;
            }
 
            if (template_status_->getTrajectoryInfo()[srv.request.trajectory_name].find(ee.first) == template_status_->getTrajectoryInfo()[srv.request.trajectory_name].end()) {
                ROS_ERROR("Controls::requestPlan() -- end-effector \'%s\' not found in template status for traj \'%s\'", ee.first.c_str(),srv.request.trajectory_name.c_str());
                return false;
            }
            template_status_->getTrajectoryInfo()[srv.request.trajectory_name][ee.first];
 
            int idx = template_status_->getTrajectoryInfo()[srv.request.trajectory_name][ee.first]->waypoint_index;
            int N = template_status_->getTrajectoryInfo()[srv.request.trajectory_name][ee.first]->num_waypoints;
            int steps = 0;

            if(command_type==CommandType::START) {
                steps = 1;
                if(idx != -1) {
                    srv.request.backwards = true;
                }
            } else if(command_type==CommandType::END) {
                steps = N;
                if (steps == 0) {
                    ROS_WARN("Controls::requestPlan() -- 0 steps for EE %s in the %s trajectory - skipping", ee.first.c_str(), srv.request.trajectory_name.c_str());
                    continue;
                }
                if(idx !=-1) {
                    srv.request.backwards = false;
                }
            }
            srv.request.steps.push_back(steps);
        }
                
    } else {
        for(auto &ee : ee_info) {
            int steps = ui_->num_steps->text().toInt();
            srv.request.steps.push_back(steps);
        }
    }
              
    if (planService_.call(srv))
    {
        ROS_INFO("PLAN command successful, returned status: %d", (int)(srv.response.status)); // FIXME
        affordance_template_msgs::AffordanceTemplateStatusConstPtr ptr(new affordance_template_msgs::AffordanceTemplateStatus(srv.response.affordance_template_status));
        bool r = template_status_->updateTrajectoryStatus(ptr);
        if (!r)
            ROS_ERROR("Controls::requestPlan() -- error updating template status");
        return r;
    }
    else
    {
        ROS_ERROR("Failed to call plan service command");
        return false;
    }                
}


bool Controls::executePlan() {

    affordance_template_msgs::AffordanceTemplateExecuteCommand srv;
    string key = ui_->control_template_box->currentText().toUtf8().constData();
    if(key=="") return false;

    ROS_INFO("Sending Execute command request for a %s", key.c_str());      

    vector<string> stuff = util::split(key, ':');
    srv.request.type = stuff[0];
    srv.request.id = int(atoi(stuff[1].c_str()));
    srv.request.trajectory_name = template_status_->getCurrentTrajectory();
    
    vector<pair<string,int> > ee_info = getSelectedEndEffectorInfo();
    for(auto &ee : ee_info) {
        srv.request.end_effectors.push_back(ee.first);
    }
    
    if (executeService_.call(srv))
    {
        ROS_INFO("EXECUTE command successful, returned status: %d", (int)(srv.response.status)); // FIXME
        affordance_template_msgs::AffordanceTemplateStatusConstPtr ptr(new affordance_template_msgs::AffordanceTemplateStatus(srv.response.affordance_template_status));
        bool r = template_status_->updateTrajectoryStatus(ptr);
        if(!r) {
            ROS_ERROR("Controls::executePlan() -- error updating template status");
        }
        return r;
    }
    else
    {
        ROS_ERROR("Failed to call execute service command");
        return false;
    }
}

vector<pair<string,int> > Controls::getSelectedEndEffectorInfo() {
    vector<pair<string,int> > selectedEndEffectorInfo;
    for (int r=0; r<ui_->end_effector_table->rowCount(); r++ ) {
        if (ui_->end_effector_table->item(r,1)->checkState() == Qt::Checked ) {
            string ee_name = ui_->end_effector_table->item(r,0)->text().toStdString();
            int ee_idx = ui_->end_effector_table->item(r,2)->text().toInt();
            pair<string,int> ee_info = make_pair(ee_name, ee_idx);
            selectedEndEffectorInfo.push_back(ee_info);
        }
    }
    return selectedEndEffectorInfo;
}