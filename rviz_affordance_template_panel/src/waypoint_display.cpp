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

#include <rviz_affordance_template_panel/waypoint_display.h>

using namespace rviz_affordance_template_panel;
using namespace std;

WaypointDisplay::WaypointDisplay(QObject *_parent) : QObject(_parent) {}


void WaypointDisplay::setupWaypointDisplayInfo(AffordanceTemplateStatusInfo::EndEffectorInfo wp_info) {

    ROS_DEBUG("WaypointDisplay::setupWaypointDisplayInfo()");
           
    ui_->waypointDisplayTree->clear();
    eeNameMap_.clear();

    for(auto &wp : wp_info) {

        for (auto& e: (*robotMap_[robotName_]).endeffectorMap) {
            std::string ee_name = e.second->name();
            if (ee_name != wp.first) {
                int n = wp_info[ee_name]->num_waypoints;
                if(n > 0) {
                    //cout << "wpi[" << ee_name << "]: " << wp_info[ee_name]->compact_view.size() << endl;
                    QTreeWidgetItem *eeTreeItem = new QTreeWidgetItem(ui_->waypointDisplayTree);
                    eeTreeItem->setText(0, QString(ee_name.c_str()));
                    eeTreeItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
                    eeTreeItem->setCheckState(1,Qt::Unchecked);
                    // eeTreeItem->setExpanded(true);
                    ROS_DEBUG("WaypointDisplay::setupWaypointDisplayInfo() -- creating %d waypoint display boxes for %s", n, ee_name.c_str());
                    for(int i=0; i<n; i++) {
                        QTreeWidgetItem *wpTreeItem = new QTreeWidgetItem();
                        wpTreeItem->setText(0, QString(std::to_string(i).c_str()));
                        wpTreeItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
                        if(wp_info[ee_name]->compact_view.size() == n) {
                            if(wp_info[ee_name]->compact_view[i]) {
                                wpTreeItem->setCheckState(1,Qt::Checked);
                            } else {
                                wpTreeItem->setCheckState(1,Qt::Unchecked);                            
                            }
                        } else {
                            wpTreeItem->setCheckState(1,Qt::Unchecked);                            
                        }
                        eeTreeItem->addChild(wpTreeItem);
                        auto p = std::make_pair(ee_name, i);
                        waypointDisplayItems_[p] = wpTreeItem;
                        eeNameMap_[ee_name] = e.second->id();
                    }
                }
            }
        }
    }
}

void WaypointDisplay::displayEventChecked(QTreeWidgetItem *item, int i) {
    std::string str = item->text(0).toStdString();
    auto search = eeNameMap_.find(str);
    std::vector<std::string> wp_names;
    std::vector<bool> modes; 
    if(search != eeNameMap_.end()) {
        std::string ee_name = item->text(0).toStdString();
        for(auto &wpItem: waypointDisplayItems_) {
            QTreeWidgetItem *wpTreeItem = wpItem.second;
            auto p = wpItem.first; 
            if(p.first == str) {
                wpItem.second->setCheckState(1,item->checkState(i));  // swhart: surprised this doesnt call the callback. why not? 
                QString wpItemStr = wpItem.second->text(0);
                std::string wp_id = wpItemStr.toStdString();
                modes.push_back(item->checkState(i) == Qt::Checked);
                std::string wp_name = std::to_string(eeNameMap_[ee_name]) + "." + wp_id + ":" + template_status_->getName() + ":" + std::to_string(template_status_->getID());
                wp_names.push_back(wp_name);
            }  
        }
    } else {
        QTreeWidgetItem *eeTreeItem = item->parent();
        QString eeItemStr = eeTreeItem->text(0);
        QString wpItemStr = item->text(0);
        std::string ee_name = eeItemStr.toStdString();
        std::string wp_id = wpItemStr.toStdString();
        bool compact_mode = item->checkState(i) == Qt::Checked;
        std::string wp_name = std::to_string(eeNameMap_[ee_name]) + "." + wp_id + ":" + template_status_->getName() + ":" + std::to_string(template_status_->getID());
        wp_names.push_back(wp_name);
        modes.push_back(compact_mode);
    }
    callWaypointDisplayService(wp_names, modes);
}


void WaypointDisplay::callWaypointDisplayService(std::vector<std::string> wp_names, std::vector<bool> modes) {
    if(wp_names.size() != modes.size()) {
        ROS_ERROR("WaypointDisplay::callWaypointDisplayService() size mismatch!");
    }
    affordance_template_msgs::SetWaypointViewModes srv;
    for(size_t idx=0; idx<wp_names.size(); idx++) {
        srv.request.waypoint_names.push_back(wp_names[idx]);
        srv.request.compact_view.push_back(modes[idx]);
    }
    if (setWaypointDisplayService_.call(srv))
        ROS_INFO("WaypointDisplay::callWaypointDisplayService() -- set waypoint display mode successful");
    else
        ROS_ERROR("WaypointDisplay::callWaypointDisplayService() -- Failed to set waypoint display mode");
}
