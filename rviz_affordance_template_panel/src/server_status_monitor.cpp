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

#include <rviz_affordance_template_panel/server_status_monitor.h>

using namespace rviz_affordance_template_panel;

AffordanceTemplateServerStatusMonitor::AffordanceTemplateServerStatusMonitor(ros::NodeHandle &nh, std::string srv_name, int update_rate) :
  nh_(nh),
  srv_name_(srv_name),
  update_rate_(update_rate),
  available_(false),
  ready_(false),
  running_(false)
{
  srv_ = nh_.serviceClient<affordance_template_msgs::GetAffordanceTemplateServerStatus>(srv_name_);
}

AffordanceTemplateServerStatusMonitor::~AffordanceTemplateServerStatusMonitor() {
  stop();
}

void AffordanceTemplateServerStatusMonitor::start() {
  ROS_INFO("AffordanceTemplateServerStatusMonitor::start()");
  monitor_thread_.reset(new boost::thread(boost::bind(&AffordanceTemplateServerStatusMonitor::run_function, this)));
}

void AffordanceTemplateServerStatusMonitor::stop() {
  ROS_INFO("AffordanceTemplateServerStatusMonitor::stop()");
  running_ = false;
  monitor_thread_->join();
}

void AffordanceTemplateServerStatusMonitor::wait(int seconds) { 
  boost::this_thread::sleep(boost::posix_time::seconds(seconds)); 
} 

void AffordanceTemplateServerStatusMonitor::run_function() {

  ROS_DEBUG("AffordanceTemplateServerStatusMonitor::run_function() -- updating");
  
  running_   = true;
  available_ = false;
  ready_     = false;
  
  while(running_) {

    boost::unique_lock<boost::mutex> scoped_lock(mutex);

    available_ = false;
    ready_     = false;
    
    if(srv_.exists()) {

      available_ = true;
      ready_ = false;
      
      ROS_DEBUG("AffordanceTemplateServerStatusMonitor::run_function() -- service exists");
      affordance_template_msgs::GetAffordanceTemplateServerStatus server_status;
      
      if (srv_.call(server_status)) {
        ready_ = server_status.response.ready;
        ROS_DEBUG("AffordanceTemplateServerStatusMonitor::run_function() -- got response: %d", (int)(server_status.response.ready)); 
      } else {
        ROS_DEBUG("AffordanceTemplateServerStatusMonitor::run_function() -- service call failed");
      }
    } else {
      ROS_DEBUG("AffordanceTemplateServerStatusMonitor::run_function() -- service does not exist");
    }

    wait(update_rate_);
  }
  ROS_DEBUG("AffordanceTemplateServerStatusMonitor::run_function() -- not running anymore");
}