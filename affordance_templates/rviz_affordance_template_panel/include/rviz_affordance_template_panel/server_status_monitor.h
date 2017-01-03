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

#ifndef AFFORDANCE_TEMPLATE_SERVER_STATUS_MONITOR_HPP
#define AFFORDANCE_TEMPLATE_SERVER_STATUS_MONITOR_HPP

#include <ros/ros.h>

#include <boost/thread.hpp> 
#include <boost/thread/mutex.hpp>

#include <rviz_affordance_template_panel/msg_headers.h>

namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel {

  class AffordanceTemplateServerStatusMonitor  {

    public:

      AffordanceTemplateServerStatusMonitor(ros::NodeHandle &nh, std::string srv_name, int update_rate=1);
      ~AffordanceTemplateServerStatusMonitor();

      void start();
      void stop();

      inline bool isReady() { return ready_; }
      inline bool isAvailable() { return available_; }

    protected:

      void run_function();
      void wait(int seconds);

      // boost thread
      boost::scoped_ptr<boost::thread> monitor_thread_;
      boost::mutex mutex;

      // ros stuff
      ros::ServiceClient srv_;
      std::string srv_name_;
      ros::NodeHandle nh_;

      // member functions
      int update_rate_;

      // status vars
      bool available_;
      bool ready_;
      bool running_;
  };
}

#endif // AFFORDANCE_TEMPLATE_SERVER_STATUS_MONITOR_HPP