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

#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

namespace robot_display
{
    class PredictiveDisplay
    {
        void animate(); // animate queue thread
        void currentJointsCallback(const sensor_msgs::JointState&);

        bool initialized_;
        bool animation_ready_;

        std::string current_group_;

        ros::NodeHandle nh_;
        ros::Subscriber current_joints_sub_;
        ros::Publisher  predictive_joints_pub_;
        
        sensor_msgs::JointState start_state_;
        sensor_msgs::JointState current_state_;

        std::vector<std::string> joint_names_;
        std::map<std::string, std::vector<std::string> > group_joints_;
        std::map<std::string, bool> group_loop_;

        std::vector<sensor_msgs::JointState> plan_queue_;
        std::vector<sensor_msgs::JointState> animate_queue_;
        
        std::map<std::string, std::vector<sensor_msgs::JointState> > group_animations_;

        boost::scoped_ptr<boost::thread> animate_thread_;
        boost::mutex animate_mutex_;

    public: 
        PredictiveDisplay(const ros::NodeHandle, const std::string _sub="joint_states", const std::string _pub="predictive");
        ~PredictiveDisplay();
        
        void setLoop(const std::string&, const bool);
        void playPlan();
        void resetDisplay(bool all=false); // needed when switching trajectories, etc
        void setStartState();
        void createAnimation(const std::string&, const trajectory_msgs::JointTrajectory&, bool autoplay=true);
    };
}