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

std::deque<sensor_msgs::JointState> state_queue_;
sensor_msgs::JointState start_state_, zero_state_;
bool flip_;
bool state_set_;

void stateCb(const sensor_msgs::JointState& state)
{
    if (!state_set_)
    {
        start_state_ = state;
        zero_state_ = start_state_;
        for (unsigned int i=0; i < zero_state_.name.size(); ++i)
            if (zero_state_.name[i].find("arm") != std::string::npos)
                zero_state_.position[i] = 0.0;
        state_set_ = true;
    }

    state_queue_.push_back(state);
}

int main(int argc, char** argv)
{
    flip_ = false;
    state_set_ = false;

    ros::init(argc, argv, "predicted_display_test");
    ros::NodeHandle nh;
    ros::Subscriber js_sub = nh.subscribe("joint_states", 1, &stateCb);
    ros::Publisher display_pub = nh.advertise<sensor_msgs::JointState>("predictive_joint_states", 1);

    while (ros::ok())
    {
        if (state_queue_.size())
        {
            state_queue_.pop_front();

            sensor_msgs::JointState st = zero_state_;
            if (flip_)
                st = start_state_;
            
            //get arm joints
            std::map<std::string, double> arm_joints;
            for (unsigned int i = 0; i < st.name.size(); ++i)
                if (st.name[i].find("left_arm") != std::string::npos || st.name[i].find("right_arm") != std::string::npos)
                    arm_joints[st.name[i]] = (start_state_.position[i] - zero_state_.position[i])/10;

            // take 10 steps to go from ready pose to home back to ready pose
            for (int cnt = 0; cnt < 10; ++cnt)
            {
                for (unsigned int i = 0; i < st.name.size(); ++i)
                    if (arm_joints.find(st.name[i]) != arm_joints.end())
                        if (!flip_)
                            st.position[i] += arm_joints[st.name[i]];
                        else 
                            st.position[i] -= arm_joints[st.name[i]];
                
                st.header.stamp = ros::Time::now() + ros::Duration(0.5);
                display_pub.publish(st);
                ros::Duration(0.1).sleep();
            }

            flip_ = !flip_;
        }
        ros::Duration(1).sleep();
        ros::spinOnce();
    }

    return 0;
}

