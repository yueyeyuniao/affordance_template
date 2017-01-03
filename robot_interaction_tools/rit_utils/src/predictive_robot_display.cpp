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

#include <rit_utils/predictive_robot_display.h>

using namespace robot_display;

PredictiveDisplay::PredictiveDisplay(const ros::NodeHandle _nh, 
                                     const std::string _sub, 
                                     const std::string _prefix):
    nh_(_nh)
{
    ROS_INFO("[PredictiveDisplay] creating predictive robot display");

    current_group_ = "";
    initialized_ = false;
    animation_ready_ = false;

    ros::NodeHandle nh; // FIXME!!!!!!! @seth 1/6
    current_joints_sub_ = nh.subscribe(_sub, 1, &PredictiveDisplay::currentJointsCallback, this);
    predictive_joints_pub_ = nh.advertise<sensor_msgs::JointState>(_prefix + "/joint_states", 1);
    
    animate_thread_.reset(new boost::thread(boost::bind(&PredictiveDisplay::animate, this)));

    ROS_INFO("[PredictiveDisplay] robot display will publish on %s/joint_states", _prefix.c_str());
}


PredictiveDisplay::~PredictiveDisplay() 
{
    animate_thread_->join();
}


void PredictiveDisplay::animate()
{
    ROS_INFO("[PredictiveDisplay] starting animation thread");

    sensor_msgs::JointState animate_state;
    ros::Rate display_rate_(5);
    while (ros::ok()) {
        animate_mutex_.lock();
        for (auto a : animate_queue_) {
            if (animation_ready_)
                break;

            animate_state = a;

            animate_state.header.stamp = ros::Time::now();
            predictive_joints_pub_.publish(animate_state);
            ros::spinOnce();
            display_rate_.sleep();
        }

        if (!group_loop_[current_group_]) {
            animate_queue_.clear();
            animate_queue_.push_back(animate_state);
        }
        animate_mutex_.unlock();

        ros::spinOnce();
    }
}


void PredictiveDisplay::currentJointsCallback(const sensor_msgs::JointState& state)
{
    if (!initialized_) {
        start_state_ = state;
        joint_names_ = state.name;

        boost::mutex::scoped_lock lock(animate_mutex_);
        animate_queue_.clear();
        animate_queue_.push_back(state);

        initialized_ = true;
    }

    current_state_ = state;
}


void PredictiveDisplay::setStartState()
{
    start_state_ = current_state_;
}


void PredictiveDisplay::resetDisplay(bool all)
{
    animation_ready_ = true;
    boost::mutex::scoped_lock lock(animate_mutex_);
    animate_queue_.clear();
    animate_queue_.push_back(current_state_);
    animation_ready_ = false;

    if (all) {
        plan_queue_.clear();
        group_animations_.clear();
    }
}


void PredictiveDisplay::setLoop(const std::string& group, const bool set) 
{ 
    ROS_INFO("[PredictiveDisplay::setLoop] animation WILL%sloop when planning for group %s",( set ? " " : " not "), group.c_str());
    group_loop_[group] = set; 
}


// convert the trajectory points to joint states
void PredictiveDisplay::createAnimation(const std::string& group, const trajectory_msgs::JointTrajectory& traj, bool autoplay)
{
    if ((int)traj.points.size() == 0) {
        ROS_ERROR("[PredictiveDisplay::createAnimation] no planned states to display!! Look to the planner for errors..");
        return;
    }

    ROS_INFO("[PredictiveDisplay::createAnimation] creating new animation for group %s that will %sautoplay %d points", group.c_str(), (autoplay ? "" : "NOT "), (int)traj.points.size());

    current_group_ = group;

    // figure out our state to build on
    sensor_msgs::JointState state;
    group_joints_[group] = traj.joint_names;
    if (animate_queue_.size() > 1 || plan_queue_.size() > 0) {
        if (animate_queue_.size() > 1)
            state = animate_queue_.back();
        else
            state = plan_queue_.back();

        // clear out stale data
        if (plan_queue_.size() == 1)
            plan_queue_.clear();

        for (auto g : group_animations_) {
            sensor_msgs::JointState last_state = g.second.back();
            for (int i = 0; i < (int)state.name.size(); ++i) {
                if (std::find(group_joints_[g.first].begin(), group_joints_[g.first].end(), state.name[i]) != group_joints_[g.first].end()) {
                    state.position[i] = last_state.position[i];
                }
            }
        }

    } else {
        state = current_state_;
    }

    state.header = traj.header;
    std::vector<sensor_msgs::JointState> traj_states;
    for (unsigned int i = 0; i < traj.points.size(); ++i) {
        for (unsigned int j = 0; j < traj.points[i].positions.size(); ++j) {
            for (unsigned int k = 0; k < state.name.size(); ++k) {
                if (state.name[k] == traj.joint_names[j])
                   state.position[k] = traj.points[i].positions[j];
            }
        }
        traj_states.push_back(state);
    }

    // if (append)
    //     group_animations_[group].insert(group_animations_[group].end(), traj_states.begin(), traj_states.end());
    // else
    group_animations_[group] = traj_states;

    if (autoplay) {
        animation_ready_ = true;
        boost::mutex::scoped_lock lock(animate_mutex_);

        // if (append) {
        //     animate_queue_.insert(animate_queue_.end(), traj_states.begin(), traj_states.end());
        // } else {
            animate_queue_.clear();
            animate_queue_ = traj_states;
        // }

        animation_ready_ = false;
    }
    
    plan_queue_.insert(plan_queue_.end(), traj_states.begin(), traj_states.end());
    ROS_INFO("[PredictiveDisplay::createAnimation] available predictive display now sized %d", (int)plan_queue_.size());
}


void PredictiveDisplay::playPlan()
{
    if (plan_queue_.size() == 0) {
        ROS_WARN("[PredictiveDisplay::playPlan] no plan available to display");
        return;
    }
 
    ROS_INFO("[PredictiveDisplay::playPlan] playing or replaying %d robot states", (int)plan_queue_.size());

    animation_ready_ = true;
    boost::mutex::scoped_lock lock(animate_mutex_);
    animate_queue_.clear();
    animate_queue_ = plan_queue_;
    animation_ready_ = false;
}
