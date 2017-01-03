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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <affordance_template_msgs/PlanAction.h>

// - new way:
//       geometry_msgs::PoseStamped pt;
//       pt = frame_store_[next_path_str + "/tf"].second;
//       goals[manipulator_name].push_back(pt);
// - or goals[manipulator_name].push_back(frame_store_[next_path_str + "/tf"].second);

// because it has to execute to the next waypoint, then execute the grasp pose, then loop if more then one step
// oh the Goal needs forward or backwards too, and if its moving straight to the first or last wp of the traj
// guess it needs the trajectory name too
// so that's the basic idea

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_actionlib");

  ros::NodeHandle nh;
  
  // TESTTTTTT
  actionlib::SimpleActionClient<affordance_template_msgs::PlanAction> ac("/affordance_template/TestWheel_0/planning_server", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, sending goal.");
  
  affordance_template_msgs::PlanGoal goal;
  goal.groups.push_back("left_hand"); // should be trajectory?? and have AT figure out which groups are in thet rajectory??
  goal.groups.push_back("right_hand");
  goal.trajectory = "Left Hand Counter Clockwise Turn";
  goal.steps = 0; // 0 to plan for all steps
  goal.planning = affordance_template_msgs::PlanGoal::CARTESIAN;
  goal.backwards = false;
  goal.execute_on_plan = true;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_ERROR("Action did not finish before the time out.");

  ros::spinOnce();

  return 0;
}