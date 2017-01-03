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

#ifndef _AT_GENERICS_H_
#define _AT_GENERICS_H_

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <planner_interface/planner_interface.h>

namespace affordance_template_object
{
    struct Origin
    {
        double position[3]; // xyz
        double orientation[3]; // rpy
    };

    struct ToleranceBounds
    {
        double position[3][2]; // xyz [lo,hi]
        double orientation[3][2]; // rpy [lo,hi]
    };

    struct TaskCompatibility
    {
        int position[3]; // xyz
        int orientation[3]; // rpy
    };

    inline geometry_msgs::Pose originToPoseMsg(Origin origin) 
    { 
      geometry_msgs::Pose p;
      p.position.x = origin.position[0];
      p.position.y = origin.position[1];
      p.position.z = origin.position[2];
      p.orientation = tf::createQuaternionMsgFromRollPitchYaw(origin.orientation[0],origin.orientation[1],origin.orientation[2]);
      return p;
    }

    inline Origin poseMsgToOrigin(geometry_msgs::Pose p) 
    { 
      Origin origin;
      tf::Quaternion q;
      double roll, pitch, yaw;

      tf::quaternionMsgToTF(p.orientation, q);
      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

      origin.position[0] = p.position.x;
      origin.position[1] = p.position.y;
      origin.position[2] = p.position.z;
      origin.orientation[0] = roll;
      origin.orientation[1] = pitch;
      origin.orientation[2] = yaw;

      return origin;
    }

    inline geometry_msgs::Pose taskCompatibilityToPoseMsg(TaskCompatibility tc) 
    { 
      geometry_msgs::Pose p;
      p.position.x = tc.position[0];
      p.position.y = tc.position[1];
      p.position.z = tc.position[2];
      p.orientation = tf::createQuaternionMsgFromRollPitchYaw(tc.orientation[0],tc.orientation[1],tc.orientation[2]);
      return p;
    }

    inline planner_interface::PlannerType stringToPlannerType(std::string planner_type) {
        if(planner_type == "CARTESIAN") {
            return planner_interface::CARTESIAN;
        } else {
            return planner_interface::JOINT;  
        }
    }    

    struct Shape
    {
        std::string color;
        double rgba[4];
        std::string type;
        std::string mesh; // if type == "mesh" then we need to fill the file name in here
        double size[3]; // for boxes and meshes
        double length; // for cylinders
        double radius; // for spherse and cylinders
    };

    inline std::string toBoolString(bool b) { return (b ? "true" : "false"); }

    struct Control
    {
        bool translation[3]; // xyz
        bool rotation[3]; // rpy
        double scale;
    };
}

#endif