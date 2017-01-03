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

#ifndef MARKER_HELPER_H_
#define MARKER_HELPER_H_

#include <ros/ros.h>
#include <tf/tf.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <math.h>

using namespace visualization_msgs;

namespace rit_utils {
  class MarkerHelper {

  public: 
    
    MarkerHelper() {}
    ~MarkerHelper() {}

    static InteractiveMarkerControl makeXTransControl() {
      InteractiveMarkerControl control;
      control.orientation.w = 1;
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;
      control.name = "move_x";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      return control;
    }

    static InteractiveMarkerControl makeYTransControl() {
      InteractiveMarkerControl control;
      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.name = "move_y";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      return control;
    }

    static InteractiveMarkerControl makeZTransControl() {
      InteractiveMarkerControl control;
      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 0;
      control.orientation.z = 1;
      control.name = "move_z";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      return control;
    }

    static InteractiveMarkerControl makeXRotControl() {
      InteractiveMarkerControl control;
      control.orientation.w = 1;
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;
      control.name = "rotate_x";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      return control;
    }

    static InteractiveMarkerControl makeYRotControl() {
      InteractiveMarkerControl control;
      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.name = "rotate_y";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      return control;
    }

    static InteractiveMarkerControl makeZRotControl() {
      InteractiveMarkerControl control;
      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 0;
      control.orientation.z = 1;
      control.name = "rotate_z";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      return control;
    }

    static std::vector<InteractiveMarkerControl> make6DOFControls() {
      std::vector<InteractiveMarkerControl> controls;
      controls.push_back(makeXTransControl());
      controls.push_back(makeYTransControl());
      controls.push_back(makeZTransControl());
      controls.push_back(makeXRotControl());
      controls.push_back(makeYRotControl());
      controls.push_back(makeZRotControl());
      return controls;
    }

    static std::vector<InteractiveMarkerControl> makeCustomDOFControls(bool x, bool y, bool z, bool R, bool P, bool Y) {
      std::vector<InteractiveMarkerControl> controls;
      if(x) 
        controls.push_back(makeXTransControl());
      if(y) 
        controls.push_back(makeZTransControl());
      if(z) 
        controls.push_back(makeYTransControl());
      if(R) 
        controls.push_back(makeXRotControl());
      if(P) 
        controls.push_back(makeZRotControl());
      if(Y) 
        controls.push_back(makeYRotControl());
      return controls;
    }

    static Marker makeBox( InteractiveMarker &msg )
    {
      Marker marker;

      marker.type = Marker::CUBE;
      marker.scale.x = msg.scale * 0.45;
      marker.scale.y = msg.scale * 0.45;
      marker.scale.z = msg.scale * 0.45;
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
      marker.color.a = 1.0;

      return marker;
    }

    static InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
    {
      InteractiveMarkerControl control;
      control.always_visible = true;
      control.markers.push_back( makeBox(msg) );
      msg.controls.push_back( control );
    
      return msg.controls.back();
    }

    static void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, 
        bool show_6dof, std::string frame_id="base_link", std::string name="simple_6dof", std::string description="Simple 6DOF Control" )
    {
      InteractiveMarker int_marker;
      int_marker.header.frame_id = "base_link";
      tf::pointTFToMsg(position, int_marker.pose.position);
      int_marker.scale = 1;

      int_marker.name = "simple_6dof";
      int_marker.description = "Simple 6-DOF Control";

      // insert a box
      makeBoxControl(int_marker);
      int_marker.controls[0].interaction_mode = interaction_mode;

      InteractiveMarkerControl control;

      if ( fixed )
      {
        int_marker.name += "_fixed";
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
      }

      if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
      {
          std::string mode_text;
          if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
          if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
          if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
          int_marker.name += "_" + mode_text;
          int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
      }

      if(show_6dof)
      {
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
      }

    };

  };

};

#endif
