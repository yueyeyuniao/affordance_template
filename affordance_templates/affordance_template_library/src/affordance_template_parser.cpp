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

#include <affordance_template_library/affordance_template_parser.h>

using namespace affordance_template_object;

bool AffordanceTemplateParser::loadFromFile(const std::string& filename, AffordanceTemplateStructure &at)
{

  char *cstr = new char[filename.length() + 1];
  strcpy(cstr, filename.c_str());
  char* str = std::strtok(cstr, "/");

  while(std::string(str).find(".json") == std::string::npos && ros::ok())
      str = std::strtok(NULL, "/");

  std::string template_name(str);
  ROS_DEBUG("[AffordanceTemplateParser::loadFromFile] Found template name: %s", template_name.c_str());
  
  delete [] cstr;


  FILE* f_pnt = std::fopen(filename.c_str(), "r");
  char json_buff[65536];
  rapidjson::FileReadStream json(f_pnt, json_buff, sizeof(json_buff));

  rapidjson::Document jdoc;
  if (jdoc.ParseStream(json).HasParseError())
  {
      ROS_WARN("[AffordanceTemplateParser::loadFromFile] couldn't properly parse template; ignoring.");
  }
  else
  {
      ROS_DEBUG("[AffordanceTemplateParser::loadFromFile] parsing template: %s", template_name.c_str());

      at.name = jdoc["name"].GetString();
      ROS_DEBUG_STREAM("[AffordanceTemplateParser::loadFromFile] name is "<<at.name);
      at.image = jdoc["image"].GetString();
      ROS_DEBUG_STREAM("[AffordanceTemplateParser::loadFromFile] img is "<<at.image);
      at.filename = filename;
      ROS_DEBUG_STREAM("[AffordanceTemplateParser::loadFromFile] filename is "<<at.filename);

      const rapidjson::Value& traj = jdoc["end_effector_trajectory"];     
      for (rapidjson::SizeType t = 0; t < traj.Size(); ++t)
      {
        Trajectory ee_trajectory;
        ee_trajectory.name = traj[t]["name"].GetString();
        ROS_DEBUG_STREAM("[AffordanceTemplateParser::loadFromFile] found trajectory with name: "<<ee_trajectory.name);
        
        const rapidjson::Value& ee_group = traj[t]["end_effector_group"];
        for (rapidjson::SizeType g = 0; g < ee_group.Size(); ++g)
        {

          EndEffectorWaypointList ee;
          ee.id = ee_group[g]["id"].GetInt();

          ROS_DEBUG_STREAM("[AffordanceTemplateParser::loadFromFile]    found EE group for ID: "<<ee.id);

          const rapidjson::Value& waypoints = ee_group[g]["end_effector_waypoint"];
          for (rapidjson::SizeType w = 0; w < waypoints.Size(); ++w)
          {
              // find the origin
              Origin org;
              if(!waypoints[w].HasMember("origin")) {
                ROS_ERROR_STREAM("[AffordanceTemplateParser::loadFromFile]    cant define waypoint " <<w+1<< " without origin");
                std::fclose(f_pnt);
                return false;
              }

              const rapidjson::Value& pos = waypoints[w]["origin"]["xyz"];
              org.position[0] = pos[0].GetDouble();
              org.position[1] = pos[1].GetDouble();
              org.position[2] = pos[2].GetDouble();
              const rapidjson::Value& euler = waypoints[w]["origin"]["rpy"];
              org.orientation[0] = euler[0].GetDouble();
              org.orientation[1] = euler[1].GetDouble();
              org.orientation[2] = euler[2].GetDouble();

              // create the controls
              // ** note: it's a little cryptic but basically some of the jsons have "1" for true under controls
              // **       so this checks to see if it's bool or int, then converts int if not bool
              Control ctrl;
              if(waypoints[w].HasMember("controls")) {
                const rapidjson::Value& trans = waypoints[w]["controls"]["xyz"];
                ctrl.translation[0] = trans[0].IsBool() ? trans[0].GetBool() : (trans[0].GetInt() == 1 ? true : false);
                ctrl.translation[1] = trans[1].IsBool() ? trans[1].GetBool() : (trans[1].GetInt() == 1 ? true : false);
                ctrl.translation[2] = trans[2].IsBool() ? trans[2].GetBool() : (trans[2].GetInt() == 1 ? true : false);
                const rapidjson::Value& rot = waypoints[w]["controls"]["rpy"];
                ctrl.rotation[0] = rot[0].IsBool() ? rot[0].GetBool() : (rot[0].GetInt() == 1 ? true : false);
                ctrl.rotation[1] = rot[1].IsBool() ? rot[1].GetBool() : (rot[1].GetInt() == 1 ? true : false);
                ctrl.rotation[2] = rot[2].IsBool() ? rot[2].GetBool() : (rot[2].GetInt() == 1 ? true : false);
                ctrl.scale = waypoints[w]["controls"]["scale"].GetDouble();
              } else {
                ctrl.translation[0] = ctrl.translation[1] = ctrl.translation[2] = true;
                ctrl.rotation[0] = ctrl.rotation[1] = ctrl.rotation[2] = true;
                ctrl.scale = 0.25;
              }

              // get planner type
              std::string planner_type = "CARTESIAN";
              if(waypoints[w].HasMember("planner_type")) {
                ROS_DEBUG_STREAM("  has planner_type");
                planner_type = waypoints[w]["planner_type"].GetString(); 
              } 

              // get conditioning metric
              std::string conditioning_metric = "MIN_DISTANCE";
              if(waypoints[w].HasMember("conditioning_metric")) {
                ROS_DEBUG_STREAM("  has conditioning_metric");
                conditioning_metric = waypoints[w]["conditioning_metric"].GetString(); 
              } 
              
              // get tolerance bounds
              ToleranceBounds bounds;
              if(waypoints[w].HasMember("tolerances")) {
                ROS_DEBUG_STREAM("  has tolerances");
                const rapidjson::Value& xpos = waypoints[w]["tolerances"]["x"];
                bounds.position[0][0] = xpos[0].GetDouble();
                bounds.position[0][1] = xpos[1].GetDouble();
                const rapidjson::Value& ypos = waypoints[w]["tolerances"]["y"];
                bounds.position[1][0] = ypos[0].GetDouble();
                bounds.position[1][1] = ypos[1].GetDouble();
                const rapidjson::Value& zpos = waypoints[w]["tolerances"]["z"];
                bounds.position[2][0] = zpos[0].GetDouble();
                bounds.position[2][1] = zpos[1].GetDouble();
                const rapidjson::Value& Xrot = waypoints[w]["tolerances"]["R"];
                bounds.orientation[0][0] = Xrot[0].GetDouble();
                bounds.orientation[0][1] = Xrot[1].GetDouble();
                const rapidjson::Value& Yrot = waypoints[w]["tolerances"]["P"];
                bounds.orientation[1][0] = Yrot[0].GetDouble();
                bounds.orientation[1][1] = Yrot[1].GetDouble();
                const rapidjson::Value& Zrot = waypoints[w]["tolerances"]["Y"];
                bounds.orientation[2][0] = Zrot[0].GetDouble();
                bounds.orientation[2][1] = Zrot[1].GetDouble();
              } else {
                for(int i=0; i<3; i++) {
                  bounds.position[i][0] = 0.01;
                  bounds.position[i][1] = -0.01;
                  bounds.orientation[i][0] = 0.0349;
                  bounds.orientation[i][1] = -0.0349;
                }
              }
              
              // get task compatibility directions
              TaskCompatibility tc;
              if(waypoints[w].HasMember("task_compatibility")) {
                ROS_DEBUG_STREAM("  has task_compatibility");
                const rapidjson::Value& trans = waypoints[w]["task_compatibility"]["xyz"];
                tc.position[0] = (trans[0].GetInt() == 0) ? 0 : (trans[0].GetInt() < 0 ? -1 : 1);
                tc.position[1] = (trans[1].GetInt() == 0) ? 0 : (trans[1].GetInt() < 0 ? -1 : 1);
                tc.position[2] = (trans[2].GetInt() == 0) ? 0 : (trans[2].GetInt() < 0 ? -1 : 1);
                const rapidjson::Value& rot = waypoints[w]["task_compatibility"]["rpy"];
                tc.orientation[0] = (rot[0].GetInt() == 0) ? 0 : (rot[0].GetInt() < 0 ? -1 : 1);
                tc.orientation[1] = (rot[1].GetInt() == 0) ? 0 : (rot[1].GetInt() < 0 ? -1 : 1);
                tc.orientation[2] = (rot[2].GetInt() == 0) ? 0 : (rot[2].GetInt() < 0 ? -1 : 1);
              } else {
                tc.position[0] = 1; 
                tc.position[1] = 1; 
                tc.position[2] = 1; 
                tc.orientation[0] = 1; 
                tc.orientation[1] = 1; 
                tc.orientation[2] = 1; 
              }

              // get tool offset
              Origin tool_offset;
              if(waypoints[w].HasMember("tool_offset")) {
                ROS_DEBUG_STREAM("  has tool_offset");
                const rapidjson::Value& pos = waypoints[w]["tool_offset"]["xyz"];
                tool_offset.position[0] = pos[0].GetDouble();
                tool_offset.position[1] = pos[1].GetDouble();
                tool_offset.position[2] = pos[2].GetDouble();
                const rapidjson::Value& euler = waypoints[w]["tool_offset"]["rpy"];
                tool_offset.orientation[0] = euler[0].GetDouble();
                tool_offset.orientation[1] = euler[1].GetDouble();
                tool_offset.orientation[2] = euler[2].GetDouble();
              } else {
                tool_offset.position[0] = tool_offset.position[1] = tool_offset.position[2] = 0.0;
                tool_offset.orientation[0] = tool_offset.orientation[1] = tool_offset.orientation[2] = 0.0;
              }

               // fill out the waypoint
              EndEffectorWaypoint wp;
              wp.ee_pose = waypoints[w]["ee_pose"].GetInt();
              wp.display_object = waypoints[w]["display_object"].GetString();
              wp.origin = org;
              wp.controls = ctrl;
              wp.bounds = bounds;
              wp.tool_offset = tool_offset;
              wp.task_compatibility = tc;
              wp.conditioning_metric = conditioning_metric;
              wp.planner_type = planner_type;

              ROS_DEBUG_STREAM("[AffordanceTemplateParser::loadFromFile]     waypoint "<<w+1<<" has ee_pose: "<<wp.ee_pose<<" and display_object: "<<wp.display_object);
              
              ee.waypoints.push_back(wp);
              
              ROS_DEBUG("[AffordanceTemplateParser::loadFromFile] \tat origin XYZ: %g %g %g and RPY: %g %g %g", org.position[0], org.position[1], org.position[2], org.orientation[0], org.orientation[1], org.orientation[2]);
              ROS_DEBUG("[AffordanceTemplateParser::loadFromFile] \tcontrol for axes set to: XYZ: %s %s %s", toBoolString(ctrl.translation[0]).c_str(), toBoolString(ctrl.translation[1]).c_str(), toBoolString(ctrl.translation[2]).c_str());
              ROS_DEBUG("[AffordanceTemplateParser::loadFromFile] \tcontrol for axes set to: RPY: %s %s %s", toBoolString(ctrl.rotation[0]).c_str(), toBoolString(ctrl.rotation[1]).c_str(), toBoolString(ctrl.rotation[2]).c_str());
              ROS_DEBUG("[AffordanceTemplateParser::loadFromFile] \twaypoint is scaled to %g", ctrl.scale);
          }
          ee_trajectory.ee_waypoint_list.push_back(ee);
        }
        at.ee_trajectories.push_back(ee_trajectory);
      }

      const rapidjson::Value& objects = jdoc["display_objects"];
      for (rapidjson::SizeType i = 0; i < objects.Size(); ++i)
      {
          // find the origin
          Origin orig;
          const rapidjson::Value& pos = objects[i]["origin"]["xyz"];
          orig.position[0] = pos[0].GetDouble();
          orig.position[1] = pos[1].GetDouble();
          orig.position[2] = pos[2].GetDouble();
          const rapidjson::Value& euler = objects[i]["origin"]["rpy"];
          orig.orientation[0] = euler[0].GetDouble();
          orig.orientation[1] = euler[1].GetDouble();
          orig.orientation[2] = euler[2].GetDouble();

          // create the controls
          // ** note: it's a little cryptic but basically some of the jsons have "1" for true under controls
          // **       so this checks to see if it's bool or int, then converts int if not bool
          Control ctrl;
          const rapidjson::Value& trans = objects[i]["controls"]["xyz"];
          ctrl.translation[0] = trans[0].IsBool() ? trans[0].GetBool() : (trans[0].GetInt() == 1 ? true : false);
          ctrl.translation[1] = trans[1].IsBool() ? trans[1].GetBool() : (trans[1].GetInt() == 1 ? true : false);
          ctrl.translation[2] = trans[2].IsBool() ? trans[2].GetBool() : (trans[2].GetInt() == 1 ? true : false);
          const rapidjson::Value& rot = objects[i]["controls"]["rpy"];
          ctrl.rotation[0] = rot[0].IsBool() ? rot[0].GetBool() : (rot[0].GetInt() == 1 ? true : false);
          ctrl.rotation[1] = rot[1].IsBool() ? rot[1].GetBool() : (rot[1].GetInt() == 1 ? true : false);
          ctrl.rotation[2] = rot[2].IsBool() ? rot[2].GetBool() : (rot[2].GetInt() == 1 ? true : false);
          ctrl.scale = objects[i]["controls"]["scale"].GetDouble();

          Shape shp;
          shp.type = objects[i]["shape"]["type"].GetString();
          std::string shape_str = "shape of type: ";
          if (shp.type == "mesh")
          {
              shp.mesh = objects[i]["shape"]["data"].GetString();
              shape_str += "mesh using mesh file: " + shp.mesh;
          }
          else // some other shape which will have material
          {
              shp.color = objects[i]["shape"]["material"]["color"].GetString();
              const rapidjson::Value& color = objects[i]["shape"]["material"]["rgba"];
              shp.rgba[0] = color[0].GetDouble();
              shp.rgba[1] = color[1].GetDouble();
              shp.rgba[2] = color[2].GetDouble();
              shp.rgba[3] = color[3].GetDouble();

              std::ostringstream rgba;
              rgba << shp.rgba[0] << " " << shp.rgba[1] << " " << shp.rgba[2] << " " << shp.rgba[3];
              shape_str += shp.type + " colored: " + shp.color + " using RGBA: " + rgba.str();
          }
          const rapidjson::Value& sz = objects[i]["shape"]["size"];
          shp.size[0] = sz[0].GetDouble();
          shp.size[1] = sz[1].GetDouble();
          shp.size[2] = sz[2].GetDouble();

          DisplayObject display_object;
          display_object.name = objects[i]["name"].GetString();

          if(objects[i].HasMember("parent"))
            display_object.parent = objects[i]["parent"].GetString();
          else
            display_object.parent = "";
          display_object.origin = orig;
          display_object.shape = shp;
          display_object.controls = ctrl;

          ROS_DEBUG_STREAM("[AffordanceTemplateParser::loadFromFile] display object "<<i+1<<" has name: "<<display_object.name);
          if(display_object.parent != "")
            ROS_DEBUG("[AffordanceTemplateParser::loadFromFile] \tparent object: %s", display_object.parent.c_str());
          ROS_DEBUG("[AffordanceTemplateParser::loadFromFile] \tat origin XYZ: %g %g %g and RPY: %g %g %g", orig.position[0], orig.position[1], orig.position[2], orig.orientation[0], orig.orientation[1], orig.orientation[2]);
          ROS_DEBUG("[AffordanceTemplateParser::loadFromFile] \tcontrol for axes set to: XYZ: %s %s %s", toBoolString(ctrl.translation[0]).c_str(), toBoolString(ctrl.translation[1]).c_str(), toBoolString(ctrl.translation[2]).c_str());
          ROS_DEBUG("[AffordanceTemplateParser::loadFromFile] \tcontrol for axes set to: RPY: %s %s %s", toBoolString(ctrl.rotation[0]).c_str(), toBoolString(ctrl.rotation[1]).c_str(), toBoolString(ctrl.rotation[2]).c_str());
          ROS_DEBUG("[AffordanceTemplateParser::loadFromFile] \t%s is scaled to %g", display_object.name.c_str(), ctrl.scale);
          ROS_DEBUG("[AffordanceTemplateParser::loadFromFile] \t%s", shape_str.c_str());

          at.display_objects.push_back(display_object);
      }
      // std::cout<<std::endl;
  }
  std::fclose(f_pnt);

  return true;
}

bool AffordanceTemplateParser::saveToFile(const std::string& filepath, const AffordanceTemplateStructure& at)
{
  ROS_DEBUG("[AffordanceTemplateParser::saveToFile] opening file for JSON saving: %s", filepath.c_str());

  // create writer doc
  rapidjson::StringBuffer json;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(json);
  writer.StartObject();
  writer.SetIndent(' ', 2);

  // extract the template type out of the name
  std::vector<std::string> names;
  boost::split(names, at.name, boost::is_any_of(":"));
  assert(names.size());

  writer.Key("name");  writer.String(names.front().c_str());
  writer.Key("image"); writer.String(at.image.c_str());

  writer.Key("display_objects"); 
  writer.StartArray();
  for ( auto d : at.display_objects)
  {
    writer.StartObject();

    // NAME, PARENT if available
    names.clear();
    boost::split(names, d.name, boost::is_any_of(":"));
    assert(names.size());
    writer.Key("name");  writer.String(names.front().c_str());
    if (!d.parent.empty()) 
    {
      writer.Key("parent");
      writer.String(d.parent.c_str());
    }

    // SHAPE
    writer.Key("shape");
    writer.StartObject();
    writer.Key("type"); writer.String(d.shape.type.c_str());
    if (d.shape.type == "mesh" || d.shape.type == "box")
    {
      if (d.shape.type == "mesh")
      {
        writer.Key("data"); writer.String(d.shape.mesh.c_str());
      }
      else
      {
        writer.Key("material");
        writer.StartObject();
        writer.Key("color"); writer.String(d.shape.color.c_str());
        writer.Key("rgba");
        writer.StartArray();
        writer.Double(d.shape.rgba[0]);
        writer.Double(d.shape.rgba[1]);
        writer.Double(d.shape.rgba[2]);
        writer.Double(d.shape.rgba[3]);
        writer.EndArray();
        writer.EndObject();
      }
      writer.Key("size");
      writer.StartArray();
      writer.Double(d.shape.size[0]);
      writer.Double(d.shape.size[1]);
      writer.Double(d.shape.size[2]);
      writer.EndArray();
    }
    else ROS_FATAL("[AffordanceTemplateParser::saveToFile] shape %s has not be formatted to write to file!! contact developer.", d.shape.type.c_str());
    writer.EndObject();

    // ORIGIN 
    writer.Key("origin");
    writer.StartObject();
    writer.Key("xyz");
    writer.StartArray();
    writer.Double(d.origin.position[0]);
    writer.Double(d.origin.position[1]);
    writer.Double(d.origin.position[2]);
    writer.EndArray();
    writer.Key("rpy");
    writer.StartArray();
    writer.Double(d.origin.orientation[0]);
    writer.Double(d.origin.orientation[1]);
    writer.Double(d.origin.orientation[2]);
    writer.EndArray();
    writer.EndObject();

    // CONTROLS
    writer.Key("controls");
    writer.StartObject();
    writer.Key("xyz");
    writer.StartArray();
    writer.Bool(d.controls.translation[0]);
    writer.Bool(d.controls.translation[1]);
    writer.Bool(d.controls.translation[2]);
    writer.EndArray();
    writer.Key("rpy");
    writer.StartArray();
    writer.Bool(d.controls.rotation[0]);
    writer.Bool(d.controls.rotation[1]);
    writer.Bool(d.controls.rotation[2]);
    writer.EndArray();
    writer.Key("scale"); writer.Double(d.controls.scale);
    writer.EndObject();

    writer.EndObject(); // end of each display object
  }
  writer.EndArray();

  writer.Key("end_effector_trajectory"); 
  writer.StartArray();
  for ( auto ee : at.ee_trajectories)
  {
    writer.StartObject();
    
    writer.Key("name"); writer.String(ee.name.c_str());

    writer.Key("end_effector_group");
    writer.StartArray();
    for ( auto grp : ee.ee_waypoint_list)
    {
      writer.StartObject(); 

      writer.Key("id"); writer.Int(grp.id);

      writer.Key("end_effector_waypoint");
      writer.StartArray();
      for ( auto wp : grp.waypoints)
      {
        writer.StartObject();

        writer.Key("ee_pose"); writer.Int(wp.ee_pose);
        writer.Key("display_object"); writer.String(wp.display_object.c_str());

        // ORIGIN 
        writer.Key("origin");
        writer.StartObject();
        writer.Key("xyz");
        writer.StartArray();
        writer.Double(wp.origin.position[0]);
        writer.Double(wp.origin.position[1]);
        writer.Double(wp.origin.position[2]);
        writer.EndArray();
        writer.Key("rpy");
        writer.StartArray();
        writer.Double(wp.origin.orientation[0]);
        writer.Double(wp.origin.orientation[1]);
        writer.Double(wp.origin.orientation[2]);
        writer.EndArray();
        writer.EndObject();

        // CONTROLS
        writer.Key("controls");
        writer.StartObject();
        writer.Key("xyz");
        writer.StartArray();
        writer.Bool(wp.controls.translation[0]);
        writer.Bool(wp.controls.translation[1]);
        writer.Bool(wp.controls.translation[2]);
        writer.EndArray();
        writer.Key("rpy");
        writer.StartArray();
        writer.Bool(wp.controls.rotation[0]);
        writer.Bool(wp.controls.rotation[1]);
        writer.Bool(wp.controls.rotation[2]);
        writer.EndArray();
        writer.Key("scale"); writer.Double(wp.controls.scale);
        writer.EndObject();

        // TOOL OFFSET
        writer.Key("tool_offset");
        writer.StartObject();
        writer.Key("xyz");
        writer.StartArray();
        writer.Double(wp.tool_offset.position[0]);
        writer.Double(wp.tool_offset.position[1]);
        writer.Double(wp.tool_offset.position[2]);
        writer.EndArray();
        writer.Key("rpy");
        writer.StartArray();
        writer.Double(wp.tool_offset.orientation[0]);
        writer.Double(wp.tool_offset.orientation[1]);
        writer.Double(wp.tool_offset.orientation[2]);
        writer.EndArray();
        writer.EndObject();

        // TOOL OFFSET
        writer.Key("tolerances");
        writer.StartObject();
        writer.Key("x");
        writer.StartArray();
        writer.Double(wp.bounds.position[0][0]);
        writer.Double(wp.bounds.position[0][1]);
        writer.EndArray();
        writer.Key("y");
        writer.StartArray();
        writer.Double(wp.bounds.position[1][0]);
        writer.Double(wp.bounds.position[1][1]);
        writer.EndArray();
        writer.Key("z");
        writer.StartArray();
        writer.Double(wp.bounds.position[2][0]);
        writer.Double(wp.bounds.position[2][1]);
        writer.EndArray();
        writer.Key("R");
        writer.StartArray();
        writer.Double(wp.bounds.orientation[0][0]);
        writer.Double(wp.bounds.orientation[0][1]);
        writer.EndArray();
        writer.Key("P");
        writer.StartArray();
        writer.Double(wp.bounds.orientation[1][0]);
        writer.Double(wp.bounds.orientation[1][1]);
        writer.EndArray();
        writer.Key("Y");
        writer.StartArray();
        writer.Double(wp.bounds.orientation[2][0]);
        writer.Double(wp.bounds.orientation[2][1]);
        writer.EndArray();
        writer.EndObject();

        // other stuff
        writer.Key("conditioning_metric"); writer.String(wp.conditioning_metric.c_str());
        writer.Key("planner_type"); writer.String(wp.planner_type.c_str());

        writer.EndObject(); // each waypoint struct
      }
      writer.EndArray(); // waypoints vector

      writer.EndObject(); // each ee group struct
    }
    writer.EndArray(); // ee groups vector

    writer.EndObject(); // end of each ee trajectory struct
  }
  writer.EndArray();

  writer.EndObject(); // end of json object
  
  std::ofstream file(filepath.c_str(), std::ios::trunc);
  file << json.GetString() << std::endl;
  file.close();


  ROS_DEBUG("[AffordanceTemplateParser::saveToFile] successfully wrote template %s to file", at.name.c_str());

  return true;
}