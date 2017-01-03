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

#include <affordance_template_server/server.h>

using namespace affordance_template_server;

AffordanceTemplateServer::AffordanceTemplateServer(const ros::NodeHandle &nh, const std::string &_robot_yaml="") :
    nh_(nh),
    robot_yaml_(_robot_yaml)
{
    
    pkg_name_ = "affordance_template_library";

    if (robot_yaml_.empty())
        ROS_WARN("[AffordanceTemplateServer] no robot yaml provided - BE SURE TO LOAD ROBOT FROM SERVICE!!");
    else
        if (!loadRobot())
            ROS_ERROR("[AffordanceTemplateServer] couldn't parse robot .yamls!!");

    im_server_.reset( new interactive_markers::InteractiveMarkerServer(std::string("affordance_template_interactive_marker_server"), "", false));

    if (!loadTemplates())
        ROS_ERROR("[AffordanceTemplateServer] couldn't parse robot JSONs!!");

    ROS_INFO("[AffordanceTemplateServer] server configured. spinning...");
}

/**
 * @brief parse robot JSONs
 * @details finds any and all JSON files in the AT Library 'templates' directory 
 *  and parses into the map at_structure_map_ 
 * 
 * @return false if can't find or load the directory, true otherwise
 */
bool AffordanceTemplateServer::loadTemplates()
{
    ROS_INFO("[AffordanceTemplateServer::loadTemplates] searching for JSON templates in package: %s", pkg_name_.c_str());

    affordance_template_object::AffordanceTemplateParser atp;

    std::string root = getPackagePath(pkg_name_);
    if (root.empty())
        return false;
    
    // clear what we have, start over
    at_structure_map_.clear();

    root += "/templates";
    if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root))
    {
        ROS_WARN("[AffordanceTemplateServer::loadTemplates] cannot find templates in path %s!!", root.c_str());
        return false;
    }

    // use boost to get all of the files with .yaml extension
    std::map<std::string, std::string> template_paths;
    boost::filesystem::recursive_directory_iterator dir_it(root);
    boost::filesystem::recursive_directory_iterator end_it;
    while (dir_it != end_it)
    {
        if (dir_it->path().extension() == ".json")
        {
            std::string file = dir_it->path().string();
            char *cstr = new char[file.length() + 1];
            strcpy(cstr, file.c_str());
            char* str = std::strtok(cstr, "/");

            while(std::string(str).find(".json") == std::string::npos && ros::ok())
                str = std::strtok(NULL, "/");

            std::string name(str);
            ROS_INFO("[AffordanceTemplateServer::loadTemplates] found template name: %s", name.c_str());
            template_paths[name] = dir_it->path().string();
            
            delete [] cstr;
        }
        ++dir_it;
    }

    // make robot instances with the .yamls we just found
    for (auto& t : template_paths)
    {
        affordance_template_object::AffordanceTemplateStructure at;
        bool loaded = atp.loadFromFile(t.second, at);
        at_structure_map_[at.name] = at;
    }

    return true;
}

bool AffordanceTemplateServer::loadRobot()
{
    ROS_INFO("[AffordanceTemplateServer::loadRobot] loading robot yaml %s from path: %s", robot_yaml_.c_str(), pkg_name_.c_str());

    robot_name_ = "";

    std::string root = getPackagePath(pkg_name_);
    if (root.empty())
        return false;

    root += "/robots/";

    if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root))
    {
        ROS_WARN("[AffordanceTemplateServer::loadRobot] cannot find robots in path: %s!!", root.c_str());
        return false;
    }

    // // use boost to get all of the files with .yaml extension
    // // std::vector<std::string> robot_paths_vec;
    // // boost::filesystem::recursive_directory_iterator dir_it(root);
    // // boost::filesystem::recursive_directory_iterator end_it;
    // while (dir_it != end_it)
    // {
    //     if (boost::filesystem::is_regular_file(*dir_it) && dir_it->path().extension() == ".yaml")
    //     {
    //         ROS_INFO("[AffordanceTemplateServer::loadRobot] found robot yaml at path: %s", dir_it->path().string().c_str());
    //         robot_paths_vec.push_back(dir_it->path().string());
    //     }
    //     ++dir_it;
    // }

    // make robot instances with the .yamls we just found
    robot_interface_.reset(new affordance_template_markers::RobotInterface(nh_));
    if ( !robot_interface_->load(root+robot_yaml_))
    {
        ROS_WARN("[AffordanceTemplateServer::loadRobot] robot yaml %s NOT loaded, ignoring.", robot_yaml_.c_str());
        return false;
    }

    if ( !robot_interface_->configure())
    {
        ROS_WARN("[AffordanceTemplateServer::loadRobot] robot yaml %s NOT configured, ignoring.", robot_yaml_.c_str());
        return false;
    }        

    robot_config_ = robot_interface_->getRobotConfig();
    robot_name_ = robot_config_.name;
    if (getPackagePath(robot_config_.config_package).empty())
        ROS_WARN("[AffordanceTemplateServer::loadRobot] config package %s NOT found, ignoring.", robot_config_.config_package.c_str());
    else
        ROS_INFO("[AffordanceTemplateServer::loadRobot] config package %s found", robot_config_.config_package.c_str());

    return true;
}

std::string AffordanceTemplateServer::getPackagePath(const std::string &pkg_name)
{
    ROS_INFO("[AffordanceTemplateServer::getPackagePath] finding path for package %s.......", pkg_name.c_str());
    std::string path = ros::package::getPath(pkg_name); // will be empty if not found
    if (path.empty())
        ROS_WARN("[AffordanceTemplateServer::getPackagePath] couldn't find path to package %s!!", pkg_name.c_str());
    else
        ROS_INFO("[AffordanceTemplateServer::getPackagePath] found path: %s", path.c_str());
    return path;
}

int AffordanceTemplateServer::getNextID(const std::string &type)
{
    //boost::mutex::scoped_lock l(mutex_);

    std::vector<int> ids;
    for (auto at : at_map_)
    {
        if (at.second->getType() == type)
            ids.push_back(at.second->getID());
    }

    int next = 0;
    while(1)
    {
        if (std::find(ids.begin(), ids.end(), next) == ids.end())
            return next;
        else 
            ++next;
    }

    return next;
}

//################
// public methods
//################

std::vector<affordance_template_msgs::AffordanceTemplateConfig> AffordanceTemplateServer::getAvailableTemplates(const std::string &name)
{
    std::vector<affordance_template_msgs::AffordanceTemplateConfig> templates;
    if (!name.empty())
    {
        affordance_template_msgs::AffordanceTemplateConfig atc;
        atc.filename = at_structure_map_[name].filename;
        atc.type = at_structure_map_[name].name;
        atc.image_path = at_structure_map_[name].image;
        for (auto ee : at_structure_map_[name].ee_trajectories)
        {
            affordance_template_msgs::WaypointTrajectory wp;
            wp.name = ee.name;
            for (int w = 0; w < ee.ee_waypoint_list.size(); ++w)
            {
                affordance_template_msgs::WaypointInfo wi;
                wi.id = ee.ee_waypoint_list[w].id;
                wi.num_waypoints = ee.ee_waypoint_list[w].waypoints.size();
                wp.waypoint_info.push_back(wi);
            }
            atc.trajectory_info.push_back(wp);
        }
        for (auto d : at_structure_map_[name].display_objects)
            atc.display_objects.push_back(d.name);        
        templates.push_back(atc);
    }
    else
    {   
        for (auto t : at_structure_map_)
        {
            affordance_template_msgs::AffordanceTemplateConfig atc;
            atc.filename = t.second.filename;
            atc.type = t.second.name;
            atc.image_path = t.second.image;
            for (auto ee : t.second.ee_trajectories)
            {
                affordance_template_msgs::WaypointTrajectory wp;
                wp.name = ee.name;
                for (int w = 0; w < ee.ee_waypoint_list.size(); ++w)
                {
                    affordance_template_msgs::WaypointInfo wi;
                    wi.id = ee.ee_waypoint_list[w].id;
                    wi.num_waypoints = ee.ee_waypoint_list[w].waypoints.size();
                    wp.waypoint_info.push_back(wi);
                }
                atc.trajectory_info.push_back(wp);
            }
            for (auto d : t.second.display_objects)
                atc.display_objects.push_back(d.name);
            templates.push_back(atc);
        }
    }
    return templates;
}

std::vector<std::string> AffordanceTemplateServer::getRunningTemplates(const std::string &name)
{
    std::vector<std::string> templates;

    for (auto a : at_map_)
       templates.push_back(a.first);

    return templates;
}

bool AffordanceTemplateServer::loadRobot(const std::string &name="")
{
    if (!name.empty())
        return false;

    robot_interface_->tearDown();
    return robot_interface_->load(name);
} 

bool AffordanceTemplateServer::loadRobot(const affordance_template_msgs::RobotConfig &msg)
{
    std::string name = msg.name;
    robot_interface_->tearDown();
    return robot_interface_->load(msg);
}

bool AffordanceTemplateServer::addTemplate(const std::string& type, uint8_t& id)
{
    geometry_msgs::PoseStamped pose;
    return addTemplate(type, id, pose);
}

bool AffordanceTemplateServer::addTemplate(const std::string &type, uint8_t& id, geometry_msgs::PoseStamped &pose)
{
    if (type.empty())
      return false;

    id = getNextID(type);
    std::string key = type + ":" + std::to_string(id);
    ROS_INFO("[AffordanceTemplateServer::addTemplate] creating new affordance template with ID: %d and key: %s", id, key.c_str());

    at_map_[key] = boost::shared_ptr<affordance_template::AffordanceTemplate>(new affordance_template::AffordanceTemplate(nh_, im_server_, robot_interface_, robot_name_, type, id));
    
    affordance_template_object::AffordanceTemplateStructure structure;
    geometry_msgs::Pose p;
    return at_map_[key]->loadFromFile( at_structure_map_[type].filename, p, structure);
}

bool AffordanceTemplateServer::removeTemplate(const std::string &type, const uint8_t id)
{
    boost::mutex::scoped_lock l(mutex_);

    std::string key = type + ":" + std::to_string(id);
    if (at_map_.find(key) == at_map_.end())
      return false;
    at_map_[key]->stop();
    at_map_.erase(key);

    for (auto at : at_map_)
      if (!at.second->buildTemplate())
        ROS_ERROR_STREAM("[AffordanceTemplateServer::removeTemplate] couldn't redraw "<< at.first.c_str() <<" template");

    return true;
}

bool AffordanceTemplateServer::updateTemplate(const std::string& type, const uint8_t id, const geometry_msgs::PoseStamped& pose)
{
    std::string key = type + ":" + std::to_string(id);
    if (at_map_.find(key) == at_map_.end())
        return false;

    ROS_INFO("[AffordanceTemplateServer::updateTemplate] updating %s to X: %g Y: %g Z: %g, x: %g y: %g z: %g w: %g", key.c_str(), pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    return at_map_[key]->setTemplatePose(pose); 
}

bool AffordanceTemplateServer::getTemplateInstance(const std::string &type, const uint8_t id, boost::shared_ptr<affordance_template::AffordanceTemplate> &ati)
{
    boost::mutex::scoped_lock l(mutex_);

    std::string key = type + ":" + std::to_string(id);
    if (at_map_.find(key) == at_map_.end())
        return false;
    ati = at_map_[key];
    return true;
}

bool AffordanceTemplateServer::getTemplateInstance(const std::string &key, boost::shared_ptr<affordance_template::AffordanceTemplate> &ati)
{
    if (at_map_.find(key) == at_map_.end())
        return false;
    ati = at_map_[key];
    return true;
}