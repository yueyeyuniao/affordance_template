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

#include <rviz_affordance_template_panel/rviz_affordance_template_panel.h>

// TODO: don't like these
#define PIXMAP_SIZE 100
#define XOFFSET 20
#define YOFFSET 20

#define CLASS_INDEX 0
#define TRAJECTORY_DATA 1
#define IMAGE 2
#define FILENAME 3
#define DISPLAY_OBJECTS 4

#define OBJECT_INDEX 0
#define PACKAGE 1
#define LAUNCH_FILE 2

using namespace rviz_affordance_template_panel;
using namespace std;

AffordanceTemplateRVizClient::AffordanceTemplateRVizClient(ros::NodeHandle &nh, Ui::RVizAffordanceTemplatePanel* ui, QGraphicsScene *at_scene) :
  nh_(nh),
  ui_(ui),
  affordanceTemplateGraphicsScene_(at_scene),
  descriptionRobot_(""),
  running_(false),
  controls_(new Controls()),
  waypointDisplay_(new WaypointDisplay()),
  server_status_(-1),
  waypoint_display_configured_(false)
{

  // setup service clients
  add_template_client_ = nh_.serviceClient<affordance_template_msgs::AddAffordanceTemplate>("/affordance_template_server/add_template");
  add_trajectory_client_ = nh_.serviceClient<affordance_template_msgs::AddAffordanceTemplateTrajectory>("/affordance_template_server/add_trajectory");
  plan_command_client_ = nh_.serviceClient<affordance_template_msgs::AffordanceTemplatePlanCommand>("/affordance_template_server/plan_command");
  execute_command_client_ = nh_.serviceClient<affordance_template_msgs::AffordanceTemplateExecuteCommand>("/affordance_template_server/execute_command");
  delete_template_client_ = nh_.serviceClient<affordance_template_msgs::DeleteAffordanceTemplate>("/affordance_template_server/delete_template");
  get_robots_client_ = nh_.serviceClient<affordance_template_msgs::GetRobotConfigInfo>("/affordance_template_server/get_robots");
  get_running_client_ = nh_.serviceClient<affordance_template_msgs::GetRunningAffordanceTemplates>("/affordance_template_server/get_running");
  get_templates_client_ = nh_.serviceClient<affordance_template_msgs::GetAffordanceTemplateConfigInfo>("/affordance_template_server/get_templates");
  get_template_status_client_ = nh_.serviceClient<affordance_template_msgs::GetAffordanceTemplateStatus>("/affordance_template_server/get_template_status");
  load_robot_client_ = nh_.serviceClient<affordance_template_msgs::LoadRobotConfig>("/affordance_template_server/load_robot");
  save_template_client_ = nh_.serviceClient<affordance_template_msgs::SaveAffordanceTemplate>("/affordance_template_server/save_template");
  scale_object_client_ = nh_.serviceClient<affordance_template_msgs::ScaleDisplayObject>("/affordance_template_server/scale_object");
  set_template_trajectory_client_ = nh_.serviceClient<affordance_template_msgs::SetAffordanceTemplateTrajectory>("/affordance_template_server/set_template_trajectory");
  set_waypoint_view_client_  = nh_.serviceClient<affordance_template_msgs::SetWaypointViewModes>("/affordance_template_server/set_waypoint_view");

  // setup publishers
  scale_object_streamer_ = nh_.advertise<affordance_template_msgs::ScaleDisplayObjectInfo>("/affordance_template_server/scale_object_streamer", 10);

  // set up controls helper class
  controls_->setUI(ui_);
  controls_->setServices(plan_command_client_, execute_command_client_);

  // set up waypoint display helper class
  waypointDisplay_->setUI(ui_);
  waypointDisplay_->setService(set_waypoint_view_client_);
  
  selected_template = make_pair("",-1);

  label_palette_ = new QPalette();// ui_->server_status_label->palette();

  // start up server monitor thread
  server_monitor_ = new AffordanceTemplateServerStatusMonitor(nh_, std::string("/affordance_template_server/get_status"), 1);

  busy_flag_ = false;
}

AffordanceTemplateRVizClient::~AffordanceTemplateRVizClient()
{
  ros::Duration(1).sleep();
  stop();
  
  // clean up pointers
  for (auto &ats : template_status_info) 
    delete ats.second;
  delete server_monitor_;
  delete label_palette_;
  delete thread_;
  delete affordanceTemplateGraphicsScene_;
}

void AffordanceTemplateRVizClient::init() {  

  ROS_WARN("AffordanceTemplateRVizClient::init()");
  getAvailableInfo();
  tryToLoadRobotFromYAML();
  getRunningItems();
}

void AffordanceTemplateRVizClient::start() {

  ROS_INFO("AffordanceTemplateRVizClient::start()");
  init();
  thread_ = new boost::thread(boost::bind(&AffordanceTemplateRVizClient::run_function, this));
  server_monitor_->start();

  if (server_monitor_->isReady()) {
      getAvailableInfo();
      getRunningItems();
  }
}

void AffordanceTemplateRVizClient::stop() {
  ROS_INFO("AffordanceTemplateRVizClient::stop()");
  running_ = false;
  thread_->join();
  server_monitor_->stop();
}

void AffordanceTemplateRVizClient::run_function() 
{
  running_ = true;
  while(running_) {
    updateServerStatus();
    if(server_monitor_->isReady()) 
        controlStatusUpdate();
    ros::Duration(0.5).sleep();
  }
}  

void AffordanceTemplateRVizClient::updateServerStatus() {
  int old_status = server_status_;
  if(server_monitor_->isAvailable()) {
    
    if(server_monitor_->isReady()) {
      setLabelText(Qt::green, std::string("READY"));
      server_status_ = 1;
    } else {
      setLabelText(Qt::blue, std::string("NOT READY"));
      server_status_ = 0;
    }   
  } else {
    setLabelText(Qt::red, std::string("UNAVAILABLE"));
    server_status_ = -1;
  }

  if (server_status_ != old_status && server_status_ == 1)
    getRunningItems();
}

void AffordanceTemplateRVizClient::setLabelText(QColor color, std::string text) {
  label_palette_->setColor(QPalette::WindowText,color);
  ui_->server_status_label->setPalette(*label_palette_);
  ui_->server_status_label->setText(QString(text.c_str()));     
}

bool AffordanceTemplateRVizClient::tryToLoadRobotFromYAML()
{
  std::string yamlRobotCandidate = "";
  bool foundRobotYaml = false;

  // first try to get robot yaml on param server
  if(nh_.getParam("affordance_templates/robot_yaml", yamlRobotCandidate)) {    
    if(yamlRobotCandidate!="") {
      foundRobotYaml = true;
      ROS_INFO("AffordanceTemplateRVizClient::tryToLoadRobotFromYAML() -- found robot yaml on param server: %s", yamlRobotCandidate.c_str());
    }
  }

  // next try getting it by getting robot name from robot_description
  if(!foundRobotYaml) {
    descriptionRobot_ = getRobotFromDescription();
    if (descriptionRobot_ != "") {
      yamlRobotCandidate = descriptionRobot_ + ".yaml";
      ROS_INFO("AffordanceTemplateRVizClient::tryToLoadRobotFromYAML() -- trying robot name based yaml: %s", yamlRobotCandidate.c_str());
    }
  }

  if(yamlRobotCandidate!="") {
    ROS_INFO("AffordanceTemplateRVizClient::tryToLoadRobotFromYAML() -- searching for Robot: %s", yamlRobotCandidate.c_str());
    map<string,RobotConfigSharedPtr>::const_iterator it = robotMap_.find(yamlRobotCandidate);
    if (it != robotMap_.end() ) {
      int idx = ui_->robot_select->findText(QString(robot_name_map_[yamlRobotCandidate].c_str()), Qt::MatchEndsWith);
      ui_->robot_select->setCurrentIndex(idx);
      setupRobotPanel(yamlRobotCandidate);
      loadConfig();
    } else {
      ROS_WARN("AffordanceTemplateRVizClient::tryToLoadRobotFromYAML() -- no robot yaml found in database of name: %s", yamlRobotCandidate.c_str());
      return false;
    }
  } else {
    ROS_WARN("AffordanceTemplateRVizClient::tryToLoadRobotFromYAML() -- not able to construct a candidate robot yaml");
    return false;
  }

  return true;
}

void AffordanceTemplateRVizClient::updateRobotConfig(const QString& text) {

  // get currently selected robot key
  string key = getLongName(ui_->robot_select->currentText().toUtf8().constData());
  // now update robotMap with current values
  (*robotMap_[key]).name(getLongName(ui_->robot_select->currentText().toUtf8().constData()));
  (*robotMap_[key]).config_package(ui_->config_package->text().toUtf8().constData());
  (*robotMap_[key]).config_file(ui_->config_file->text().toUtf8().constData());
  (*robotMap_[key]).planner_type(ui_->planner_type->text().toUtf8().constData());
  (*robotMap_[key]).frame_id(ui_->frame_id->text().toUtf8().constData());

  string ee_key = ui_->end_effector_select->currentText().toUtf8().constData();
  vector<affordance_template_msgs::GripperActionMap> gm = (*robotMap_[key]).gripper_action();

  for (auto& g: gm) {
    if(g.name == ee_key) {
      g.action = ui_->gripper_action->text().toUtf8().constData();
    }  
  }
  //(*robotMap_[key]).gripper_action(gm);

  vector<float> root_offset(7);
  vector<float> q = util::RPYToQuaternion(ui_->robot_rr->text().toFloat(), ui_->robot_rp->text().toFloat(), ui_->robot_ry->text().toFloat());
  root_offset[0] = ui_->robot_tx->text().toFloat();
  root_offset[1] = ui_->robot_ty->text().toFloat();
  root_offset[2] = ui_->robot_tz->text().toFloat();
  root_offset[3] = q[0];
  root_offset[4] = q[1];
  root_offset[5] = q[2];
  root_offset[6] = q[3];
  (*robotMap_[key]).root_offset(root_offset);
}

void AffordanceTemplateRVizClient::updateEndEffectorGroupMap(const QString& text) {
  string robot_key = getLongName(ui_->robot_select->currentText().toUtf8().constData());
  string key = ui_->end_effector_select->currentText().toUtf8().constData();
  for (auto& e: (*robotMap_[robot_key]).endeffectorMap) {
    if (e.second->name() == key) {
      e.second->name(ui_->ee_name->text().toUtf8().constData());
      e.second->id(ui_->ee_id->text().toInt());
      
      vector<float> pose_offset(7);
      vector<float> q = util::RPYToQuaternion(ui_->ee_rr->text().toFloat(), ui_->ee_rp->text().toFloat(), ui_->ee_ry->text().toFloat());
      pose_offset[0] = ui_->ee_tx->text().toFloat();
      pose_offset[1] = ui_->ee_ty->text().toFloat();
      pose_offset[2] = ui_->ee_tz->text().toFloat();
      pose_offset[3] = q[0];
      pose_offset[4] = q[1];
      pose_offset[5] = q[2];
      pose_offset[6] = q[3];
      e.second->pose_offset(pose_offset);

       /* totx = ui_->ee_totx->text().toFloat();
      toty = ui_->ee_toty->text().toFloat();
      totz = ui_->ee_totz->text().toFloat();
      torr = ui_->ee_torr->text().toFloat();
      torp = ui_->ee_torp->text().toFloat();
      tory = ui_->ee_tory->text().toFloat();*/

      float totx, toty, totz, torr, torp, tory;
      totx = toty = totz = torr = torp = tory = 0;

      vector<float> toq = util::RPYToQuaternion(torr, torp, tory);

      vector<float> tool_offset(7);
      tool_offset[0] = totx;
      tool_offset[1] = toty;
      tool_offset[2] = totz;
      tool_offset[3] = toq[0];
      tool_offset[4] = toq[1];
      tool_offset[5] = toq[2];
      tool_offset[6] = toq[3];
      e.second->tool_offset(tool_offset);
      break;
    }
  }
}

void AffordanceTemplateRVizClient::enableConfigPanel(int state) {
  if (state == Qt::Checked) {
    ui_->groupBox->setEnabled(false);
    ui_->load_config_button->setEnabled(false);
  } else {
    ui_->groupBox->setEnabled(true);
    ui_->load_config_button->setEnabled(true);
  }
}

void AffordanceTemplateRVizClient::refreshCallback() {
  removeAffordanceTemplates();
  getAvailableInfo();
  getRunningItems();

  if(!robot_configured_) {
    tryToLoadRobotFromYAML();
  }
  ROS_WARN("ROBOT NAME: %s", robot_name_.c_str());
  waypoint_display_configured_ = false;
}

void AffordanceTemplateRVizClient::getAvailableInfo() {
  getAvailableTemplates();
  getAvailableRobots();
}

void AffordanceTemplateRVizClient::getAvailableTemplates() {

  ROS_DEBUG("querying available templates");  
  affordance_template_msgs::GetAffordanceTemplateConfigInfo srv;
  if (get_templates_client_.call(srv)) {

    int yoffset = YOFFSET;
    affordanceTemplateGraphicsScene_->clear();
    for (auto& t: srv.response.templates) {

      string image_path = util::resolvePackagePath(t.image_path);
      string filename = t.filename;
      QMap<QString, QVariant> trajectory_map;
      QStringList display_objects;
    
      for (auto& traj: t.trajectory_info) {
    
        QMap<QString, QVariant> waypoint_map;
        for (auto& wp: traj.waypoint_info)
          waypoint_map[QString::number(wp.id)] = QVariant(wp.num_waypoints);
    
        trajectory_map[QString(traj.name.c_str())] = waypoint_map;
      }     
    
      for (auto& objs: t.display_objects)
        display_objects.append(QString(objs.c_str()));

      AffordanceSharedPtr pitem(new Affordance(t.type.c_str(), image_path, trajectory_map, display_objects, filename));
      pitem->setPos(XOFFSET, yoffset);
      yoffset += PIXMAP_SIZE + YOFFSET;
    
      if(!checkAffordance(pitem))
        addAffordance(pitem); 

      affordanceTemplateGraphicsScene_->addItem(pitem.get());
    }

    affordanceTemplateGraphicsScene_->update();
  } else {
    ROS_ERROR("Failed to call service get templates");
  }

  controlStatusUpdate();
}

void AffordanceTemplateRVizClient::getAvailableRobots() {

  ROS_INFO("AffordanceTemplateRVizClient::getAvailableRobots() -- querying available robots");  

  affordance_template_msgs::GetRobotConfigInfo srv;
  if (get_robots_client_.call(srv)) {

    // load stuff for robot config sub panel
    ui_->robot_select->disconnect(SIGNAL(currentIndexChanged(int)));
    ui_->end_effector_select->disconnect(SIGNAL(currentIndexChanged(int)));
    ui_->robot_select->clear();
    ui_->end_effector_select->clear();
     
    for (auto& r: srv.response.robots)  {

      RobotConfigSharedPtr pitem(new RobotConfig(r.filename));
      pitem->uid(r.filename);
      pitem->name(r.name);
      pitem->config_package(r.config_package);
      pitem->config_file(r.config_file);
      pitem->planner_type(r.planner_type);
      pitem->frame_id(r.frame_id);
      pitem->gripper_action(r.gripper_action);

      vector<float> root_offset = util::poseMsgToVector(r.root_offset);
      pitem->root_offset(root_offset);

      for (auto& e: r.end_effectors) {

        EndEffectorConfigSharedPtr eitem(new EndEffectorConfig(e.name));
        eitem->id(int(e.id));

        vector<float> pose_offset = util::poseMsgToVector(e.pose_offset);
        eitem->pose_offset(pose_offset);
        pitem->endeffectorMap[e.name] = eitem;

        vector<float> tool_offset = util::poseMsgToVector(e.tool_offset);
        eitem->tool_offset(tool_offset);
        pitem->endeffectorMap[e.name] = eitem;
      }

      for (auto& p: r.end_effector_pose_data) {

        EndEffectorPoseIDConfigSharedPtr piditem(new EndEffectorPoseConfig(p.name));
        piditem->id(int(p.id))  ;
        piditem->group(p.group);
        pitem->endeffectorPoseMap[p.name] = piditem;
      }

      addRobot(pitem);
      ui_->robot_select->addItem(QString(robot_name_map_[pitem->uid()].c_str()));
    }

    setupRobotPanel(robotMap_.begin()->first);

    // set robot map in helpers
    controls_->setRobotMap(robotMap_);
    waypointDisplay_->setRobotMap(robotMap_);
  } else {
    ROS_ERROR("Failed to call service get robots");
  }
}

void AffordanceTemplateRVizClient::setupRobotPanel(const string& key) {

  string name = (*robotMap_[key]).name();
  string pkg = (*robotMap_[key]).config_package();
  string file = (*robotMap_[key]).config_file();
  string planner = (*robotMap_[key]).planner_type();
  string frame_id = (*robotMap_[key]).frame_id();
  //string gripper_action = (*robotMap_[key]).gripper_action();

  vector<float> root_offset = (*robotMap_[key]).root_offset();

  ui_->robot_name->setText(QString(name.c_str()));
  ui_->config_package->setText(QString(pkg.c_str()));
  ui_->config_file->setText(QString(file.c_str()));
  ui_->planner_type->setText(QString(planner.c_str()));
  ui_->frame_id->setText(QString(frame_id.c_str()));
  //ui_->gripper_action->setText(QString(gripper_action.c_str()));

  ui_->robot_tx->setText(QString::number(root_offset[0]));
  ui_->robot_ty->setText(QString::number(root_offset[1]));
  ui_->robot_tz->setText(QString::number(root_offset[2]));

  vector<float> rpy = util::quaternionToRPY(root_offset[3],root_offset[4],root_offset[5],root_offset[6]);

  ui_->robot_rr->setText(QString::number(rpy[0]));
  ui_->robot_rp->setText(QString::number(rpy[1]));
  ui_->robot_ry->setText(QString::number(rpy[2]));

  ui_->end_effector_select->clear();

  for (auto& e: (*robotMap_[key]).endeffectorMap) {
    ui_->end_effector_select->addItem(e.second->name().c_str());
  }

  setupEndEffectorConfigPanel((*robotMap_[key]).endeffectorMap.begin()->first);

}

void AffordanceTemplateRVizClient::setupEndEffectorConfigPanel(const string& key) {

  string robot_key = getLongName(ui_->robot_select->currentText().toUtf8().constData());

  for (auto& e: (*robotMap_[robot_key]).endeffectorMap) {
    if (e.second->name() == key) {
      ui_->ee_name->setText(e.second->name().c_str());
      ui_->ee_id->setText(QString::number(e.second->id()));
      
      vector<float> pose_offset = e.second->pose_offset();
      ui_->ee_tx->setText(QString::number(pose_offset[0]));
      ui_->ee_ty->setText(QString::number(pose_offset[1]));
      ui_->ee_tz->setText(QString::number(pose_offset[2]));

      vector<float> rpy = util::quaternionToRPY(pose_offset[3],pose_offset[4],pose_offset[5],pose_offset[6]);
      ui_->ee_rr->setText(QString::number(rpy[0]));
      ui_->ee_rp->setText(QString::number(rpy[1]));
      ui_->ee_ry->setText(QString::number(rpy[2]));

      for (auto& g: (*robotMap_[robot_key]).gripper_action()) {
        if (g.name == e.second->name()) {
          ui_->gripper_action->setText(g.action.c_str());
        }
      }
      // FIX ME, THIS NEEDS TO BE DONE FOR TOOL OFFSET
      /*vector<float> tool_offset = e.second->tool_offset();
      ui_->ee_totx->setText(QString::number(tool_offset[0]));
      ui_->ee_toty->setText(QString::number(tool_offset[1]));
      ui_->ee_totz->setText(QString::number(tool_offset[2]));

      vector<float> torpy = util::quaternionToRPY(tool_offset[3],tool_offset[4],tool_offset[5],tool_offset[6]);
      ui_->ee_torr->setText(QString::number(torpy[0]));
      ui_->ee_torp->setText(QString::number(torpy[1]));
      ui_->ee_tory->setText(QString::number(torpy[2]));*/

      break;
    }
  }
}

void AffordanceTemplateRVizClient::changeRobot(int id) {
  QString r = ui_->robot_select->itemText(id);
  setupRobotPanel(getLongName(r.toUtf8().constData()));
}

void AffordanceTemplateRVizClient::changeEndEffector(int id) {
  QString ee = ui_->end_effector_select->itemText(id);
  setupEndEffectorConfigPanel(ee.toUtf8().constData());
}

void AffordanceTemplateRVizClient::changeSaveInfo(int id) {

  if (id >= ui_->save_template_combo_box->count() || ui_->save_template_combo_box->count()==0 ) {
    
    ROS_WARN("AffordanceTemplateRVizClient::changeSaveInfo() -- something funny!! clearing drop down");
    ui_->save_template_combo_box->clear();
    ui_->new_save_type->clear();
    ui_->new_filename->clear();
    ui_->new_image->clear();
    return;
  } 

  QString key = ui_->save_template_combo_box->itemText(id);
  vector<string> stuff = util::split(key.toStdString(), ':');
  
  string class_type = stuff[0];
  int at_id = int(atoi(stuff[1].c_str()));

  QList<QGraphicsItem*> list = affordanceTemplateGraphicsScene_->items();
  for (int i=0; i < list.size(); ++i) {
    // Get the object template class name from the first element in the QGraphicsItem's custom data
    // field. This field is set in the derived Affordance class when setting up the widgets.
    string class_name = list.at(i)->data(CLASS_INDEX).toString().toStdString();

    if (class_name != class_type)
      continue;

    string image_name = list.at(i)->data(IMAGE).toString().toStdString();
    string filename = list.at(i)->data(FILENAME).toString().toStdString();
    
    ROS_DEBUG("AffordanceTemplateRVizClient::changeSaveInfo() -- %s", class_name.c_str());
    ROS_DEBUG("AffordanceTemplateRVizClient::changeSaveInfo() -- %s", image_name.c_str());
    ROS_DEBUG("AffordanceTemplateRVizClient::changeSaveInfo() -- %s", filename.c_str());
    
    vector<string> image_tokens = util::split(image_name, '/');
    vector<string> fname_tokens = util::split(filename, '/');

    ui_->new_save_type->setText(QString(class_name.c_str()));

    if (image_tokens.size() > 0)  {
      string stripped_image = image_tokens[image_tokens.size()-1];
      ROS_DEBUG("AffordanceTemplateRVizClient::changeSaveInfo() -- %s", stripped_image.c_str());
      ui_->new_image->setText(QString(stripped_image.c_str()));
    } else {
      ui_->new_image->setText(QString(image_name.c_str()));      
    }

    if (fname_tokens.size() > 0) {
      string stripped_fname = fname_tokens[fname_tokens.size()-1];
      ROS_DEBUG("AffordanceTemplateRVizClient::changeSaveInfo() -- %s", stripped_fname.c_str());
      ui_->new_filename->setText(QString(stripped_fname.c_str()));
    } else {
      ui_->new_filename->setText(QString(filename.c_str()));
    }
    break;
  }   
}

void AffordanceTemplateRVizClient::selectTemplateTrajectory(const QString& text) 
{
  string template_key = ui_->control_template_box->currentText().toUtf8().constData();
  string traj_key = ui_->control_trajectory_box->currentText().toUtf8().constData();
  ROS_DEBUG("Changing template trajectory for %s to %s", template_key.c_str(), traj_key.c_str());    
  affordance_template_msgs::SetAffordanceTemplateTrajectory srv;
  srv.request.name = template_key;
  srv.request.trajectory = traj_key;
  
  if (set_template_trajectory_client_.call(srv)) {
    ROS_DEBUG("select trajectory response: %d", int(srv.response.success));
    waypoint_display_configured_ = false;
  } else {
    ROS_ERROR("Failed to call service select trajectory");
  }
}

void AffordanceTemplateRVizClient::deleteAffordanceTemplate() {
  if(ui_->server_output_status->currentItem())
    killAffordanceTemplate(ui_->server_output_status->currentItem());
}

void AffordanceTemplateRVizClient::removeAffordanceTemplates() {
  for (auto& pitem: affordanceTemplateGraphicsScene_->items()) {
    affordanceTemplateGraphicsScene_->removeItem(pitem);
  }
  affordanceMap_.clear();
  affordanceTemplateGraphicsScene_->update();
}

int AffordanceTemplateRVizClient::sendAffordanceTemplateAdd(const string& class_name) {
  ROS_INFO("Sending Add Template request for a %s", class_name.c_str());    
  affordance_template_msgs::AddAffordanceTemplate srv;
  srv.request.class_type = class_name;
  if (add_template_client_.call(srv))
  {
    ROS_INFO("Add successful, new %s with id: %d", class_name.c_str(), (int)(srv.response.id));
    return (int)(srv.response.id);
  }
  else
  {
    ROS_ERROR("Failed to call service add_template");
  }
  return -1;
}

void AffordanceTemplateRVizClient::sendAffordanceTemplateKill(const string& class_name, int id) {

  ROS_INFO("Sending kill to %s:%d", class_name.c_str(), id);    

  busy_flag_ = true;

  affordance_template_msgs::DeleteAffordanceTemplate srv;
  srv.request.class_type = class_name;
  srv.request.id = id;
  if (delete_template_client_.call(srv)) {

    ROS_INFO("Delete successful");

    string full_name = class_name + ":" + to_string(id);
    int idx = ui_->save_template_combo_box->findText(QString(full_name.c_str()));
    if ( idx != -1) {

      ui_->save_template_combo_box->removeItem(idx);  
      if(ui_->save_template_combo_box->count()==0) {

        ui_->save_template_combo_box->clear();  
        ui_->new_save_type->clear();
        ui_->new_filename->clear();
        ui_->new_image->clear();
      }
    }

    TemplateInstanceID template_instance = make_pair(class_name, id);
    
    QList<QGraphicsItem*> list = affordanceTemplateGraphicsScene_->items();
    for (int i=0; i < list.size(); ++i) {
      string n = list.at(i)->data(CLASS_INDEX).toString().toStdString();        
      if(class_name==n) {

        for (auto& c: list.at(i)->data(DISPLAY_OBJECTS).toStringList()) {

          std::pair<TemplateInstanceID, std::string> object_key = make_pair(template_instance, c.toStdString());
          //cout << "deleting display object scale for: " << class_name.c_str() << ":" << id << ", object: " << c.toStdString().c_str() << endl; 
          display_object_scale_map.erase(object_key);
          end_effector_adjustment_map.erase(object_key);
        }
      }
    }
  } else {
    ROS_ERROR("Failed to call service delete_template");
  }

  busy_flag_ = false;
}

void AffordanceTemplateRVizClient::killAffordanceTemplate(QListWidgetItem* item) {
  vector<string> template_info = util::split(item->text().toUtf8().constData(), ':');
  int id;
  istringstream(template_info[1]) >> id;
  sendAffordanceTemplateKill(template_info[0], id);
  getRunningItems();
}

void AffordanceTemplateRVizClient::saveAffordanceTemplate() {
  sendSaveAffordanceTemplate();
  getRunningItems();
}

void AffordanceTemplateRVizClient::addTrajectory() {
  sendAddTrajectory();
}

void AffordanceTemplateRVizClient::selectAffordanceTemplate(QListWidgetItem* item) {
  vector<string> template_info = util::split(item->text().toUtf8().constData(), ':');
  int id = ui_->save_template_combo_box->findText(item->text());
  if(id != -1)
    changeSaveInfo(id);

  ui_->save_template_combo_box->setCurrentIndex(id);  

  string class_type = template_info[0];
  int template_id = atoi(template_info[1].c_str());
  selected_template = make_pair(class_type, template_id);

  setupDisplayObjectSliders(selected_template);
}

void AffordanceTemplateRVizClient::selectScaleObject(const QString& object_name) {
  int v;
  std::pair<TemplateInstanceID, std::string> object_info = make_pair(selected_template, object_name.toStdString());

  if ( display_object_scale_map.find(object_info) == display_object_scale_map.end() ) {
    v = ui_->object_scale_slider->minimum() + (ui_->object_scale_slider->maximum() - ui_->object_scale_slider->minimum()) / 2;
    display_object_scale_map[object_info] = v;
  }
  ui_->object_scale_slider->setSliderPosition(display_object_scale_map[object_info]);
  
  if ( end_effector_adjustment_map.find(object_info) == end_effector_adjustment_map.end() ) {
    v = ui_->end_effector_adjustment_slider->minimum() + (ui_->end_effector_adjustment_slider->maximum() - ui_->end_effector_adjustment_slider->minimum()) / 2;
    end_effector_adjustment_map[object_info] = v;   
  }
  ui_->end_effector_adjustment_slider->setSliderPosition(end_effector_adjustment_map[object_info]);
}

void AffordanceTemplateRVizClient::setupDisplayObjectSliders(TemplateInstanceID template_instance) {
  QList<QGraphicsItem*> list = affordanceTemplateGraphicsScene_->items();
  ui_->object_scale_combo_box->clear();
  for (int i=0; i < list.size(); ++i) {
    string class_name = list.at(i)->data(CLASS_INDEX).toString().toStdString();
    if(class_name==template_instance.first) {
      for (auto& c: list.at(i)->data(DISPLAY_OBJECTS).toStringList()) {
        ui_->object_scale_combo_box->addItem(QString(c.toStdString().c_str()));       
        selectScaleObject(c);
      }
    }
  }
}

void AffordanceTemplateRVizClient::updateObjectScale(int value) {
  string current_scale_object = ui_->object_scale_combo_box->currentText().toStdString();
  std::pair<TemplateInstanceID, std::string> object_info = make_pair(selected_template, current_scale_object);
  display_object_scale_map[object_info] = value;
  if(ui_->stream_scale_check_box->isChecked()) {
    sendScaleInfo();
  }
}

void AffordanceTemplateRVizClient::updateEndEffectorScaleAdjustment(int value) {
  string current_scale_object = ui_->object_scale_combo_box->currentText().toStdString();
  std::pair<TemplateInstanceID, std::string> object_info = make_pair(selected_template, current_scale_object);
  end_effector_adjustment_map[object_info] = value;
  if(ui_->stream_scale_check_box->isChecked()) {
    sendScaleInfo();
  }
}

void AffordanceTemplateRVizClient::scaleSliderReleased() {
  string current_scale_object = ui_->object_scale_combo_box->currentText().toStdString();
  std::pair<TemplateInstanceID, std::string> object_info = make_pair(selected_template, current_scale_object);
  display_object_scale_map[object_info] = ui_->object_scale_slider->value();  
  end_effector_adjustment_map[object_info] = ui_->end_effector_adjustment_slider->value();
  sendScaleInfo();
}

void AffordanceTemplateRVizClient::sendScaleInfo() 
{
  string current_template_class = selected_template.first;
  int current_template_id = selected_template.second;

  if( (current_template_class == "") || (current_template_id == -1)) {
    ROS_WARN("AffordanceTemplateRVizClient::sendScaleInfo() -- trying to scale object when nothing is selected");
    return;
  }  

  string current_scale_object = ui_->object_scale_combo_box->currentText().toStdString();
  std::pair<TemplateInstanceID, std::string> object_info = make_pair(selected_template, current_scale_object);
  
  int obj_scale = display_object_scale_map[object_info];
  int ee_scale = end_effector_adjustment_map[object_info];

  double min_value = atof(ui_->object_scale_min->text().toStdString().c_str());
  double max_value = atof(ui_->object_scale_max->text().toStdString().c_str());
  double range = max_value - min_value;

  double obj_scale_value = range*double(obj_scale)/100.0 + min_value;
  double ee_adj_value = (range/2.0)*double(ee_scale)/100.0 + (min_value+range/4.0);

  affordance_template_msgs::ScaleDisplayObjectInfo msg;
  msg.class_type = current_template_class;
  msg.id = current_template_id;
  msg.object_name = current_scale_object;
  msg.scale_factor = obj_scale_value;
  msg.end_effector_scale_factor = ee_adj_value;
  
  ROS_DEBUG("sending scale to template[%s:%d].%s  with scales(%2.2f,%2.2f) " , current_template_class.c_str(), current_template_id, current_scale_object.c_str(), obj_scale_value, ee_adj_value);
  streamObjectScale(msg);
}

void AffordanceTemplateRVizClient::resetScale() {

  string current_scale_object = ui_->object_scale_combo_box->currentText().toStdString();
  std::pair<TemplateInstanceID, std::string> object_info = make_pair(selected_template, current_scale_object);
  int v = ui_->object_scale_slider->minimum() + (ui_->object_scale_slider->maximum() - ui_->object_scale_slider->minimum()) / 2;
  display_object_scale_map[object_info] = v;  
  ui_->object_scale_slider->setSliderPosition(v);  
  v = ui_->end_effector_adjustment_slider->minimum() + (ui_->end_effector_adjustment_slider->maximum() - ui_->end_effector_adjustment_slider->minimum()) / 2;
  end_effector_adjustment_map[object_info] = v;   
  ui_->end_effector_adjustment_slider->setSliderPosition(v);  
  sendScaleInfo();
}

void AffordanceTemplateRVizClient::sendObjectScale(affordance_template_msgs::ScaleDisplayObjectInfo scale_info) {
  affordance_template_msgs::ScaleDisplayObject srv;
  srv.request.scale_info = scale_info;
  if (scale_object_client_.call(srv))
    ROS_INFO("Scale successful");
  else
    ROS_ERROR("Failed to scale objectd");
}

void AffordanceTemplateRVizClient::streamObjectScale(affordance_template_msgs::ScaleDisplayObjectInfo scale_info) {  
   scale_object_streamer_.publish(scale_info);
}

void AffordanceTemplateRVizClient::sendSaveAffordanceTemplate() {
  
  string key = ui_->save_template_combo_box->currentText().toUtf8().constData();
  vector<string> stuff = util::split(key, ':');

  string class_type = stuff[0];
  int id = int(atoi(stuff[1].c_str()));

  affordance_template_msgs::SaveAffordanceTemplate srv;
  srv.request.original_class_type = class_type;
  srv.request.id = id;

  //srv.request.new_class_type = ui_->new_save_type->text().toUtf8().constData());
  srv.request.new_class_type = ui_->new_save_type->text().toStdString();
  srv.request.image = ui_->new_image->text().toStdString();
  srv.request.filename = ui_->new_filename->text().toStdString();
  srv.request.save_scale_updates = (bool)(ui_->save_scaling_check_box->isChecked());

  bool abort_flag = false;

  if(srv.request.new_class_type=="") {
    ROS_WARN("No Class Name entered, not sending anything...");
    abort_flag = true;  
  }
  
  if(srv.request.image=="") {
    ROS_WARN("No Image entered, not sending anything...");
    abort_flag = true;  
  }
  
  if(srv.request.filename=="") {
    ROS_WARN("No Filename entered, not sending anything...");
    abort_flag = true;  
  }

  if(abort_flag)
    return;

  ROS_INFO("Sending save to %s:%d, with image %s to file: %s, scale:%d", srv.request.new_class_type.c_str(), id, srv.request.image.c_str(), srv.request.filename.c_str(), (int)(ui_->save_scaling_check_box->isChecked()));    

  if (save_template_client_.call(srv)) {

    ROS_INFO("Save successful");
    int idx = ui_->save_template_combo_box->findText(QString(key.c_str()));
    if ( idx != -1) {

      ui_->save_template_combo_box->removeItem(idx);  
      if(ui_->save_template_combo_box->count()==0) {

        ui_->save_template_combo_box->clear();
        ui_->new_save_type->clear();
        ui_->new_filename->clear();
        ui_->new_image->clear();
      }
    }
  } else {
    ROS_ERROR("Failed to save template");
  }
}

void AffordanceTemplateRVizClient::sendAddTrajectory() {
  
  string key = ui_->save_template_combo_box->currentText().toUtf8().constData();
  vector<string> stuff = util::split(key, ':');

  string class_type = stuff[0];
  int id = int(atoi(stuff[1].c_str()));

  affordance_template_msgs::AddAffordanceTemplateTrajectory srv;
  srv.request.class_type = class_type;
  srv.request.id = id;
  srv.request.trajectory_name = ui_->new_traj_name->text().toStdString();

  if(srv.request.trajectory_name=="") {
    ROS_WARN("No Trajectory Name entered, not sending anything...");
    return;    
  }
   
  ROS_INFO("Sending add traj [%s] to %s:%d", srv.request.trajectory_name.c_str(), srv.request.class_type.c_str(), id);    

  if (add_trajectory_client_.call(srv))
    ROS_INFO("Add Trajectory successful");
  else
    ROS_ERROR("Failed to add trajectory");
}

void AffordanceTemplateRVizClient::getRunningItems() 
{
  affordance_template_msgs::GetRunningAffordanceTemplates srv;
  if (get_running_client_.call(srv)) {
    ui_->server_output_status->clear();
    ui_->control_template_box->clear();
    ui_->control_trajectory_box->clear();
    
    for (int i=0; i < srv.response.templates.size(); i++) {
      string t = srv.response.templates[i];
      ROS_DEBUG("Found running template: %s", t.c_str());
      ui_->server_output_status->addItem(QString::fromStdString(t.c_str()));
      ui_->server_output_status->item(i)->setForeground(Qt::blue);
      ui_->control_template_box->addItem(QString(t.c_str()));
      int idx = ui_->save_template_combo_box->findText(t.c_str());
      if (idx == -1)
        ui_->save_template_combo_box->addItem(QString(t.c_str()));  
    }
  } else {
    ROS_ERROR("Failed to call service get_running");
  }
  ui_->server_output_status->sortItems();
}

void AffordanceTemplateRVizClient::safeLoadConfig() 
{
  if(ui_->robot_lock->isChecked()) {
    ROS_WARN("Can't load while RobotConfig is locked");
    return;
  }
  loadConfig();
}

void AffordanceTemplateRVizClient::loadConfig() 
{
  ROS_WARN("AffordanceTemplateRVizClient::loadConfig() -- WARNING::taking parameters loaded from original config, not the GUI yet!!! ");

  affordance_template_msgs::LoadRobotConfig srv;

  string key = getLongName(ui_->robot_select->currentText().toUtf8().constData());

  string name = (*robotMap_[key]).name();
  string pkg = (*robotMap_[key]).config_package();
  string file = (*robotMap_[key]).config_file();
  string planner = (*robotMap_[key]).planner_type();
  //string gripper_action = (*robotMap_[key]).gripper_action();
  string frame_id = (*robotMap_[key]).frame_id();
  vector<float> root_offset = (*robotMap_[key]).root_offset();
  vector<affordance_template_msgs::GripperActionMap> gripper_action = (*robotMap_[key]).gripper_action();

  srv.request.robot_config.filename = key;
  srv.request.robot_config.name = name;
  srv.request.robot_config.config_package = pkg;
  srv.request.robot_config.config_file = file;
  srv.request.robot_config.planner_type = planner;
  srv.request.robot_config.gripper_action = gripper_action;
  srv.request.robot_config.frame_id = frame_id;
  srv.request.robot_config.root_offset = util::vectorToPoseMsg(root_offset);
  
  // remove all rows from before
  while(ui_->end_effector_table->rowCount()>0) {
    ui_->end_effector_table->removeCellWidget(0,0);
    ui_->end_effector_table->removeCellWidget(0,1);
    ui_->end_effector_table->removeCellWidget(0,2);
    ui_->end_effector_table->removeCellWidget(0,3);
    ui_->end_effector_table->removeCellWidget(0,4);
    ui_->end_effector_table->removeRow(0);
  }

  int r = 0;
  for (auto& e: (*robotMap_[key]).endeffectorMap) {

    affordance_template_msgs::EndEffectorConfig ee_config;
    ee_config.name =  e.second->name();
    ee_config.id =  e.second->id();
    
    vector<float> pose_offset = e.second->pose_offset();
    ee_config.pose_offset = util::vectorToPoseMsg(pose_offset);

    vector<float> tool_offset = e.second->tool_offset();
    ee_config.tool_offset = util::vectorToPoseMsg(tool_offset);

    // add rows to end effector controls table
    QTableWidgetItem *i= new QTableWidgetItem(QString(e.second->name().c_str()));
    ui_->end_effector_table->insertRow(r);

    ui_->end_effector_table->setItem(r,0,new QTableWidgetItem(QString(e.second->name().c_str())));  // ee_name
    ui_->end_effector_table->setItem(r,2,new QTableWidgetItem(QString("-1")));  // current_waypoint
    ui_->end_effector_table->setItem(r,3,new QTableWidgetItem(QString("-1")));  // num waypoints
    
    ui_->end_effector_table->setItem(r,4,new QTableWidgetItem(QString("NO PLAN")));  // plan status
    QTableWidgetItem* item_status = ui_->end_effector_table->item(r, 4);
    item_status->setTextColor(QColor::fromRgb(255,0,0));
            
    QTableWidgetItem *pItem = new QTableWidgetItem();
    pItem->setCheckState(Qt::Checked);
    ui_->end_effector_table->setItem(r,1,pItem);

    ui_->end_effector_table->setColumnWidth(0,13);
    ui_->end_effector_table->setColumnWidth(1,5);
    ui_->end_effector_table->setColumnWidth(2,5);
    ui_->end_effector_table->setColumnWidth(3,5);
    
    r++;

    srv.request.robot_config.end_effectors.push_back(ee_config);

  }

  for (auto& e: (*robotMap_[key]).endeffectorPoseMap) {
    affordance_template_msgs::EndEffectorPoseData ee_pose;
    ee_pose.name = e.second->name();
    ee_pose.group = e.second->group();
    ee_pose.id = e.second->id();
    srv.request.robot_config.end_effector_pose_data.push_back(ee_pose);
  }

  ui_->end_effector_table->resizeColumnsToContents();
  ui_->end_effector_table->resizeRowsToContents();
    
  robot_name_ = key;
  controls_->setRobotName(robot_name_);
  waypointDisplay_->setRobotName(robot_name_);
  robot_configured_ = true;

  if(ui_->robot_lock->isChecked()) {
    enableConfigPanel(Qt::Checked);
  } else {
    enableConfigPanel(Qt::Unchecked);    
  }
}

void AffordanceTemplateRVizClient::addAffordanceDisplayItem() {
  // Add an object template to the InteractiveMarkerServer for each selected item.
  QList<QGraphicsItem*> list = affordanceTemplateGraphicsScene_->selectedItems();
  AffordanceTemplateStatusInfo::EndEffectorInfo wp_info;

  for (int i=0; i < list.size(); ++i) {
    // Get the object template class name from the first element in the QGraphicsItem's custom data
    // field. This field is set in the derived Affordance class when setting up the widgets.
    string class_name = list.at(i)->data(CLASS_INDEX).toString().toStdString();
    string image_name = list.at(i)->data(IMAGE).toString().toStdString();
    string filename = list.at(i)->data(FILENAME).toString().toStdString();
    
    ROS_WARN("AffordanceTemplateRVizClient::addAffordanceDisplayItem() -- %s", class_name.c_str());
    int idx = sendAffordanceTemplateAdd(class_name);
    if (idx < 0) {
      ROS_ERROR("AffordanceTemplateRVizClient::addAffordanceDisplayItem() something wrong!!");
      return;
    }

    vector<string> image_tokens = util::split(image_name, '/');
    vector<string> fname_tokens = util::split(filename, '/');
    string at_full_name = class_name + ":" + to_string(idx);
    if (ui_->save_template_combo_box->findText(at_full_name.c_str()) == -1) {
      ui_->save_template_combo_box->addItem(QString(at_full_name.c_str()));
      ui_->save_template_combo_box->setItemData(idx,QString(at_full_name.c_str()));
    }

    //cout << "AffordanceTemplateRVizClient::addAffordanceDisplayItem() -- retrieving waypoint info" << endl;
    for (auto& c: list.at(i)->data(TRAJECTORY_DATA).toMap().toStdMap()) {
      string robot_key = getLongName(ui_->robot_select->currentText().toUtf8().constData());
      for (auto& e: (*robotMap_[robot_name_]).endeffectorMap) {
        for (int r=0; r<ui_->end_effector_table->rowCount(); r++ ) {
          if (e.second->name() == ui_->end_effector_table->item(r,0)->text().toStdString() ) {
            ui_->end_effector_table->setItem(r,3,new QTableWidgetItem(QString::number(c.second.toInt())));
          }
        }
      }
    }
  }
  getRunningItems();
}

bool AffordanceTemplateRVizClient::addAffordance(const AffordanceSharedPtr& obj) {
  // check if template is in our map
  if (!checkAffordance(obj)) {
    affordanceMap_[(*obj).key()] = obj;
    return true;
  }
  return false;
}

bool AffordanceTemplateRVizClient::removeAffordance(const AffordanceSharedPtr& obj) {
  // check if template is in our map
  if (checkAffordance(obj)) {
    affordanceMap_.erase((*obj).key());
    return true;
  }
  return false;
}

bool AffordanceTemplateRVizClient::checkAffordance(const AffordanceSharedPtr& obj) {
  if (affordanceMap_.find((*obj).key()) == affordanceMap_.end()) {
    return false;
  }
  return true;
}

bool AffordanceTemplateRVizClient::addRobot(const RobotConfigSharedPtr& obj) {
  // check if robot is in our map
  if (!checkRobot(obj)) {
    robotMap_[(*obj).uid()] = obj;
    robot_name_map_[(*obj).uid()] = createShortName((*obj).uid());
    return true;
  }
  return false;
}

bool AffordanceTemplateRVizClient::removeRobot(const RobotConfigSharedPtr& obj) {
  // check if robot is in our map
  if (checkRobot(obj)) {
    robotMap_.erase((*obj).uid());
    robot_name_map_.erase((*obj).uid());
    return true;
  }
  return false;
}

bool AffordanceTemplateRVizClient::checkRobot(const RobotConfigSharedPtr& obj) {
  if (robotMap_.find((*obj).uid()) == robotMap_.end())
    return false;

  return true;
}


std::string AffordanceTemplateRVizClient::getRobotFromDescription() {
  std::string robot = "";
  urdf::Model model;
  if (!model.initParam("robot_description")) {
    ROS_ERROR("Failed to parse robot_description rosparam");
  } else {
    ROS_INFO("AffordanceTemplateRVizClient::getRobotFromDescription() -- found robot: %s", model.name_.c_str());
    robot = model.name_;
  }
  return robot;
}

void AffordanceTemplateRVizClient::doCommand(Controls::CommandType command_type) {

  string key = ui_->control_template_box->currentText().toUtf8().constData();
  if(key=="") return;
  ROS_INFO("being called from docommand");
  controlStatusUpdate(); // this is probably inefficient. we could store whether things have been updated, but this is safer.

  bool ret = controls_->requestPlan(command_type, ui_->execute_on_plan->isChecked()); 
  if (ret) {
    ROS_INFO("[AffordanceTemplateRVizClient::doCommand(%d)] plan found", (int)command_type);
  } else {
    ROS_ERROR("AffordanceTemplateRVizClient::doCommand(%d) -- computing plan failed", (int)command_type);
  }
  updateStatusFromControls();
}

void AffordanceTemplateRVizClient::updateStatusFromControls() {
  AffordanceTemplateStatusInfo * status = controls_->getTemplateStatusInfo();
  string full_name = status->getName() + to_string(status->getID());
  template_status_info[full_name] = status;
  updateTables(full_name, status->getCurrentTrajectory());
}  

void AffordanceTemplateRVizClient::goToStart() { 
  doCommand(Controls::START);
};

void AffordanceTemplateRVizClient::goToEnd() { 
  doCommand(Controls::END);
};

void AffordanceTemplateRVizClient::stepBackward() { 
  doCommand(Controls::STEP_BACKWARD); 
};

void AffordanceTemplateRVizClient::stepForward() { 
  doCommand(Controls::STEP_FORWARD); 
};

void AffordanceTemplateRVizClient::goToCurrentWaypoint() { 
  doCommand(Controls::CURRENT); 
};

void AffordanceTemplateRVizClient::executePlan() { 
  controls_->executePlan();
  updateStatusFromControls();
}
 
void AffordanceTemplateRVizClient::controlStatusUpdate()
{
  if (ui_->control_template_box->currentText().toStdString().empty() || busy_flag_)
    return;

  affordance_template_msgs::GetAffordanceTemplateStatus srv;
  srv.request.name = ui_->control_template_box->currentText().toStdString();  
  srv.request.trajectory_name = "";
  
  if (get_template_status_client_.call(srv)) {
    ROS_DEBUG("[AffordanceTemplateRVizClient::controlStatusUpdate] Got info for template %s", srv.request.name.c_str());//(int)(srv.response.affordance_template_status.size()));
     
    if (srv.response.affordance_template_status.size() == 0) {
      ROS_WARN("[AffordanceTemplateRVizClient::controlStatusUpdate] No status returned");
      return;
    }

    for (int i = 0; i < srv.response.affordance_template_status.size(); ++i) {
      string full_name = srv.response.affordance_template_status[i].type + ":" + to_string(srv.response.affordance_template_status[i].id);
      if(srv.request.name != full_name) {
        ROS_ERROR("AffordanceTemplateRVizClient::control_status_update() -- wait, something's wrong. requested %s, got %s", srv.request.name.c_str(), full_name.c_str());
        return;
      }

      if(template_status_info.find(full_name)==template_status_info.end())     
        template_status_info[full_name] = new AffordanceTemplateStatusInfo(srv.response.affordance_template_status[i].type, srv.response.affordance_template_status[i].id);

      affordance_template_msgs::AffordanceTemplateStatusConstPtr ptr(new affordance_template_msgs::AffordanceTemplateStatus(srv.response.affordance_template_status[i]));
      template_status_info[full_name]->updateTrajectoryStatus(ptr);
    }

    if(srv.response.trajectory_names.size() == 0)
      return;

    ROS_DEBUG("[AffordanceTemplateRVizClient::controlStatusUpdate] Current Trajectory: %s", srv.response.current_trajectory.c_str());
    template_status_info[srv.request.name]->setCurrentTrajectory(srv.response.current_trajectory);

    ui_->control_trajectory_box->blockSignals(true);

    for(int t=0; t<srv.response.trajectory_names.size(); ++t)
      if(ui_->control_trajectory_box->findText(QString(srv.response.trajectory_names[t].c_str())) == -1)
        ui_->control_trajectory_box->addItem(QString(srv.response.trajectory_names[t].c_str()));

    int id = ui_->control_trajectory_box->findText(QString(srv.response.current_trajectory.c_str()));
    ui_->control_trajectory_box->setCurrentIndex(id);
    
    updateTables(srv.request.name, srv.response.current_trajectory);

    ui_->control_trajectory_box->blockSignals(false);
  } else {
    ROS_ERROR("AffordanceTemplateRVizClient::control_status_update() -- Failed");
  }
}

void AffordanceTemplateRVizClient::updateTables(std::string name, std::string trajectory) 
{
  updateControlsTable(name, trajectory);
  updateWaypointDisplayTable(name, trajectory);   

  controls_->setTemplateStatusInfo(template_status_info[name]);
  waypointDisplay_->setTemplateStatusInfo(template_status_info[name]);
}

void AffordanceTemplateRVizClient::updateControlsTable(std::string name, std::string trajectory)
{
  // set the control GUI with the current trajectories information
  AffordanceTemplateStatusInfo::EndEffectorInfo wp_info = template_status_info[name]->getTrajectoryStatus(trajectory);

  for (int r=0; r<ui_->end_effector_table->rowCount(); r++ ) {
    std::string s = ui_->end_effector_table->item(r,0)->text().toStdString();
    QTableWidgetItem* pItem = ui_->end_effector_table->item(r, 1);
    
    if(!template_status_info[name]->endEffectorInTrajectory(trajectory,s)) {
      // if the table entry isnt in the trajectory, dont let the operator check it
      pItem->setFlags(pItem->flags() & ~Qt::ItemIsEnabled);  

      QTableWidgetItem* item_status = ui_->end_effector_table->item(r, 4);
      item_status->setTextColor(QColor::fromRgb(100,100,100));
      std::string status_string = "N/A";
      item_status->setText(QString(status_string.c_str()));

    } else {
      pItem->setFlags(pItem->flags() | Qt::ItemIsEnabled);   
    }
  }

  for(auto &wp : wp_info) {
    for (auto& e: (*robotMap_[robot_name_]).endeffectorMap) {
      if (e.second->name() != wp.first) {
        // match the waypoint name to the internal end-effector names 
        continue;
      }
      for (int r=0; r<ui_->end_effector_table->rowCount(); r++ ) {
         
        // get the right row for this end-effector
        if (e.second->name() != ui_->end_effector_table->item(r,0)->text().toStdString()) {
          continue;
        }

        QTableWidgetItem* item_idx = ui_->end_effector_table->item(r, 2);
        item_idx->setText(QString::number(wp.second->waypoint_index));

        QTableWidgetItem* item_n = ui_->end_effector_table->item(r, 3);
        item_n->setText(QString::number(wp.second->num_waypoints));

        QTableWidgetItem* item_status = ui_->end_effector_table->item(r, 4);
        if (wp.second->execution_valid) {
          item_status->setTextColor(QColor::fromRgb(0,0,255));
          item_status->setText(QString("SUCCESS"));          
        } else {
          if (wp.second->plan_valid) {
            item_status->setTextColor(QColor::fromRgb(0,255,0));
            std::string status_string = "PLAN -> id[" + to_string(wp.second->waypoint_plan_index) + "]";
            item_status->setText(QString(status_string.c_str()));
          } else {
            item_status->setTextColor(QColor::fromRgb(255,0,0));
            std::string status_string = "NO PLAN -> id[" + to_string(wp.second->waypoint_plan_index) + "]";
            item_status->setText(QString(status_string.c_str()));
          }
        }

      }
    }
  }
}

void AffordanceTemplateRVizClient::updateWaypointDisplayTable(std::string name, std::string trajectory) {
  if(!waypoint_display_configured_) {
    // set the waypoint display GUI with the current trajectories information
    AffordanceTemplateStatusInfo::EndEffectorInfo wp_info = template_status_info[name]->getTrajectoryStatus(trajectory);
    
    // try {
    //   // preserve expansion level
    //   int n = ui_->waypointDisplayTree->topLevelItemCount();
    //   for(int i=0; i<n; i++) {
    //     QTreeWidgetItem *eeTreeItem = ui_->waypointDisplayTree->itemAt(i,0);
    //     QString eeItemStr = eeTreeItem->text(0);
    //     waypointExpansionStatus_[eeItemStr.toStdString()] = eeTreeItem->isExpanded();
    //   }
    // } catch (int e) {
    //   cout << "An exception occurred. Exception Nr. " << e << '\n';
    // }

    waypointDisplay_->setupWaypointDisplayInfo(wp_info);

    // reinstate expansions
    // int n = ui_->waypointDisplayTree->topLevelItemCount();
    // for(int i=0; i<n; i++) {
    //   QTreeWidgetItem *eeTreeItem = ui_->waypointDisplayTree->itemAt(i,0);
    //   QString eeItemStr = eeTreeItem->text(0);
    //   std::string ee_name = eeItemStr.toStdString();
    //   auto search = waypointExpansionStatus_.find(ee_name);
    //   if(search != waypointExpansionStatus_.end()) {
    //     eeTreeItem->setExpanded(waypointExpansionStatus_[ee_name]);
    //   }
    // }
  }
  waypoint_display_configured_ = true;
}

bool AffordanceTemplateRVizClient::endEffectorInTrajectory(AffordanceTemplateStatusInfo::EndEffectorInfo ee_info) {
  std::string s;
  for (int r=0; r<ui_->end_effector_table->rowCount(); r++ ) {

    s = ui_->end_effector_table->item(r,0)->text().toStdString();
    for(auto &ee : ee_info)
      if(ee.first==s)
        return true;
  }
  return false;
}
    
void AffordanceTemplateRVizClient::printTemplateStatus() {

  //std::map<std::string, AffordanceTemplateStatusInfo> template_status_info; 
  for (auto& ts : template_status_info) {

    std::cout << "Stored template name: " << ts.first << std::endl;
    std::cout << " -- template name: " << ts.second->getName() << std::endl;
    std::cout << " -- template id:   " << ts.second->getID() << std::endl;
    std::cout << " -- trajectories:   " << std::endl;
      
    for (auto& traj_info : ts.second->getTrajectoryInfo()) {

      //affordance_template_msgs::AffordanceTemplateStatus status = traj_info->getTrajectoryStatus[traj_info.first];
      std::cout << "  - " << traj_info.first;
      if(traj_info.first == template_status_info[ts.first]->getCurrentTrajectory())
        std::cout << " * ";

      std::cout << std::endl;
    }
  }
}

std::string AffordanceTemplateRVizClient::createShortName(const std::string& long_name) {
  QStringList split_name = QString(long_name.c_str()).split("/");
  return split_name.back().toStdString();
}

std::string AffordanceTemplateRVizClient::getLongName(const std::string& short_name) {
  for (auto n : robot_name_map_)
    if (n.second == short_name)
      return n.first;
}