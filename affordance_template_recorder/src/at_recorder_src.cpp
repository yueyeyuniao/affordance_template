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
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
//#include <tf/tfScalar.h>
#include <iostream>
#include <fstream>
#include <kdl/frames.hpp>
#include <linux/input.h>
#include <fcntl.h>
#include <termios.h>
#include <vector>
#include <json.h>
#include <string>
#include <sstream>
#include <streambuf>

std::string base_frame;
std::string target_frame;
std::string hand;
std::string AT_name="Valve", object_name="Valve", instance="0";

struct pose_struct{
	float x, y, z, roll, pitch, yaw;
};

std::vector<pose_struct> pose_vector;

//std::ofstream op_file_id;

int kbhit(void);
void record_pose();
void delete_pose();
void reset_array();
void save_to_jason();
bool process_user_option(char);
void display_recorded_poses();
void write_to_file();

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

void record_pose()
{
	KDL::Frame T_ee_obj, T_abs_obj, T_abs_ee;
	KDL::Frame T_ee_abs;
	KDL::Frame T_viz_offset;

	tf::StampedTransform transform;
	geometry_msgs::Vector3 position;
	tf::Matrix3x3 tf_matrix;
	double roll, pitch, yaw;

	tf::TransformListener listener;

	pose_struct pose_struct_instance;

	try {
		listener.waitForTransform(base_frame, target_frame, ros::Time(0), ros::Duration(3.0));
		listener.lookupTransform(base_frame, target_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
    	std::cout << "Error" << std::endl;
        ROS_ERROR("%s",ex.what());
       	ros::Duration(1.0).sleep();
       	return;
    }
    
    position.x = transform.getOrigin().x();
    position.y = transform.getOrigin().y();
    position.z = transform.getOrigin().z();

    tf_matrix.setRotation(transform.getRotation());
    tf_matrix.getRPY(roll, pitch, yaw);

    //Populate the translation elements of T_ee_obj
    T_ee_obj.p.data[0] = position.x;
    T_ee_obj.p.data[1] = position.y;
    T_ee_obj.p.data[2] = position.z;

    //Populate the rotation elements of T_ee_obj
    T_ee_obj.M = KDL::Rotation::RPY(roll, pitch, yaw);

    /*std::cout << "hand wrt object" << std::endl;
    std::cout << "position: " << position.x << "\t" << position.y << "\t" << position.z << "\t" << std::endl;
    std::cout << "rotation: " << roll << "\t" << pitch << "\t" << yaw << "\t" << std::endl << std::endl;*/

    //Populate the translation elements of T_ee_abs
    if(hand=="right") {
      T_ee_abs.p.data[0] = 0.1;
      T_ee_abs.p.data[1] = -0.2;
      T_ee_abs.p.data[2] = 0.0;
      T_ee_abs.M = KDL::Rotation::RPY(0.0, 1.57, 3.14);
    } else {
      T_ee_abs.p.data[0] = 0.1;
      T_ee_abs.p.data[1] = 0.2;
      T_ee_abs.p.data[2] = 0.0;
      T_ee_abs.M = KDL::Rotation::RPY(0.0, 1.57, 3.14);
    }

    /*std::cout << "yaml: hand wrt abstract" << std::endl;
    std::cout << "position: " << T_ee_abs.p.data[0] << "\t" << T_ee_abs.p.data[1] << "\t" << T_ee_abs.p.data[2] << "\t" << std::endl;
    std::cout << "rotation: " << 0.0 << "\t" << 1.57 << "\t" << 3.14 << "\t" << std::endl << std::endl;*/


    if(hand=="right") {
      T_viz_offset.p.data[0] = 0.0;
      T_viz_offset.p.data[1] = -0.1;
      T_viz_offset.p.data[2] = 0.0;
      T_viz_offset.M = KDL::Rotation::RPY(0.0, 0.0, -1.57);
    } else {
      T_viz_offset.p.data[0] = 0.0;
      T_viz_offset.p.data[1] = 0.1;
      T_viz_offset.p.data[2] = 0.0;
      T_viz_offset.M = KDL::Rotation::RPY(0.0, 0.0, 1.57);
    }

    //T_ee_obj = T_abs_obj * T_ee_abs
    //T_abs_obj = T_ee_obj * inv(T_ee_abs)
    T_abs_obj = T_ee_obj * T_viz_offset.Inverse() * T_ee_abs.Inverse();

    /*std::cout << "T_abs_obj" << std::endl;
    std::cout << T_abs_obj.M.data[0] << "\t" <<	T_abs_obj.M.data[1] << "\t" <<	T_abs_obj.M.data[2] << "\t" << T_abs_obj.p.data[0] << std::endl;
    std::cout << T_abs_obj.M.data[3] << "\t" <<	T_abs_obj.M.data[4] << "\t" <<	T_abs_obj.M.data[5] << "\t" << T_abs_obj.p.data[1] << std::endl;
    std::cout << T_abs_obj.M.data[6] << "\t" <<	T_abs_obj.M.data[7] << "\t" <<	T_abs_obj.M.data[8] << "\t" << T_abs_obj.p.data[2] << std::endl;
    std::cout << std::endl;*/

    T_abs_obj.M.GetRPY(roll, pitch, yaw);

    /*std::cout << "abstract wrt object" << std::endl;
    std::cout << "\"xyz\": [" << T_abs_obj.p.data[0] << ",  " << T_abs_obj.p.data[1] << ",  " << T_abs_obj.p.data[2] << "]," << std::endl;
    std::cout << "\"rpy\": [" << roll << ",  " << pitch << ",  " << yaw << "]" << std::endl << std::endl;*/

    pose_struct_instance.x = T_abs_obj.p.data[0];
    pose_struct_instance.y = T_abs_obj.p.data[1];
    pose_struct_instance.z = T_abs_obj.p.data[2];
    pose_struct_instance.roll = roll;
    pose_struct_instance.pitch = pitch;
    pose_struct_instance.yaw = yaw;

    //pose_vector.resize(pose_vector.size() + 1);
    pose_vector.push_back(pose_struct_instance);

    std::cout << "Array size:" << pose_vector.size() << std::endl;

    display_recorded_poses();

}

void delete_pose()
{
	if (!pose_vector.empty())
		pose_vector.pop_back();
	else
		std::cout << "No poses stored in the scratch pad" << std::endl;

	display_recorded_poses();
}

void reset_array()
{
	if (!pose_vector.empty())
		pose_vector.clear();
	else
		std::cout << "No poses stored in the scratch pad" << std::endl;

	display_recorded_poses();
}

void display_recorded_poses()
{
	if (!pose_vector.empty()){
		std::cout << "Stored poses are:" << std::endl;
		for(int i=0; i<pose_vector.size(); i++)
			std::cout << "x:" << pose_vector[i].x << ",\t" << "y:" << pose_vector[i].y << ",\t" << "z:" << pose_vector[i].z << ",\t" 
				<< "roll:" << pose_vector[i].roll << ",\t" << "pitch:" << pose_vector[i].pitch << ",\t" << "yaw:" << pose_vector[i].yaw << "\n";
	}
}

void write_to_file()
{
	std::string fileName, trajName;
	static std::ofstream op_file_id;
	static bool first_write=true;
	int new_file_choice;

	if (!first_write){	//not the first write
		std::cout << "Do you want to write it to the same file or different file?" << std::endl;
		std::cout << "Enter 1 for same and 2 for different, and press Enter" << std::endl;
		std::cin >> new_file_choice;

		if (new_file_choice ==2){	//user chooses a different file name
			op_file_id.close();

			std::cout << "Enter the filename to record the poses into: " << std::endl;
			std::cin >> fileName;
			fileName = fileName + ".txt";
			op_file_id.open(fileName.c_str());

			std::cout << "Enter the trajectory name: " << std::endl;
			std::cin >> trajName;
		}
		else{	//user chooses same file as last time
			op_file_id << std::endl;
			op_file_id << "-------------------------" << std::endl;
			op_file_id << std::endl;

			std::cout << "Enter the trajectory name: " << std::endl;
			std::cin >> trajName;
		}
	}
	else{	//first write
		first_write=!first_write;
		std::cout << "Enter the filename to record the poses into: " << std::endl;
		std::cin >> fileName;
		fileName = fileName + ".txt";
		op_file_id.open(fileName.c_str());

		std::cout << "Enter the trajectory name: " << std::endl;
		std::cin >> trajName;
	}

	op_file_id << "Trajectory name: " << trajName << std::endl;
	op_file_id << "AT name: " << AT_name << std::endl;
	op_file_id << "object name: " << object_name << std::endl;
	op_file_id << "instance: " << instance << std::endl;
	op_file_id << "hand: " << hand << std::endl;
	op_file_id << "target_frame:" << target_frame << std::endl;		

	while (!pose_vector.empty()){
			op_file_id << "[x:" << pose_vector[0].x << ",\t" << "y:" << pose_vector[0].y << ",\t" << "z:" << pose_vector[0].z << ",\t" 
				<< "roll:" << pose_vector[0].roll << ",\t" << "pitch:" << pose_vector[0].pitch << ",\t" << "yaw:" << pose_vector[0].yaw 
				<< "]" << std::endl;

			for(int i=0; i<pose_vector.size()-1; i++)
				pose_vector[i]=pose_vector[i+1];
			pose_vector.pop_back();
	}

	std::cout << "No poses stored in the scratch pad. All poses written to file" << std::endl;

}

Json::Value write_end_effector_waypoint_array_object(pose_struct pose)
{
	
	Json::Value waypoint_array_object, origin_object, rpy_array, xyz_array, controls_object, rpy_mask_array, xyz_mask_array;

	rpy_array.append(pose.roll); rpy_array.append(pose.pitch); rpy_array.append(pose.yaw);
	xyz_array.append(pose.x); xyz_array.append(pose.y); xyz_array.append(pose.z);
	rpy_mask_array.append(true); rpy_mask_array.append(true); rpy_mask_array.append(true);
	xyz_mask_array.append(true); xyz_mask_array.append(true); xyz_mask_array.append(true);

	waypoint_array_object["origin"]["rpy"] = rpy_array;
	waypoint_array_object["origin"]["xyz"] = xyz_array;
	waypoint_array_object["ee_pose"] = 0;
	waypoint_array_object["controls"]["rpy"]= rpy_mask_array;
  waypoint_array_object["controls"]["scale"]= 0.25;
	waypoint_array_object["controls"]["xyz"]= xyz_mask_array;
	waypoint_array_object["display_object"] = object_name;

	return (waypoint_array_object);
}

void write_end_effector_waypoint_root(Json::Value &waypoint_root)
{
	
	Json::Value waypoint_array_object;

	while (!pose_vector.empty())
	{
		waypoint_array_object = write_end_effector_waypoint_array_object(pose_vector[0]);

		for(int i=0; i<pose_vector.size()-1; i++)
			pose_vector[i]=pose_vector[i+1];
		pose_vector.pop_back();

		waypoint_root.append(waypoint_array_object);
	}
}

void write_end_effector_group_root(Json::Value &group_root)
{
	
	Json::Value group_array_object, waypoint_root;
	int id;

	if (hand.compare("left")==0)
		id = 0;
	else
		id = 1;

	write_end_effector_waypoint_root(waypoint_root);

	group_array_object["id"] = id;
	group_array_object["end_effector_waypoint"] = waypoint_root;

	group_root.append(group_array_object);
}

void write_end_effector_trajectory_array_object(Json::Value &traj_root, std::string trajName)
{
	
	Json::Value traj_array_object, group_root;

	write_end_effector_group_root(group_root);
	
	traj_array_object["end_effector_group"] = group_root;
	traj_array_object["name"] = trajName;

	traj_root.append(traj_array_object);

}

void save_to_jason()
{
	std::string fileName, trajName;
    bool found_traj=false;
    Json::StyledWriter styledWriter;
    static std::ofstream op_file_id;

    //when the user hits 'S', ask the name of the trajectory to save
    std::cout << "Enter the trajectory name: " << std::endl;
	std::cin >> trajName;

    //The file to save the new trajectory points was already entered at the beginning of running the program
    fileName = "/home/karan/DRC/affordance_templates/affordance_template_library/templates/" + AT_name + ".json";
    //fileName = "valve.json";

    std::ifstream t(fileName.c_str());
    std::string json_file_string;

    t.seekg(0, std::ios::end);
    json_file_string.reserve(t.tellg());
    t.seekg(0, std::ios::beg);

    json_file_string.assign((std::istreambuf_iterator<char>(t)),
    	std::istreambuf_iterator<char>());

    // Let's parse it
 	Json::Value root;
 	Json::Reader reader;
 	bool parsedSuccess = reader.parse(json_file_string, 
                                   root, 
                                   false);
  
 	if(not parsedSuccess)
 	{
   	// Report failures and their locations in the document.
   	std::cout<<"Failed to parse JSON"<<std::endl 
       <<reader.getFormatedErrorMessages()
       <<std::endl;
   	exit(1);
 	}

 	Json::Value traj_root = root["end_effector_trajectory"];

    //check if the user entered trajectory exists
    for (int index = 0; index < traj_root.size(); ++index) {
    	
    	if (trajName.compare(traj_root[index].get("name", "NULL").asString()) == 0) {
    		found_traj=true;
    		break;
    	}
    }

    if (!found_traj) {
    	//if it doesn't, create an object under end_effector_trajectory array of objects
    	std::cout << "Entered trjectory does not exist" << std::endl;
    	std::cout << "Creating new trajectory and writing to Json file" << std::endl;

    	op_file_id.open(fileName.c_str());
    	write_end_effector_trajectory_array_object(traj_root, trajName);
    	root["end_effector_trajectory"] = traj_root;
    	op_file_id << styledWriter.write(root);
    	op_file_id.close();
    }
    else {
    	std::cout << "Entered trjectory exists" << std::endl;
    	found_traj=false;
    }

       	//if it does, check if the end-effector group exists (from hand info entered at the beginning of the program)
       		//if it doesn't, create a new object under end_effector_group array of objects
       		//if it does, add an object under end_effector_waypoint array of objects

}

bool process_user_option(char user_input)
{
	switch(user_input)
	{
		case 'R':
		case 'r': record_pose();
				  break;

		case 'D':
		case 'd': delete_pose();
				  break;

		case 'T':
		case 't': reset_array();
				  break;

		case 'S':
		case 's': save_to_jason();
				  break;

		case 'V':
		case 'v': display_recorded_poses();
				  break;

		case 'W':
		case 'w': write_to_file();
				  break;

		case 'Q':
    	case 'q': return false;

    	default: std::cout << std::endl;
    			 std::cout << "Please enter 'R', 'D', 'T', 'S', or 'Q' to quit" << std::endl;
	}

	return true;
}

int main(int argc, char** argv)
{
  //std::cout << argc << std::endl;
	//std::sstream ss;

	std::cout << "Enter affordance template name: ";
	std::cin >> AT_name;
	
	std::cout << "Enter object name: ";
	std::cin >> object_name;
	
	std::cout << "Enter instance: ";
	std::cin >> instance;
	
	/*ss << instance;
	std::string str = ss.str();*/

	base_frame = AT_name + ":" + instance + "/" + object_name + ":" + instance;

	//std::cout << base_frame << std::endl;

	std::cout << "Enter hand: ";
	std::cin >> hand;
	std::cout << std::endl;

	//std::cout << hand << std::endl;

  /*if (argc!=3)
  {
    std::cout << "Three arguments are needed" << std::endl;
    std::cout << "./at_recorder [obj_frame] [right|left]" << std::endl;
    std::cout << "Quitting!" << std::endl;

    exit(1);
  }
  
  base_frame = argv[1];
  hand = argv[2];*/
  target_frame = "";

  if(hand=="right") {
    target_frame = "r_end";
  } else if(hand=="left") {
    target_frame = "l_end";
  } else {
    std::cout << "The third argument should be \"right\" or \"left\""  << std::endl;
    std::cout << "Quitting!" << std::endl;
    exit(1);	
  }
  
  bool menu_displayed=false;
  //op_file_id.open("op_file.txt");
  
  ros::init(argc, argv, "record_traj_point_node");
  ros::NodeHandle node;
  ros::Rate rate(100);

  while (node.ok())
  {

    if (!menu_displayed)
    {
      std::cout << std::endl;
      std::cout << "Affordance Template keyboard interaction options:" << std::endl;
      std::cout << "Press 'R' or 'r' to record pose to scratch pad" << std::endl;
      std::cout << "Press 'D' or 'd' to delete previously recorded pose" << std::endl;
      std::cout << "Press 'T' or 't' to reset and start recording again" << std::endl;
      std::cout << "Press 'W' or 'w' to transfer the recorded to file" << std::endl;
      std::cout << "Press 'S' or 's' to save the recorded poses to the JASON file" << std::endl;
      std::cout << "Press 'Q' or 'q' to quit the program" << std::endl;

      menu_displayed=true;
    }

  	if (kbhit())
    {
    	menu_displayed=false;
    	char key_pressed = getchar();
    	std::cout <<std::endl;
    	if(!process_user_option(key_pressed))
    		break;
    }

    rate.sleep();

  }

  //op_file_id.close();
  
  return 0;

}



