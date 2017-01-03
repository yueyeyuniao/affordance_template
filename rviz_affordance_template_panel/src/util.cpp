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

#include <rviz_affordance_template_panel/util.h>

using namespace std;

namespace util {

vector<string> &split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

/** \brief Split a string by a delimiter and return it in a vector.
 */
vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

/** \brief KDL helper function convert quaternion to rpy angles
*/
vector<float> quaternionToRPY(float x, float y, float z, float w) {
    vector<float> rpy(3);
    double rr, rp, ry;
    KDL::Rotation::Quaternion(x,y,z,w).GetRPY(rr,rp,ry);
    rpy[0] = (float)rr;
    rpy[1] = (float)rp;
    rpy[2] = (float)ry;
    //cout << "converted quaternion(" << x << ", " << y << ", " << z << ", " << w << ") to rpy(" << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << ")" << endl;
    return rpy;
}

/** \brief KDL helper function convert rpy angles to quaternion
*/
vector<float> RPYToQuaternion(float rr, float rp, float ry) {
    vector<float> q(4);
    double x,y,z,w;
    KDL::Rotation::RPY(rr,rp,ry).GetQuaternion(x,y,z,w);
    q[0] = (float)x;
    q[1] = (float)y;
    q[2] = (float)z;
    q[3] = (float)w;
    return q;
}


/** \brief ROS helper function convert Pose to vector
*/
vector<float> poseMsgToVector(geometry_msgs::Pose msg) {
    vector<float> pose(7);
    pose[0] = (float)(msg.position.x);
    pose[1] = (float)(msg.position.y);
    pose[2] = (float)(msg.position.z);
    pose[3] = (float)(msg.orientation.x);
    pose[4] = (float)(msg.orientation.y);
    pose[5] = (float)(msg.orientation.z);
    pose[6] = (float)(msg.orientation.w);
    return pose;
}

/** \brief ROS helper function convert Pose to vector
*/
geometry_msgs::Pose vectorToPoseMsg(vector<float> pose) {
    
    assert(pose.size() == 7);

    geometry_msgs::Pose msg;
    msg.position.x = pose[0];
    msg.position.y = pose[1];
    msg.position.z = pose[2];
    msg.orientation.x = pose[3];
    msg.orientation.y = pose[4];
    msg.orientation.z = pose[5];
    msg.orientation.w = pose[6];
    return msg;
}

string resolvePackagePath(const string& str) {
    string package_prefix_str ("package://");
    size_t found = str.find(package_prefix_str);
    if (found==string::npos) return str;
    string sub_str = str.substr(package_prefix_str.length(),str.length()-1);
    string delimiter = "/";
    string package_name = sub_str.substr(0, sub_str.find(delimiter));
    string package_path = ros::package::getPath(package_name);
    string file_path = sub_str.substr(package_name.length(),sub_str.length()-1);
    return package_path + file_path;
}

}