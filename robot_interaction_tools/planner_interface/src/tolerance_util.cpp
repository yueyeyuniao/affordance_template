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

#include <planner_interface/tolerance_util.h>

using namespace tolerance_util;

ToleranceUtil::ToleranceUtil(const ros::NodeHandle &nh)
{

  ROS_INFO("ToleranceUtil() -- loading tolerance modes from rosparam"); 
  if (nh.getParam(std::string("tolerances/modes"), tolerance_modes_)) {
    ROS_DEBUG("found %d tolerance modes", (int)tolerance_modes_.size());
    for(auto &m: tolerance_modes_) {
      ROS_DEBUG (" - %s", m.c_str());
    }
  }

  for(auto &mode : tolerance_modes_) {
    std::vector<std::string> tol_types;
    if (nh.getParam(std::string("tolerances/" + mode + "_tolerances"), tol_types)) {
      ROS_DEBUG("found %d tolerances for mode %s", (int)tol_types.size(), mode.c_str());
      ToleranceMap tm;
      for(auto &tt: tol_types) {
        std::vector<double> vals;
        if (nh.getParam(std::string("tolerances/" + mode + "/"+ tt), vals)) {
          ToleranceVal tv;
          if(vals.size() != tv.size()) {
            ROS_WARN(" - wrong number of values (%d)!!", (int)vals.size());
            continue;
          } else {
            for(size_t idx=0; idx<vals.size(); idx++) {
              tv[idx] = vals[idx];
            }
            tm[tt] = tv;
            ROS_DEBUG(" - %s: [%.3f, %.3f, %.3f]", tt.c_str(), (double)tv[0], (double)tv[1], (double)tv[2]); 
          }
        } else {
          ROS_WARN(" - %s -- no values found!!", tt.c_str());
        }
      }
      ROS_DEBUG("setting tolerance map for mode %s", mode.c_str());     
      tols_[mode] = tm;        
    }

    if (nh.getParam(std::string("tolerances/defaults/" + mode), defaults_[mode])) {
      ROS_DEBUG("found default tolerance[%s]: %s", mode.c_str(), defaults_[mode].c_str());
    } else {
      ROS_INFO("no default tolerance found for mode %s", mode.c_str());
    }

  }

}

ToleranceUtil::~ToleranceUtil() {}

std::vector<std::string> ToleranceUtil::getToleranceModes() {
  return tolerance_modes_;
}

bool ToleranceUtil::getToleranceTypes(const std::string &mode, std::vector<std::string> &types) {
  if(!hasMode(mode)) {
    ROS_ERROR("ToleranceUtil::getToleranceTypes() -- no mode: %s", mode.c_str());
    return false;
  } else {
    types.clear();
    for(auto t: tols_[mode])
      types.push_back(t.first);
  }
  return true;
}

bool ToleranceUtil::getToleranceType(const std::string &mode, const ToleranceVal &val, std::string &type) {
  if(hasMode(mode)) {
    for(auto &t: tols_[mode]) {
      if(isApproxToleranceVal(t.second, val)) {
        type = t.first;
        return true;
      }
    }
  }
  return false;
}

bool ToleranceUtil::getToleranceVal(const std::string &mode, const std::string &type, ToleranceVal &val) {
  if(hasMode(mode) && hasType(mode, type)) {
    val = tols_[mode][type];
    return true;
  } 
  return false;
}

bool ToleranceUtil::getDefaultToleranceType(const std::string &mode, std::string &type) {
  if(hasMode(mode)) {
    type = defaults_[mode];
    return true;
  }
  return false;
}

bool ToleranceUtil::getDefaultToleranceVal(const std::string &mode, ToleranceVal &val) {
  std::string type;
  if(getDefaultToleranceType(mode, type)) {
    if(hasType(mode, type)) {
      val = tols_[mode][type];
      return true;
    }
  }
  return false;
}

bool ToleranceUtil::setDefaultToleranceType(const std::string &mode, std::string type) {
  if(hasMode(mode) && hasType(mode,type)) {
    defaults_[mode] = type;
    return true;
  }
  return false;
}

bool ToleranceUtil::hasMode(const std::string &mode) 
{
  return (tols_.find(mode) != std::end(tols_)); 
}

bool ToleranceUtil::hasType(const std::string &mode, const std::string &type) 
{
  if(hasMode(mode)) {
    return (tols_[mode].find(type) != std::end(tols_[mode])); 
  } else {
    ROS_ERROR("ToleranceUtil::hasType() -- no mode %s found", mode.c_str());
    return false;
  }
}

void ToleranceUtil::printVal(const ToleranceVal &val) {
  ROS_INFO("[%.3f, %.3f, %.3f]", (double)val[0], (double)val[1], (double)val[2]); 
}

void ToleranceUtil::printVal(const std::string &mode, const std::string &type) {
  ToleranceVal tv;
  getToleranceVal(mode, type, tv);
  printVal(tv);
}
        
void ToleranceUtil::printModes() {
  ROS_INFO("TOLERANCE MODES:");
  for(auto &m: tols_) {
    ROS_INFO(" - %s", m.first.c_str());
  }
}
   
void ToleranceUtil::printTypes(const std::string &mode) {
  if(hasMode(mode)) {
    ROS_INFO("TOLERANCE TYPES (%s):" , mode.c_str());
    for(auto &t: tols_[mode]) {
      ToleranceVal tv;
      getToleranceVal(mode,t.first,tv);
      ROS_INFO(" - %s: [%.3f, %.3f, %.3f]", t.first.c_str(), (double)tv[0], (double)tv[1], (double)tv[2]);
    }
  }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "tolerance_util_test");
  ros::NodeHandle nh("~");

  ToleranceUtil tol(nh);

  ROS_INFO(" ");
  ROS_INFO("==============");
  ROS_INFO("TOLERANCE TEST");
  ROS_INFO("==============");

  tol.printModes();
  std::vector<std::string> modes = tol.getToleranceModes();
  for(auto &m : modes) {
    tol.printTypes(m);
    std::string def_type;
    ToleranceVal tv;
    tol.getDefaultToleranceType(m, def_type);
    tol.getDefaultToleranceVal(m, tv);
    ROS_INFO("Default Type (mode: %s): %s", m.c_str(), def_type.c_str());
    tol.printVal(tv);
  }

  ToleranceVal tv_test_1 = {0.393, 0.035, 0.035};
  std::string  tv_type_1 = "eighth_cyl";
  ToleranceVal tv_test_2 = {0.01, 0.01, 0.01};
  std::string  tv_type_2 = "centimeter";
  std::string matched_type;
  if(tol.getToleranceType("orientation", tv_test_1, matched_type) ) {
    ROS_INFO("matched tolerance test val 1 to %s -- result=%d", matched_type.c_str(), (int)(matched_type==tv_type_1));
  }
  if(tol.getToleranceType("position", tv_test_2, matched_type) ) {
    ROS_INFO("matched tolerance test val 2 to %s -- result=%d", matched_type.c_str(), (int)(matched_type==tv_type_2));
  }
    
  return 0;

}



