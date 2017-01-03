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

#ifndef GEN_UTILS_H_
#define GEN_UTILS_H_

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <math.h>

namespace rit_utils
{
  enum GroupType
  {
    NONE = 0,
  	JOINT = 1,
  	CARTESIAN = 2,
  	ENDEFFECTOR = 3
  };

  inline std::string groupTypeToString(const GroupType& type)
  {
    switch(type)
    {
      default:
        return "";
    }
    return ""; //TODO
  }

  inline tf::Matrix3x3 doXRot(double v) {
    tfScalar xx(1.0);
    tfScalar xy(0.0);  
    tfScalar xz(0.0);
    tfScalar yx(0.0); 
    tfScalar yy(cos(v)); 
    tfScalar yz(-sin(v));
    tfScalar zx(0.0); 
    tfScalar zy(sin(v));
    tfScalar zz(cos(v));
    tf::Matrix3x3 R(xx, xy, xz, yx, yy, yz, zx, zy, zz);
    return R;
  } 

  inline tf::Matrix3x3 doYRot(double v) {
    tfScalar xx(cos(v));
    tfScalar xy(0.0);  
    tfScalar xz(sin(v));
    tfScalar yx(0.0); 
    tfScalar yy(1.0); 
    tfScalar yz(0.0);
    tfScalar zx(-sin(v)); 
    tfScalar zy(0.0);
    tfScalar zz(cos(v));
    tf::Matrix3x3 R(xx, xy, xz, yx, yy, yz, zx, zy, zz);
    return R;
  } 

  inline tf::Matrix3x3 doZRot(double v) {
    tfScalar xx(cos(v));
    tfScalar xy(-sin(v));  
    tfScalar xz(0.0);
    tfScalar yx(sin(v)); 
    tfScalar yy(cos(v)); 
    tfScalar yz(0.0);
    tfScalar zx(0.0); 
    tfScalar zy(0.0);
    tfScalar zz(1.0);

    // std::cout << "-------------------------------" << std::endl;
    // std::cout << xx << " " << xy << " " << xz << std::endl;
    // std::cout << yx << " " << yy << " " << yz << std::endl;
    // std::cout << zx << " " << zy << " " << zz << std::endl;
    // std::cout << "-------------------------------" << std::endl;

    tf::Matrix3x3 R(xx, xy, xz, yx, yy, yz, zx, zy, zz);
    return R;
  
  } 

  inline tf::Matrix3x3 getRotationFromJointVal(urdf::Vector3 axis, double v) {
    if(axis.x > 0) {
      return doXRot(v);
    } else if(axis.x < 0) {
      return doXRot(-v);
    } else if(axis.y > 0) {
      return doYRot(v);
    } else if(axis.y < 0) {
      return doYRot(-v);
    } else if(axis.z < 0) {
      return doZRot(-v);
    } else {
      return doZRot(v);
    }
  }

  inline tf::Vector3 getTranslationFromJointVal(urdf::Vector3 axis, double v) {
    tf::Vector3 V(0,0,0);
    if(axis.x > 0) {
      V.setX(v);
    } else if(axis.x < 0) {
      V.setX(-v);
    } else if(axis.y > 0) {
      V.setY(v);
    } else if(axis.y < 0) {
      V.setY(-v);
    } else if(axis.z < 0) {
      V.setZ(v);
    } else {
      V.setZ(-v);
    }
    return V;
  }


}

#endif
