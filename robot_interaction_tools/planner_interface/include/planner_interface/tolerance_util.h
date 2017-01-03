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

#ifndef TOLERANCE_UTIL_H_
#define TOLERANCE_UTIL_H_

#include <ros/ros.h>

namespace tolerance_util
{

    typedef std::array<double, 3> ToleranceVal;
    typedef std::map<std::string, ToleranceVal> ToleranceMap;

   /** @brief The Tolerance Utility Class.
    *
    *   This class loads/stored the tolerance ranges from rosparam.  Tolerances are 
    *   grouped by "modes" such as "position" or "orientation."  Each mode has a list 
    *   of "types" such as "sphere," "half_cyl," or "centimenter."  Each type has a 
    *   string classifier, but really representes a (symmetric) 3-dimensional vector.
    *   e.g., The "centimenter" position tolerance corresponds to the vector [0.01, 0.01, 0.01].
    *
    */
    class ToleranceUtil
    {

      public:

        /** @brief Cestructor.
         *
         * @param nh a ROS Node Handle
         */
        ToleranceUtil(const ros::NodeHandle &nh);
        
        /** @brief Destructor.
         *
         */
        ~ToleranceUtil();

        /** @brief Gets the list of known tolerance modes 
         *
         * @return the list of modes 
         */
        std::vector<std::string> getToleranceModes();
        
        /** @brief Gets the list of tolerance types for a mode
         *
         * @param mode the mode to get the types of 
         * @param type the (returned) list of types for the mode 
         * @return true if the mode was found
         */
        bool getToleranceTypes(const std::string &mode, std::vector<std::string> &types);

        /** @brief Gets the tolerance type for a mode that matches the given 3D value
         *
         * @param mode the mode to check the type for
         * @param val the 3D tolerance value to check for
         * @param type the (returned) type the value matches
         * @return true if the type was found
         */
        bool getToleranceType(const std::string &mode, const ToleranceVal &val, std::string &type);
        
        /** @brief Gets the 3D tolerance value for a given mode and type
         *
         * @param mode the mode the type is in
         * @param type the type to get the value of
         * @param val the (returned) 3D tolerance value
         * @return true if mode and type exists
         */
        bool getToleranceVal(const std::string &mode, const std::string &type, ToleranceVal &val);
        
        /** @brief Gets the default tolerance type identifier for the given mode
         *
         * @param mode the mode to get the default type of
         * @param type the (returned) default tolerance type
         * @return true if mode exists
         */
        bool getDefaultToleranceType(const std::string &mode, std::string &type);
        
        /** @brief Gets the default 3D tolerance value for the given mode
         *
         * @param mode the mode to get the default type of
         * @param val the (returned) default 3D tolerance value
         * @return true if mode exists
         */
        bool getDefaultToleranceVal(const std::string &mode, ToleranceVal &val);

        /** @brief Sets the default tolerance type for the given mode
         *
         * @param mode the mode to set the default type of
         * @param type the new default type
         * @return true if set was successful
         */
        bool setDefaultToleranceType(const std::string &mode, std::string type);
      
        /** @brief Prints the given 3D tolerance value.
         *
         * @param tv the tolerance value to pring
         *
         */
        void printVal(const ToleranceVal &val);
        
        /** @brief Prints the 3D Tolerance value for a type.
         *
         * @param mode the mode the type is in 
         * @param type the type to print the value for
         *
         */
        void printVal(const std::string &mode, const std::string &type);
        
        /** @brief Prints all known modes.
         *
         */
        void printModes();
        
        /** @brief Prints all types of mode.
         *
         * @param mode the mode to print the types of 
         *
         */
        void printTypes(const std::string &mode);

        /** @brief checks to see if is a known  mode.
         *
         * @param mode the mode to check 
         * @return if known mode
         */
        bool hasMode(const std::string &mode);
        
        /** @brief checks to see if is a known type in given mode.
         *
         * @param mode the mode to check type for 
         * @param type the type to check
         * @return if known type
         */
        bool hasType(const std::string &mode, const std::string &type);

      protected:

        std::vector<std::string> tolerance_modes_;
        std::map<std::string, ToleranceMap> tols_;
        std::map<std::string, std::string> defaults_;

      inline bool isApproxDouble(double a, double b, double epsilon = 0.001)
      {
        return std::abs(a - b) < epsilon;
      }

      inline bool isApproxToleranceVal(ToleranceVal a, ToleranceVal b, double epsilon = 0.001)
      {
        return (isApproxDouble(a[0],b[0],epsilon) && isApproxDouble(a[1],b[1],epsilon) && isApproxDouble(a[2],b[2],epsilon));
      }

    };

    typedef boost::shared_ptr<ToleranceUtil> ToleranceUtilSharedPtr;
};

#endif
