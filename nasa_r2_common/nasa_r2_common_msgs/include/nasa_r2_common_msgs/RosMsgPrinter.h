#ifndef ROS_MSG_PRINTER_H
#define ROS_MSG_PRINTER_H

#include <nasa_r2_common_msgs/PowerState.h>
#include <nasa_r2_common_msgs/JointControlData.h>

namespace RosMsgPrinter
{
    std::string powerStateToString(const nasa_r2_common_msgs::PowerState& powerState);
    std::string jointControlModeToString(const nasa_r2_common_msgs::JointControlMode& controlMode);
    std::string jointControlCommandModeToString(const nasa_r2_common_msgs::JointControlCommandMode& commandMode);
    std::string jointControlCalibrationModeToString(const nasa_r2_common_msgs::JointControlCalibrationMode& calibrationMode);
    std::string jointControlClearFaultModeToString(const nasa_r2_common_msgs::JointControlClearFaultMode& clearFaultMode);
    std::string jointControlCoeffsLoadedStateToString(const nasa_r2_common_msgs::JointControlCoeffState& coeffState);

} //RosMsgPrinter

#endif // ROS_MSG_PRINTER_H
