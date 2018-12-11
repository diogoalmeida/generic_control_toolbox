#ifndef __MANAGER_BASE__
#define __MANAGER_BASE__

#include <generic_control_toolbox/ArmInfo.h>
#include <ros/ros.h>

namespace generic_control_toolbox
{
/**
  Provides common functionality to all the manager classes.
**/
class ManagerBase
{
 public:
  ManagerBase();
  virtual ~ManagerBase();
};

/**
  Fill in an ArmInfo structure with the parameters of the arm.
  Uses the ros parameter server.

  Searches for "arm_name/kdl_eef_frame", "arm_name/gripping_frame",
  "arm_name/has_state_", "arm_name/sensor_frame" and "arm_name/sensor_topic",
  relative to the calling node's name.

  @param arm_name The arm name used in the ros parameter server.
  @param info The ArmInfo structure to fill in.
  @param nh The ros nodehandle

  @return True for success, false otherwise
**/
bool getArmInfo(const std::string &arm_name, ArmInfo &info,
                ros::NodeHandle &nh);

/**
  getArmInfo overload which creates a private nodehandle
**/
bool getArmInfo(const std::string &arm_name, ArmInfo &info);
}  // namespace generic_control_toolbox

#endif
