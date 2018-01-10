#ifndef __MANAGER_BASE__
#define __MANAGER_BASE__

#include <ros/ros.h>
#include <generic_control_toolbox/ArmInfo.h>

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

  protected:
    std::vector<std::string> manager_index_;

    /**
      Get the index for the given key stored in the manager index vector.

      @param key The key.
      @param i The index
      @return true if the key is found, false otherwise.
    **/
    bool getIndex(const std::string &key, int &i) const;
  };

  /**
    Fill in an ArmInfo structure with the parameters of the arm.
    Uses the ros parameter server.

    Searches for "arm_name/kdl_eef_frame", "arm_name/gripping_frame",
    "arm_name/has_state_", "arm_name/sensor_frame" and "arm_name/sensor_topic",
    relative to the calling node's name.

    @param arm_name The arm name used in the ros parameter server.
    @param info The ArmInfo structure to fill in.
  **/
  bool getArmInfo(const std::string &arm_name, ArmInfo &info);
}

#endif
