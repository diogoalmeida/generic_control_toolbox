#ifndef __MANAGER_BASE__
#define __MANAGER_BASE__

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

  protected:
    std::vector<std::string> end_effector_;

    /**
      Get the index of the arm with the given end-effector.

      @param eef The end-effector of the kinematic chain.
      @param arm The arm index
      @return true if the arm is found, false otherwise.
    **/
    bool getArmIndex(const std::string &eef, int &arm);
  };
}

#endif
