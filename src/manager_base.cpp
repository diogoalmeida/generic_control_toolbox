#include <generic_control_toolbox/manager_base.hpp>

namespace generic_control_toolbox
{
  ManagerBase::ManagerBase(){}
  ManagerBase::~ManagerBase(){}

  bool ManagerBase::getArmIndex(const std::string &eef, int &arm)
  {
    arm = -1;

    for (int i = 0; i < end_effector_.size(); i++)
    {
      if (eef == end_effector_[i])
      {
        arm = i;
        break;
      }
    }

    if (arm < 0)
    {
      ROS_ERROR("End-effector %s was not initialized", eef.c_str());
      return false;
    }

    return true;
  }
}
