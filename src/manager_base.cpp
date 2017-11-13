#include <generic_control_toolbox/manager_base.hpp>

namespace generic_control_toolbox
{
  ManagerBase::ManagerBase(){}
  ManagerBase::~ManagerBase(){}

  bool ManagerBase::getIndex(const std::string &key, int &i) const
  {
    i = -1;

    for (int j = 0; j < manager_index_.size(); j++)
    {
      if (key == manager_index_[j])
      {
        i = j;
        break;
      }
    }

    if (i < 0)
    {
      ROS_ERROR("Key %s was not initialized", key.c_str());
      return false;
    }

    return true;
  }
}
