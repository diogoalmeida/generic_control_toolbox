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
    std::vector<std::string> manager_index_;

    /**
      Get the index for the given key stored in the manager index vector.

      @param key The key.
      @param i The index
      @return true if the key is found, false otherwise.
    **/
    bool getIndex(const std::string &key, int &i) const;
  };
}

#endif
