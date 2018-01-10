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

  bool getArmInfo(const std::string &arm_name, ArmInfo &info)
  {
    ros::NodeHandle nh("~");
    bool has_ft_sensor; // HACK: using the boolean in "ArmInfo" gives an rvalue assignment error that I do not understand

    if (!nh.getParam(arm_name + "/kdl_eef_frame", info.kdl_eef_frame))
    {
      ROS_ERROR("Missing kinematic chain eef (%s/kdl_eef_frame)", arm_name.c_str());
      return false;
    }

    if (!nh.getParam(arm_name + "/gripping_frame", info.gripping_frame))
    {
      ROS_ERROR("Missing kinematic gripping_frame (%s/gripping_frame)", arm_name.c_str());
      return false;
    }

    if (!nh.getParam(arm_name + "/has_ft_sensor", has_ft_sensor))
    {
      ROS_ERROR("Missing sensor info (%s/has_ft_sensor)", arm_name.c_str());
      return false;
    }

    info.has_ft_sensor = has_ft_sensor;

    if (!nh.getParam(arm_name + "/sensor_frame", info.sensor_frame))
    {
      ROS_ERROR("Missing sensor info (%s/sensor_frame)", arm_name.c_str());
      return false;
    }

    if (!nh.getParam(arm_name + "/sensor_topic", info.sensor_topic))
    {
      ROS_ERROR("Missing sensor info (%s/sensor_topic)", arm_name.c_str());
      return false;
    }

    info.name = arm_name;

    return true;
  }
}
