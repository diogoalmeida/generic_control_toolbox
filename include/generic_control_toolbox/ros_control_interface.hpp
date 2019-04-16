#ifndef __ROS_CONTROL_INTERFACE__
#define __ROS_CONTROL_INTERFACE__

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <urdf/model.h>
#include <generic_control_toolbox/controller_template.hpp>

namespace generic_control_toolbox
{
enum JointType
{
  EFFORT,
  VELOCITY,
  POSITION
};

/**
  Embeds a ControllerBase within the ros control architecture.
**/
template <class JointInterface>
class RosControlInterface
    : public controller_interface::Controller<JointInterface>
{
 public:
  RosControlInterface();

  bool init(JointInterface* hw, ros::NodeHandle& nh);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);

 protected:
  std::shared_ptr<ControllerBase> controller_;

 private:
  ros::NodeHandle nh_;
  JointType joint_type_;
  unsigned int n_joints_;
  std::vector<std::string> joint_names_;
  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;
  std::vector<hardware_interface::JointHandle> joints_;
};

// Implementation in header file due to being a template class.

template <class JointInterface>
RosControlInterface<JointInterface>::RosControlInterface()
{
  nh_ = ros::NodeHandle("~");
  controller_ = NULL;

  // check interface type
  JointInterface* hw = new JointInterface();
  if (hardware_interface::EffortJointInterface* j =
          dynamic_cast<hardware_interface::EffortJointInterface*>(hw))
  {
    joint_type_ = EFFORT;
  }
  else if (hardware_interface::VelocityJointInterface* j =
               dynamic_cast<hardware_interface::VelocityJointInterface*>(hw))
  {
    joint_type_ = VELOCITY;
  }
  else if (hardware_interface::PositionJointInterface* j =
               dynamic_cast<hardware_interface::PositionJointInterface*>(hw))
  {
    joint_type_ = POSITION;
  }
  else
  {
    throw std::logic_error("UNKNOWN JOINT INTERFACE TYPE");
  }
}

template <class JointInterface>
bool RosControlInterface<JointInterface>::init(JointInterface* hw,
                                               ros::NodeHandle& nh)
{
  if (!controller_)
  {
    ROS_ERROR_STREAM("Failed to initialize controller algorithm! (namespace: "
                     << nh.getNamespace() << ").");
  }

  // joint initialization from
  // https://github.com/ros-controls/ros_controllers/blob/melodic-devel/effort_controllers/src/joint_group_position_controller.cpp

  // List of controlled joints
  std::string param_name = "joints";
  if (!nh.getParam(param_name, joint_names_))
  {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: "
                                            << nh.getNamespace() << ").");
    return false;
  }

  n_joints_ = joint_names_.size();

  if (n_joints_ == 0)
  {
    ROS_ERROR_STREAM("List of joint names is empty.");
    return false;
  }

  // Get URDF
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }

  for (unsigned int i = 0; i < n_joints_; i++)
  {
    const std::string& name = joint_names_[i];

    try
    {
      joints_.push_back(hw->getHandle(name));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception: " << e.what());
    }

    urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(name);
    if (!joint_urdf)
    {
      ROS_ERROR("Could not find joint '%s' in urdf", name.c_str());
      return false;
    }
    joint_urdfs_.push_back(joint_urdf);
  }

  controller_->resetInternalState();
}

template <class JointInterface>
void RosControlInterface<JointInterface>::starting(const ros::Time& time)
{
  controller_->resetInternalState();
}

template <class JointInterface>
void RosControlInterface<JointInterface>::update(const ros::Time& time,
                                                 const ros::Duration& period)
{
  sensor_msgs::JointState curr_state, command;

  for (unsigned int i = 0; i < n_joints_; i++)
  {
    curr_state.name.push_back(joint_names_[i]);
    curr_state.position.push_back(joints_[i].getPosition());
    curr_state.velocity.push_back(joints_[i].getVelocity());
    curr_state.effort.push_back(joints_[i].getEffort());
  }

  command = controller_->updateControl(curr_state, period);

  for (unsigned int i = 0; i < n_joints_; i++)
  {
    const std::string& name = joint_names_[i];

    for (unsigned int j = 0; j < n_joints_; j++)
    {
      if (command.name[j] == name)
      {
        switch (joint_type_)
        {
          case EFFORT:
            joints_[i].setCommand(command.effort[j]);
            break;
          case VELOCITY:
            joints_[i].setCommand(command.velocity[j]);
            break;
          case POSITION:
            joints_[i].setCommand(command.position[j]);
            break;
        }
      }
    }
  }
}

template <class JointInterface>
void RosControlInterface<JointInterface>::stopping(const ros::Time& time)
{
  controller_->resetInternalState();
}
}  // namespace generic_control_toolbox
#endif
