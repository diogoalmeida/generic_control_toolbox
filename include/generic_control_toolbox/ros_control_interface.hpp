#ifndef __ROS_CONTROL_INTERFACE__
#define __ROS_CONTROL_INTERFACE__

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
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
  RosControlInterface(ControllerBase& controller);

  bool init(JointInterface* hw, ros::NodeHandle& nh);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);

 private:
  ros::NodeHandle& nh_;
  ControllerBase controller_;
  JointType joint_type_;
};

template <class JointInterface>
RosControlInterface<JointInterface>::RosControlInterface(
    ControllerBase& controller)
    : controller_(controller)
{
  nh_ = ros::NodeHandle("~");

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
}

template <class JointInterface>
void RosControlInterface<JointInterface>::starting(const ros::Time& time)
{
}

template <class JointInterface>
void RosControlInterface<JointInterface>::update(const ros::Time& time,
                                                 const ros::Duration& period)
{
}

template <class JointInterface>
void RosControlInterface<JointInterface>::stopping(const ros::Time& time)
{
}

}  // namespace generic_control_toolbox

#endif
