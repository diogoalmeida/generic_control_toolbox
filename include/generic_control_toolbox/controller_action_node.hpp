#ifndef __CONTROLLER_ACTION_NODE__
#define __CONTROLLER_ACTION_NODE__

#include <sensor_msgs/JointState.h>
#include <generic_control_toolbox/controller_template.hpp>
#include <stdexcept>

namespace generic_control_toolbox
{
/**
  Maintains a generic action node which integrates the controller
  template in a stand-alone ROS node, for systems where using ROS control
  is not an option.
**/
class ControllerActionNode
{
 public:
  ControllerActionNode();
  ~ControllerActionNode();

  /**
    This blocking method will run the controller by providing
    it with the currently available joint states and time intervals
    between calls. It will publish the controller output to the robot
    command topic.

    @param controller Any controller which complies with ControllerBase.
  **/
  void runController(ControllerBase &controller);

 private:
  void jointStatesCb(const sensor_msgs::JointState::ConstPtr &msg);

  ros::NodeHandle nh_;
  sensor_msgs::JointState state_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher state_pub_;
  bool got_first_;
  double loop_rate_;
};
}  // namespace generic_control_toolbox
#endif
