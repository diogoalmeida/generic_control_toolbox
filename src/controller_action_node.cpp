#include <generic_control_toolbox/controller_action_node.hpp>

namespace generic_control_toolbox
{
ControllerActionNode::ControllerActionNode()
{
  nh_ = ros::NodeHandle("~");

  if (!nh_.getParam("loop_rate", loop_rate_))
  {
    ROS_WARN_STREAM("Missing loop_rate parameter for "
                    << ros::this_node::getName() << ". Using default.");
    loop_rate_ = 100;
  }

  got_first_ = false;
  joint_state_sub_ = nh_.subscribe("/joint_states", 1,
                                   &ControllerActionNode::jointStatesCb, this);
  state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_command", 1);
}

ControllerActionNode::~ControllerActionNode() {}

void ControllerActionNode::runController(ControllerBase &controller)
{
  ros::Rate r(loop_rate_);
  ros::Time prev_time = ros::Time::now();
  sensor_msgs::JointState command;
  bool was_running = false;

  while (ros::ok())
  {
    if (got_first_)  // TODO: Maybe keep track of how long was it since the last
                     // joint states msg received?
    {
      command = controller.updateControl(state_, ros::Time::now() - prev_time);
      if (controller.isActive())
      {
        ROS_DEBUG_THROTTLE(10, "Controller is active, publishing");
        was_running = true;
        state_pub_.publish(command);
      }
      else
      {
        if (was_running)
        {
          state_pub_.publish(command);  // publish the last command msg
          was_running = false;
        }
        ROS_DEBUG_THROTTLE(10, "Controller is not active, skipping");
      }
    }
    else
    {
      ROS_WARN_THROTTLE(10, "No joint state received");
    }

    prev_time = ros::Time::now();
    ros::spinOnce();
    r.sleep();
  }
}

void ControllerActionNode::jointStatesCb(
    const sensor_msgs::JointState::ConstPtr &msg)
{
  ROS_INFO_ONCE("Joint state received!");
  state_ = *msg;
  got_first_ = true;
}
}  // namespace generic_control_toolbox
