#include <generic_control_toolbox/tf_manager.hpp>

namespace generic_control_toolbox
{
TFManager::TFManager(ros::NodeHandle nh) : nh_(nh), listener_(buffer_)
{
  if (!getParam())
  {
    throw std::runtime_error("ERROR getting TFManager's params");
  }
  buffer_.setUsingDedicatedThread(true);
}

bool TFManager::getPoseInFrame(const std::string &frame,
                               const geometry_msgs::PoseStamped &pose,
                               geometry_msgs::PoseStamped &out) const
{
  int attempts = 0;
  std::string error;
  bool success = false;

  if (!buffer_.canTransform(frame, pose.header.frame_id, ros::Time(0),
                            ros::Duration(max_wait_time_), &error))
  {
    ROS_ERROR_STREAM("TFManager: Failed to get the target pose from frame "
                     << pose.header.frame_id << " in frame " << frame
                     << ". Exception: " << error);
    return false;
  }

  out = buffer_.transform<geometry_msgs::PoseStamped>(pose, frame);

  return true;
}  // namespace generic_control_toolbox

bool TFManager::getParam()
{
  if (!nh_.getParam("tf_manager/max_wait_time", max_wait_time_))
  {
    ROS_WARN("TFManager: Missing max_wait_time parameter, setting default");
    max_wait_time_ = 5.0;
  }

  return true;
}
}  // namespace generic_control_toolbox