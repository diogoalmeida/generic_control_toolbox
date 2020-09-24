#include <generic_control_toolbox/tf_manager.hpp>

namespace generic_control_toolbox
{
TFManager::TFManager(ros::NodeHandle nh) : nh_(nh)
{
  if (!getParam())
  {
    throw std::runtime_error("ERROR getting TFManager's params");
  }
}

bool TFManager::getPoseInFrame(const std::string &frame,
                               const geometry_msgs::PoseStamped &pose,
                               geometry_msgs::PoseStamped &out)
{
  int attempts = 0;
  bool success = false;
  std::string error;
  while (attempts < max_tf_attempts_)
  {
    try
    {
      listener_.transformPose(frame, pose, out);
      success = true;
      break;
    }
    catch (tf::TransformException ex)
    {
      attempts++;
      ros::Duration(0.1).sleep();
      error = ex.what();
    }
  }

  if (!success)
  {
    ROS_ERROR_STREAM("TFManager: Failed to get the target pose from frame "
                     << pose.header.frame_id << " in frame " << frame
                     << ". Exception: " << error);
  }

  return success;
}

bool TFManager::getParam()
{
  if (!nh_.getParam("tf_manager/max_attempts", max_tf_attempts_))
  {
    ROS_WARN("TFManager: Missing max_attempts parameter, setting default");
    max_tf_attempts_ = 5;
  }

  return true;
}
}  // namespace generic_control_toolbox