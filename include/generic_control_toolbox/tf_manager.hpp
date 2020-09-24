#ifndef __TF_MANAGER__
#define __TF_MANAGER__

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <generic_control_toolbox/manager_base.hpp>

namespace generic_control_toolbox
{
/**
  Provides helper methods to handle common TF queries.
**/
class TFManager : public ManagerBase
{
 public:
  TFManager(ros::NodeHandle nh = ros::NodeHandle("~"));
  ~TFManager() {}

  /**
   *  Convert a pose to another frame using the TF tree. Handles TF
   *exceptions.
   *
   * @param frame The target frame to which to transform the given pose.
   * @param pose The given pose.
   *queried, after which a failure is assumed.
   * @param out The given pose in the given frame.
   * @returns False is the listener fails to produce an answer
   **/
  bool getPoseInFrame(const std::string &frame,
                      const geometry_msgs::PoseStamped &pose,
                      geometry_msgs::PoseStamped &out) const;

 private:
  bool getParam();

  ros::NodeHandle nh_;
  double max_wait_time_;
  tf2_ros::TransformListener listener_;
  tf2_ros::Buffer buffer_;
};
}  // namespace generic_control_toolbox
#endif
