#include <generic_control_toolbox/wrench_manager.hpp>

namespace generic_control_toolbox
{
  WrenchManager::WrenchManager()
  {
    nh_ = ros::NodeHandle("~");
    // TODO: Make parameter
    max_tf_attempts_ = 5;
  }

  WrenchManager::~WrenchManager(){}

  bool WrenchManager::initializeWrenchComm(const std::string &end_effector, const std::string &sensor_frame, const std::string &gripping_point_frame, const std::string &sensor_topic)
  {
    int a;
    if (getIndex(end_effector, a))
    {
      ROS_ERROR("Cannot initialize wrench subscriber for end-effector %s: already initialized", end_effector.c_str());
      return false;
    }

    // get rigid transform between sensor frame and arm gripping point
    geometry_msgs::PoseStamped sensor_to_gripping_point;
    sensor_to_gripping_point.header.frame_id = sensor_frame;
    sensor_to_gripping_point.header.stamp = ros::Time(0);
    sensor_to_gripping_point.pose.position.x = 0;
    sensor_to_gripping_point.pose.position.y = 0;
    sensor_to_gripping_point.pose.position.z = 0;
    sensor_to_gripping_point.pose.orientation.x = 0;
    sensor_to_gripping_point.pose.orientation.y = 0;
    sensor_to_gripping_point.pose.orientation.z = 0;
    sensor_to_gripping_point.pose.orientation.w = 1;

    int attempts;
    for (attempts = 0; attempts < max_tf_attempts_; attempts++)
    {
      try
      {
        listener_.transformPose(gripping_point_frame, sensor_to_gripping_point, sensor_to_gripping_point);
        break;
      }
      catch (tf::TransformException ex)
      {
        ROS_WARN("TF exception in wrench manager: %s", ex.what());
      }
      sleep(0.1);
    }

    if (attempts >= max_tf_attempts_)
    {
      ROS_ERROR("Wrench manager could not find the transform between the sensor frame %s and gripping point %s", sensor_frame.c_str(), gripping_point_frame.c_str());
      return false;
    }

    // Everything is ok, can add new comm.
    KDL::Frame sensor_to_gripping_point_kdl;
    tf::poseMsgToKDL(sensor_to_gripping_point.pose, sensor_to_gripping_point_kdl);
    manager_index_.push_back(end_effector);
    sensor_frame_.push_back(sensor_frame);
    sensor_to_gripping_point_.push_back(sensor_to_gripping_point_kdl);
    measured_wrench_.push_back(KDL::Wrench::Zero());
    ft_sub_.push_back(nh_.subscribe(sensor_topic, 1, &WrenchManager::forceTorqueCB, this));
    return true;
  }

  bool WrenchManager::wrenchAtGrippingPoint(const std::string &end_effector, Eigen::Matrix<double, 6, 1> &wrench) const
  {
    int arm;
    if (!getIndex(end_effector, arm))
    {
      return false;
    }

    KDL::Wrench wrench_kdl;
    wrench_kdl = sensor_to_gripping_point_[arm]*measured_wrench_[arm];
    tf::wrenchKDLToEigen(wrench_kdl, wrench);
    return true;
  }

  void WrenchManager::forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg)
  {
    int sensor_num = -1;
    for (int i = 0; i < sensor_frame_.size(); i++)
    {
      if (msg->header.frame_id == sensor_frame_[i])
      {
        sensor_num = i;
        break;
      }
    }

    if (sensor_num < 0)
    {
      ROS_ERROR("Got wrench message from sensor at frame %s, which was not configured in the wrench manager", msg->header.frame_id.c_str());
      return;
    }

    tf::wrenchMsgToKDL(msg->wrench, measured_wrench_[sensor_num]);
  }
}
