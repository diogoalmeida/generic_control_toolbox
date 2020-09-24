#include <generic_control_toolbox/wrench_manager.hpp>

namespace generic_control_toolbox
{
WrenchManager::WrenchManager(ros::NodeHandle nh) : nh_(nh), tf_manager_(nh) {}

WrenchManager::~WrenchManager() {}

bool WrenchManager::initializeWrenchComm(
    const std::string &end_effector, const std::string &sensor_frame,
    const std::string &gripping_point_frame, const std::string &sensor_topic,
    const std::string &calib_matrix_param)
{
  if (sensor_frame_.find(end_effector) != sensor_frame_.end())
  {
    ROS_ERROR(
        "Cannot initialize wrench subscriber for end-effector %s: already "
        "initialized",
        end_effector.c_str());
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

  if (!tf_manager_.getPoseInFrame(gripping_point_frame,
                                  sensor_to_gripping_point,
                                  sensor_to_gripping_point))
  {
    return false;
  }

  Eigen::MatrixXd C;
  if (!parser_.parseMatrixData(C, calib_matrix_param, nh_))
  {
    ROS_WARN(
        "WrenchManager: missing force torque sensor calibration matrix "
        "parameter %s. Setting default.",
        calib_matrix_param.c_str());
    C = Eigen::Matrix<double, 6, 6>::Identity();
  }
  else
  {
    if (C.cols() != 6 || C.rows() != 6)
    {
      ROS_ERROR("WrenchManager: calibration matrix must be 6x6. Got %ldx%ld",
                C.rows(), C.cols());
      return false;
    }
  }

  // Everything is ok, can add new comm.
  KDL::Frame sensor_to_gripping_point_kdl;
  calibration_matrix_[end_effector] = C;
  tf::poseMsgToKDL(sensor_to_gripping_point.pose, sensor_to_gripping_point_kdl);
  sensor_frame_[end_effector] = sensor_frame;
  sensor_to_gripping_point_[end_effector] = sensor_to_gripping_point_kdl;
  measured_wrench_[end_effector] = KDL::Wrench::Zero();
  ft_sub_[end_effector] =
      nh_.subscribe(sensor_topic, 1, &WrenchManager::forceTorqueCB, this);
  gripping_frame_[end_effector] = gripping_point_frame;
  processed_ft_pub_[end_effector] = nh_.advertise<geometry_msgs::WrenchStamped>(
      sensor_topic + "_converted", 1);

  return true;
}

bool WrenchManager::wrenchAtGrippingPoint(
    const std::string &end_effector, Eigen::Matrix<double, 6, 1> &wrench) const
{
  if (sensor_frame_.find(end_effector) == sensor_frame_.end())
  {
    return false;
  }

  KDL::Wrench wrench_kdl;
  geometry_msgs::WrenchStamped temp_wrench;
  wrench_kdl = sensor_to_gripping_point_.at(end_effector) *
               measured_wrench_.at(end_effector);
  tf::wrenchKDLToEigen(wrench_kdl, wrench);

  // publish processed wrench to facilitate debugging
  tf::wrenchKDLToMsg(wrench_kdl, temp_wrench.wrench);
  temp_wrench.header.frame_id = gripping_frame_.at(end_effector);
  temp_wrench.header.stamp = ros::Time::now();
  processed_ft_pub_.at(end_effector).publish(temp_wrench);

  return true;
}

bool WrenchManager::wrenchAtSensorPoint(
    const std::string &end_effector, Eigen::Matrix<double, 6, 1> &wrench) const
{
  if (sensor_frame_.find(end_effector) == sensor_frame_.end())
  {
    return false;
  }

  tf::wrenchKDLToEigen(measured_wrench_.at(end_effector), wrench);

  return true;
}

void WrenchManager::forceTorqueCB(
    const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  std::string eef;
  for (auto const &it : sensor_frame_)
  {
    if (msg->header.frame_id == it.second)
    {
      eef = it.first;
      break;
    }
  }

  if (eef.empty())
  {
    ROS_ERROR(
        "WrenchManager: got wrench message from sensor at frame %s, which "
        "was "
        "not configured in the wrench manager",
        msg->header.frame_id.c_str());
    return;
  }

  // apply computed sensor intrinsic calibration
  Eigen::Matrix<double, 6, 1> wrench_eig;
  tf::wrenchMsgToEigen(msg->wrench, wrench_eig);
  wrench_eig = calibration_matrix_.at(eef) * wrench_eig;
  tf::wrenchEigenToKDL(wrench_eig, measured_wrench_.at(eef));
}

bool setWrenchManager(const ArmInfo &arm_info, WrenchManager &manager)
{
  if (arm_info.has_ft_sensor)
  {
    if (!manager.initializeWrenchComm(
            arm_info.kdl_eef_frame, arm_info.sensor_frame,
            arm_info.gripping_frame, arm_info.sensor_topic,
            arm_info.name + "/sensor_calib"))
    {
      return false;
    }

    ROS_DEBUG("WrenchManager: successfully initialized wrench comms for arm %s",
              arm_info.name.c_str());
  }
  else
  {
    ROS_WARN("WrenchManager: end-effector %s has no F/T sensor.",
             arm_info.kdl_eef_frame.c_str());
  }

  return true;
}

bool setWrenchManager(const ArmInfo &arm_info,
                      std::shared_ptr<WrenchManager> &manager)
{
  return setWrenchManager(arm_info, *manager);
}
}  // namespace generic_control_toolbox
