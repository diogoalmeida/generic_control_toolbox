#ifndef __WRENCH_MANAGER__
#define __WRENCH_MANAGER__

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/WrenchStamped.h>
#include <kdl_conversions/kdl_msg.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <generic_control_toolbox/manager_base.hpp>
#include <generic_control_toolbox/matrix_parser.hpp>

namespace generic_control_toolbox
{
/**
  Subscribes to wrench topics and produces the wrench value at a requested
  frame.
**/
class WrenchManager : public ManagerBase
{
 public:
  WrenchManager();
  ~WrenchManager();

  /**
    Adds a new wrench subscription. The wrench manager stores the rigid
  transform between the F/T sensor and the gripping point.

    @param end_effector The sensor arm's end-effector.
    @param sensor_frame The TF frame name that represents the sensor pose.
    @param gripping_point_frame The TF frame that represents the gripping point
  pose.
    @param sensor_topic The wrench topic name for the desired sensor.
    @return False if something goes wrong, true otherwise.
  **/
  bool initializeWrenchComm(const std::string &end_effector,
                            const std::string &sensor_frame,
                            const std::string &gripping_point_frame,
                            const std::string &sensor_topic,
                            const std::string &calib_matrix_param);

  /**
    Provides access to the measured wrench at the arm's gripping point.
    Wrench is expressed in the point's frame.

    @param end_effector The arm's end-effector name.
    @param wrench The wrench at the gripping point.
    @return false in case of error, true otherwise.
  **/
  bool wrenchAtGrippingPoint(const std::string &end_effector,
                             Eigen::Matrix<double, 6, 1> &wrench) const;

  /**
    Gets the original sensor measurements, at the sensor frame.

    @param end_effector The arm's end-effector name.
    @param wrench The wrench at the sensor point.
    @return false in case of error, true otherwise.
  **/
  bool wrenchAtSensorPoint(const std::string &end_effector,
                           Eigen::Matrix<double, 6, 1> &wrench) const;

 private:
  int max_tf_attempts_;
  std::map<std::string, std::string> sensor_frame_;
  std::map<std::string, KDL::Frame> sensor_to_gripping_point_;
  std::map<std::string, KDL::Wrench> measured_wrench_;
  std::map<std::string, ros::Subscriber> ft_sub_;
  std::map<std::string, ros::Publisher> processed_ft_pub_;
  std::map<std::string, std::string> gripping_frame_;
  std::map<std::string, Eigen::Matrix<double, 6, 6> > calibration_matrix_;
  tf::TransformListener listener_;
  MatrixParser parser_;
  ros::NodeHandle nh_;

  /**
    Obtains wrench measurements for a force torque sensor.

    @param msg The force torque message from the sensor node.
  **/
  void forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg);
};

/**
  Initializes a wrench manager class with the given arm's sensor info.
  Uses the ros parameter server to obtain information."

  @param arm_info The arm information
  @param manager Reference to the wrench manager.
  @return False if something goes wrong, true otherwise.
**/
bool setWrenchManager(const ArmInfo &arm_info, WrenchManager &manager);
}  // namespace generic_control_toolbox
#endif
