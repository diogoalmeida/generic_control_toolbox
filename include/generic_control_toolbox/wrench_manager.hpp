#ifndef __WRENCH_MANAGER__
#define __WRENCH_MANAGER__

#include <ros/ros.h>
#include <Eigen/Dense>
#include <generic_control_toolbox/manager_base.hpp>
#include <tf/transform_listener.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_kdl.h>

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
      Adds a new wrench subscription. The wrench manager stores the rigid transform between the
      F/T sensor and the gripping point.

      @param end_effector The sensor arm's end-effector.
      @param sensor_frame The TF frame name that represents the sensor pose.
      @param gripping_point_frame The TF frame that represents the gripping point pose.
      @param sensor_topic The wrench topic name for the desired sensor.
      @return False if something goes wrong, true otherwise.
    **/
    bool initializeWrenchComm(const std::string &end_effector, const std::string &sensor_frame, const std::string &gripping_point_frame, const std::string &sensor_topic);

    /**
      Provides access to the measured wrench at the arm's gripping point.

      @param end_effector The arm's end-effector name.
      @wrench The wrench at the gripping point.
      @return false in case of error, true otherwise.
    **/
    bool wrenchAtGrippingPoint(const std::string &end_effector, Eigen::Matrix<double, 6, 1> &wrench) const;

  private:
    int max_tf_attempts_;
    std::vector<std::string> sensor_frame_;
    std::vector<KDL::Frame> sensor_to_gripping_point_;
    std::vector<KDL::Wrench > measured_wrench_;
    std::vector<ros::Subscriber> ft_sub_;
    std::vector<ros::Publisher> processed_ft_pub_;
    std::vector<std::string> gripping_frame_;
    tf::TransformListener listener_;
    ros::NodeHandle nh_;

    /**
      Obtains wrench measurements for a force torque sensor.

      @param msg The force torque message from the sensor node.
    **/
    void forceTorqueCB(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  };
}
#endif
