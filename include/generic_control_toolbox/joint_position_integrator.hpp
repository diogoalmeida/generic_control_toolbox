#ifndef __VELOCITY_TO_POSITION__
#define __VELOCITY_TO_POSITION__

#include <ros/ros.h>
#include <kdl/jntarray.hpp>

namespace generic_control_toolbox
{
/**
 *  Implement a joint position integrator which produces joint position
 *references given a joint velocity target.
 **/
class JointPositionIntegrator
{
 public:
  JointPositionIntegrator(ros::NodeHandle &nh);
  ~JointPositionIntegrator() {}

  /**
   *  Produce an updated joint position vector for a given set of joint
   *velocities.
   *
   *  The output is q = sat(q + q_dot * dt), where the function sat(.) saturates
   *the output if any joint exceeds the configured max_joint_error.
   *
   * @param q_dot The array of joint velocities to be integrated.
   * @param q The current joint state.
   * @param dt The integration delta.
   * @returns An updated set of joint positions.
   **/
  KDL::JntArray update(const KDL::JntArray &q_dot, const KDL::JntArray &q,
                       float dt);

  /**
   * Reset the integrator's state.
   **/
  void reset();

 private:
  ros::NodeHandle nh_;
  KDL::JntArray virt_q_;
  float max_joint_error_;
  bool got_state_;

  /**
   * Get the parameters for the integrator.
   **/
  bool init();
};
}  // namespace generic_control_toolbox
#endif