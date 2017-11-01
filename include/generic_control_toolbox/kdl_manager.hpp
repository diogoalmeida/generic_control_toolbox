#ifndef __KDL_MANAGER__
#define __KDL_MANAGER__

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <urdf/model.h>
#include <kdl/frames.hpp>
#include <stdexcept>
#include <generic_control_toolbox/manager_base.hpp>

namespace generic_control_toolbox
{
  /**
    Loads and maintains a URDF robot description and provides access to relevant
    KDL objects/solvers. Supports n end-effectors
  **/
  class KDLManager : public ManagerBase
  {
  public:
    KDLManager(const std::string &chain_base_link);
    ~KDLManager();

    /**
      Initialize the kinematic chain, solvers and joint arrays for an arm defined by its end-effector link.
      The kinematic chain is assumed to start at chain_base_link_.

      @param end_effector_link The final link of the kinematic chain.
      @return false if it is not possible to initialize.
    **/
    bool initializeArm(const std::string &end_effector_link);

    /**
      Returns the pose of the requested end-effector, given a joint state.

      @param end_effector_link The name of the requested end-effector.
      @param state The current robot joint state.
      @param out The resulting eef pose.
      @return False in case something goes wrong, true otherwise.
    **/
    bool getEefPose(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Frame &out);

    /**
      Returns the twist of the requested end-effector, given a joint state.

      @param end_effector_link The name of the requested end-effector.
      @param state The current robot joint state.
      @param out The resulting eef twist.
      @return False in case something goes wrong, true otherwise.
    **/
    bool getEefTwist(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::FrameVel &out);

    /**
      Returns the inverse kinematics of the requested end-effector, given a desired pose.

      @param end_effector_link The name of the requested end-effector.
      @param state The current robot joint state.
      @param in Desired eef pose.
      @param out Joint state for the desired pose.
      @return False in case something goes wrong, true otherwise.
    **/
    bool getPoseIK(const std::string &end_effector_link, const sensor_msgs::JointState &state, const KDL::Frame &in, KDL::JntArray &out);

    /**
      Returns the inverse differential kinematics of the requested end-effector, given a desired twist.

      @param end_effector_link The name of the requested end-effector.
      @param state The current robot joint state.
      @param in Desired eef twist.
      @param out Joint state for the desired twist.
      @return False in case something goes wrong, true otherwise.
    **/
    bool getVelIK(const std::string &end_effector_link, const sensor_msgs::JointState &state, const KDL::Twist &in, KDL::JntArray &out);

    /**
      Returns the jacobian the requested end-effector's chain.

      @param end_effector_link The name of the requested end-effector.
      @param state The current robot joint state.
      @param out End-effector's jacobian.
      @return False in case something goes wrong, true otherwise.
    **/
    bool getJacobian(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Jacobian &out);

  private:
    std::vector<std::shared_ptr<KDL::ChainIkSolverVel_wdls> > ikvel_;
    std::vector<std::shared_ptr<KDL::ChainIkSolverPos_LMA> > ikpos_;
    std::vector<std::shared_ptr<KDL::ChainFkSolverPos_recursive> > fkpos_;
    std::vector<std::shared_ptr<KDL::ChainFkSolverVel_recursive> > fkvel_;
    std::vector<std::shared_ptr<KDL::ChainJntToJacSolver> > jac_solver_;
    std::vector<KDL::Chain> chain_;

    urdf::Model model_;
    std::vector<std::vector<std::string> > actuated_joint_names_; // list of actuated joints per arm
    std::string chain_base_link_;
    double eps_;

    /**
      Fills in the joint arrays with the state of a given kinematic chain.
      The joint state might include joints outside of the kinematic chain, so there is
      the need to process it.

      @param current_state The robot joint state.
      @param chain The target kinematic chain.
      @param positions The joint positions of the kinematic chain.
      @param velocities The joint velocities of the kinematic chain.
      @return True if the full joint chain was found in the current state, false otherwise.
    **/
    bool getChainJointState(const sensor_msgs::JointState &current_state, const KDL::Chain &chain, KDL::JntArray &positions, KDL::JntArrayVel &velocities);

    /**
      Check if a chain has the given joint_name.

      @param chain The kinematic chain where to look for the joint.
      @param joint_name The joint name we wish to check.
      @return True if the joint was found in the kinematic chain, false otherwise.
    **/
    bool hasJoint(const KDL::Chain &chain, const std::string &joint_name);
  };
}
#endif
