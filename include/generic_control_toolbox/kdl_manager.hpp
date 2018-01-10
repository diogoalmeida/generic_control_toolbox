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
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <urdf/model.h>
#include <kdl/frames.hpp>
#include <stdexcept>
#include <generic_control_toolbox/manager_base.hpp>
#include <generic_control_toolbox/ArmInfo.h>

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
      Queries TF for the rigid transform between the end-effector link frame and
      the gripping point frame and stores it as a KDL Frame.

      @param end_effector_link The end-effector link name.
      @param gripping_point_frame TF name of the gripping point frame.
      @return False if transform is not found or something else goes wrong, true otherwise.
    **/
    bool setGrippingPoint(const std::string &end_effector_link, const std::string &gripping_point_frame);

    /**
      Queries TF for the rigid transform between the end-effector link frame and
      the gripping point frame, and stores it as a KDL Frame.

      @param end_effector_link The end-effector link name.
      @param sensor_point_frame TF name of the sensor point frame.
      @return False if transform is not found or something else goes wrong, true otherwise.
    **/
    bool setSensorPoint(const std::string &end_effector_link, const std::string &sensor_point_frame);

    /**
      Fills the joint state message given only joint velocities. Joint positions are
      taken from the joint state message.

      @param end_effector_link The chain's end_effector.
      @param q The chain's joint posisitions.
      @param qdot The chain's joint velocities.
      @param state The generated joint state message.
      @return False if something goes wrong.
    **/
    bool getJointState(const std::string &end_effector_link, const Eigen::VectorXd &qdot, sensor_msgs::JointState &state) const;

    /**
      Fills the joint state message appropriately given the joint
      positions and velocities of the selected end-effector's joint chain.

      @param end_effector_link The chain's end_effector.
      @param q The chain's joint posisitions.
      @param qdot The chain's joint velocities.
      @param state The generated joint state message.
      @return False if something goes wrong.
    **/
    bool getJointState(const std::string &end_effector_link, const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, sensor_msgs::JointState &state) const;

    /**
      Returns the gripping point of the chosen arm.
      By default this is set to be the end-effector pose.

      @param end_effector_link The arm's end-effector name.
      @param state The robot joint state.
      @param out The resulting gripping point frame.
      @return False if something goes wrong, true otherwise.
    **/
    bool getGrippingPoint(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Frame &out) const;

    /**
      Returns the sensor point of the chosen arm.
      By default this is set to be the end-effector pose.

      @param end_effector_link The arm's end-effector name.
      @param state The robot joint state.
      @param out The resulting sensor point frame.
      @return False if something goes wrong, true otherwise.
    **/
    bool getSensorPoint(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Frame &out) const;

    /**
      Returns the twist of the requested end-effector's gripping point.
      By default this is set to be the end-effector's twist.

      @param end_effector_link The name of the requested end-effector.
      @param state The current robot joint state.
      @param out The resulting eef twist.
      @return False in case something goes wrong, true otherwise.
    **/
    bool getGrippingTwist(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Twist &out) const;

    /**
      Returns the inverse kinematics of the requested end-effector, given a desired pose.

      @param end_effector_link The name of the requested end-effector.
      @param state The current robot joint state.
      @param in Desired eef pose.
      @param out Joint state for the desired pose.
      @return False in case something goes wrong, true otherwise.
    **/
    bool getPoseIK(const std::string &end_effector_link, const sensor_msgs::JointState &state, const KDL::Frame &in, KDL::JntArray &out) const;

    /**
      Returns the inverse differential kinematics of the requested end-effector, given a desired twist.

      @param end_effector_link The name of the requested end-effector.
      @param state The current robot joint state.
      @param in Desired eef twist.
      @param out Joint state for the desired twist.
      @return False in case something goes wrong, true otherwise.
    **/
    bool getVelIK(const std::string &end_effector_link, const sensor_msgs::JointState &state, const KDL::Twist &in, KDL::JntArray &out) const;

    /**
      Returns the jacobian the requested end-effector's chain.

      @param end_effector_link The name of the requested end-effector.
      @param state The current robot joint state.
      @param out End-effector's jacobian.
      @return False in case something goes wrong, true otherwise.
    **/
    bool getJacobian(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Jacobian &out) const;

    /**
    Returns the pose of the requested end-effector, given a joint state.

    @param end_effector_link The name of the requested end-effector.
    @param state The current robot joint state.
    @param out The resulting eef pose.
    @return False in case something goes wrong, true otherwise.
    **/
    bool getEefPose(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Frame &out) const;

    /**
    Returns the twist of the requested end-effector, given a joint state.

    @param end_effector_link The name of the requested end-effector.
    @param state The current robot joint state.
    @param out The resulting eef twist.
    @return False in case something goes wrong, true otherwise.
    **/
    bool getEefTwist(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::FrameVel &out) const;

  private:
    std::vector<std::shared_ptr<KDL::ChainIkSolverVel_wdls> > ikvel_;
    std::vector<std::shared_ptr<KDL::ChainIkSolverPos_LMA> > ikpos_;
    std::vector<std::shared_ptr<KDL::ChainFkSolverPos_recursive> > fkpos_;
    std::vector<std::shared_ptr<KDL::ChainFkSolverVel_recursive> > fkvel_;
    std::vector<std::shared_ptr<KDL::ChainJntToJacSolver> > jac_solver_;
    std::vector<KDL::Frame> eef_to_gripping_point_;
    std::vector<KDL::Frame> eef_to_sensor_point_;
    std::vector<KDL::Chain> chain_;

    urdf::Model model_;
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    std::vector<std::vector<std::string> > actuated_joint_names_; // list of actuated joints per arm
    std::string chain_base_link_;
    double eps_, max_tf_attempts_;


    /**
      Queries TF to get the rigid transform between two frames

      @param base_frame The base frame of the transform.
      @param target_frame The target frame of the transform.
      @param out The rigid transform between the two frames in the KDL format.
      @return False in case something goes wrong, true otherwise.
    **/
    bool getRigidTransform(const std::string &base_frame, const std::string &target_frame, KDL::Frame &out) const;

    /**
      Fills in the joint arrays with the state of a given kinematic chain.
      The joint state might include joints outside of the kinematic chain, so there is
      the need to process it.

      @param current_state The robot joint state.
      @param arm The target arm index.
      @param positions The joint positions of the kinematic chain.
      @param velocities The joint velocities of the kinematic chain.
      @return True if the full joint chain was found in the current state, false otherwise.
    **/
    bool getChainJointState(const sensor_msgs::JointState &current_state, int arm, KDL::JntArray &positions, KDL::JntArrayVel &velocities) const;

    /**
      Check if a chain has the given joint_name.

      @param chain The kinematic chain where to look for the joint.
      @param joint_name The joint name we wish to check.
      @return True if the joint was found in the kinematic chain, false otherwise.
    **/
    bool hasJoint(const KDL::Chain &chain, const std::string &joint_name) const;
  };

  /**
    Initializes a kdl manager class with the given arm info.
    Uses the ros parameter server to obtain information.

    @param arm_info The arm information.
    @param manager Reference to the kdl manager.
    @return False if something goes wrong, true otherwise.
  **/
  bool setKDLManager(const ArmInfo &arm_info, std::shared_ptr<KDLManager> manager);

}
#endif
