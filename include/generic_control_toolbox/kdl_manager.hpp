#ifndef __KDL_MANAGER__
#define __KDL_MANAGER__

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <generic_control_toolbox/ArmInfo.h>
#include <geometry_msgs/WrenchStamped.h>
#include <kdl_conversions/kdl_msg.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <generic_control_toolbox/manager_base.hpp>
#include <generic_control_toolbox/matrix_parser.hpp>
#include <generic_control_toolbox/tf_manager.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/kdl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <stdexcept>

const std::string WDLS_SOLVER("wdls"), NSO_SOLVER("nso");

namespace generic_control_toolbox
{
/**
  Loads and maintains a URDF robot description and provides access to relevant
  KDL objects/solvers. Supports n end-effectors
**/
class KDLManager : public ManagerBase
{
 public:
  KDLManager(const std::string &chain_base_link,
             ros::NodeHandle nh = ros::NodeHandle("~"));
  ~KDLManager();

  /**
    Initialize the kinematic chain, solvers and joint arrays for an arm defined
  by its end-effector link. The kinematic chain is assumed to start at
  chain_base_link_.

    @param end_effector_link The final link of the kinematic chain.
    @return false if it is not possible to initialize.
  **/
  bool initializeArm(const std::string &end_effector_link);

  /**
    Checks whether a kinematic chain with the given end-effector link was
  initialized in a KDL manager instance.

    @param end_effector_link The final link of the kinematic chain.
    @return False if the chain is not initialized, true otherwise.
  **/
  bool isInitialized(const std::string &end_effector_link) const;

  /**
    Checks if the provided joint state message has the required information for
  the kinematic chain indexed by the given end effector link. Useful with the
  joint states topic does not contain consistently the same information.

    @param end_effector_link The end_effector link name.
    @param state The joint state message to verify.
    @returns True if the joint state message contains valid information for the
  requested chain, false otherwise.
  **/
  bool checkStateMessage(const std::string &end_effector_link,
                         const sensor_msgs::JointState &state) const;
  /**
    Queries TF for the rigid transform between the end-effector link frame and
    the gripping point frame and stores it as a KDL Frame.

    @param end_effector_link The end-effector link name.
    @param gripping_point_frame TF name of the gripping point frame.
    @return False if transform is not found or something else goes wrong, true
  otherwise.
  **/
  bool setGrippingPoint(const std::string &end_effector_link,
                        const std::string &gripping_point_frame);

  /**
    Queries TF for the rigid transform between the end-effector link frame and
    a sensor point frame, and stores it as a KDL Frame.

    @param end_effector_link The end-effector link name.
    @param sensor_point_frame TF name of the sensor point frame.
    @return False if transform is not found or something else goes wrong, true
  otherwise.
  **/
  bool setSensorPoint(const std::string &end_effector_link,
                      const std::string &sensor_point_frame);

  /**
    Create a joint state message for the given end-effector link with the
  desired joint position and velocities.

    @param end_effector_link The chain's end_effector.
    @param qdot The chain's joint velocities.
    @param state The generated joint state message.
    @return False if something goes wrond.
  **/
  bool createJointState(const std::string &end_effector_link,
                        const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                        sensor_msgs::JointState &state) const;
  /**
    Fills the joint state message given only joint velocities. Joint positions
  are taken from the joint state message.

    @param end_effector_link The chain's end_effector.
    @param qdot The chain's joint velocities.
    @param state The generated joint state message.
    @return False if something goes wrong.
  **/
  bool getJointState(const std::string &end_effector_link,
                     const Eigen::VectorXd &qdot,
                     sensor_msgs::JointState &state) const;

  /**
    Fills the joint state message appropriately given the joint
    positions and velocities of the selected end-effector's joint chain.

    @param end_effector_link The chain's end_effector.
    @param q The chain's joint posisitions.
    @param qdot The chain's joint velocities.
    @param state The generated joint state message.
    @return False if something goes wrong.
  **/
  bool getJointState(const std::string &end_effector_link,
                     const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                     sensor_msgs::JointState &state) const;

  /**
    Fills the joint state message appropriately given the joint
    positions, velocities and efforts of the selected end-effector's joint
  chain.

    @param end_effector_link The chain's end_effector.
    @param q The chain's joint posisitions.
    @param qdot The chain's joint velocities.
    @param effort The chain's joint efforts.
    @param state The generated joint state message.
    @return False if something goes wrong.
  **/
  bool getJointState(const std::string &end_effector_link,
                     const Eigen::VectorXd &q, const Eigen::VectorXd &qdot,
                     const Eigen::VectorXd &effort,
                     sensor_msgs::JointState &state) const;

  /**
    Returns the gripping point of the chosen arm.
    By default this is set to be the end-effector pose.

    @param end_effector_link The arm's end-effector name.
    @param state The robot joint state.
    @param out The resulting gripping point frame.
    @return False if something goes wrong, true otherwise.
  **/
  bool getGrippingPoint(const std::string &end_effector_link,
                        const sensor_msgs::JointState &state,
                        KDL::Frame &out) const;

  /**
    Returns the sensor point of the chosen arm.
    By default this is set to be the end-effector pose.

    @param end_effector_link The arm's end-effector name.
    @param state The robot joint state.
    @param out The resulting sensor point frame.
    @return False if something goes wrong, true otherwise.
  **/
  bool getSensorPoint(const std::string &end_effector_link,
                      const sensor_msgs::JointState &state,
                      KDL::Frame &out) const;

  /**
    Returns the twist of the requested end-effector's gripping point.
    By default this is set to be the end-effector's twist.

    @param end_effector_link The name of the requested end-effector.
    @param state The current robot joint state.
    @param out The resulting eef twist.
    @return False in case something goes wrong, true otherwise.
  **/
  bool getGrippingTwist(const std::string &end_effector_link,
                        const sensor_msgs::JointState &state,
                        KDL::Twist &out) const;

  /**
    Verifies if a given pose is reachable by the requested end-effector.

    @param end_effector_link The name of the requested end-effector.
    @param in The pose that is to be verified.
    @returns True if the pose is reachable, false otherwise or in case of error.
  **/
  bool verifyPose(const std::string &end_effector_link,
                  const KDL::Frame &in) const;

  /**
    Returns the inverse kinematics of the requested end-effector, given a
  desired pose.

    @param end_effector_link The name of the requested end-effector.
    @param state The current robot joint state.
    @param in Desired eef pose.
    @param out Joint state for the desired pose.
    @return False in case something goes wrong or if the solver did not converge
  to the desired pose, true otherwise.
  **/
  bool getPoseIK(const std::string &end_effector_link,
                 const sensor_msgs::JointState &state, const KDL::Frame &in,
                 KDL::JntArray &out) const;

  /**
    Returns the inverse kinematics of a chain gripping point , given a desired
  pose.

    @param end_effector_link The name of the requested end-effector.
    @param state The current robot joint state.
    @param in Desired eef pose.
    @param out Joint state for the desired gripping pose.
    @return False in case something goes wrong or if the solver did not converge
  to the desired pose, true otherwise.
  **/
  bool getGrippingPoseIK(const std::string &end_effector_link,
                         const sensor_msgs::JointState &state,
                         const KDL::Frame &in, KDL::JntArray &out) const;

  /**
    Returns the forward kinematics of the requested end-effector, given a
  desired joint state.

    @param end_effector_link The name of the requested end-effector.
    @param state The current robot joint state.
    @param in Desired eef joint state.
    @param out Pose for the desired joint state.
    @return False in case something goes wrong, true otherwise.
  **/
  bool getPoseFK(const std::string &end_effector_link,
                 const sensor_msgs::JointState &state, const KDL::JntArray &in,
                 KDL::Frame &out) const;

  /**
    Returns the inverse differential kinematics of the requested gripping point,
  given a desired twist.

    @param end_effector_link The name of the requested end-effector.
    @param state The current robot joint state.
    @param in Desired gripping point twist, in the gripping point frame.
    @param out Joint state for the desired twist.
    @return False in case something goes wrong, true otherwise.
  **/
  bool getGrippingVelIK(const std::string &end_effector_link,
                        const sensor_msgs::JointState &state,
                        const KDL::Twist &in, KDL::JntArray &out) const;

  /**
   *  Convert an input twist expressed in the chain's gripping frame to an
   *equivalent twist at the end-effector. Assumes  rigid linkage between the two
   *frames.
   *
   * @param end_effector_link The name of the requested end-effector.
   * @param state The current robot_joint state
   * @param in Gripping point twist, in the gripping point frame.
   * @param out End-effector twist, in the end-effector frame.
   * @return False in case something goes wrong, true otherwise.
   **/
  bool grippingTwistToEef(const std::string &end_effector_link,
                          const sensor_msgs::JointState &state,
                          const KDL::Twist &in, KDL::Twist &out) const;

  /**
    Returns the inverse differential kinematics of the requested
  end-effector, given a desired twist.

    @param end_effector_link The name of the requested end-effector.
    @param state The current robot joint state.
    @param in Desired eef twist.
    @param out Joint state for the desired twist.
    @return False in case something goes wrong, true otherwise.
  **/
  bool getVelIK(const std::string &end_effector_link,
                const sensor_msgs::JointState &state, const KDL::Twist &in,
                KDL::JntArray &out) const;

  /**
    Returns the jacobian the requested end-effector's chain.

    @param end_effector_link The name of the requested end-effector.
    @param state The current robot joint state.
    @param out End-effector's jacobian.
    @return False in case something goes wrong, true otherwise.
  **/
  bool getJacobian(const std::string &end_effector_link,
                   const sensor_msgs::JointState &state,
                   KDL::Jacobian &out) const;

  /**
    Returns the pose of the requested end-effector, given a joint state.

    @param end_effector_link The name of the requested end-effector.
    @param state The current robot joint state.
    @param out The resulting eef pose.
    @return False in case something goes wrong, true otherwise.
  **/
  bool getEefPose(const std::string &end_effector_link,
                  const sensor_msgs::JointState &state, KDL::Frame &out) const;

  /**
    Returns the twist of the requested end-effector, given a joint state.

    @param end_effector_link The name of the requested end-effector.
    @param state The current robot joint state.
    @param out The resulting eef twist.
    @return False in case something goes wrong, true otherwise.
  **/
  bool getEefTwist(const std::string &end_effector_link,
                   const sensor_msgs::JointState &state,
                   KDL::FrameVel &out) const;

  /**
    Returns the joint limits for all the actuated joints in the eef kinematic
    chain.

    @param end_effector_link The name of the requested end-effector.
    @param q_min The lower joint position limit.
    @param q_max The upper joint position limit.
    @param q_vel_lim The maximum joint velocity.
    @return False in case something goes wrong, true otherwise.
  **/
  bool getJointLimits(const std::string &end_effector_link,
                      KDL::JntArray &q_min, KDL::JntArray &q_max,
                      KDL::JntArray &q_vel_lim) const;

  /**
    Returns the current joint positions in the KDL format.

    @param end_effector_link The name of the requested end-effector.
    @param state The current joint state
    @param q The joint positions in KDL format
    @return False in case something goes wrong, true otherwise.
  **/
  bool getJointPositions(const std::string &end_effector_link,
                         const sensor_msgs::JointState &state,
                         KDL::JntArray &q) const;
  bool getJointPositions(const std::string &end_effector_link,
                         const sensor_msgs::JointState &state,
                         Eigen::VectorXd &q) const;

  /**
    Returns the current joint velocities in the KDL format.

    @param end_effector_link The name of the requested end-effector.
    @param state The current joint state.
    @param q_dot The joint velocities in the KDL format.
    @returns False in case something goes wrong, true otherwise.
  **/
  bool getJointVelocities(const std::string &end_effector_link,
                          const sensor_msgs::JointState &state,
                          KDL::JntArray &q_dot) const;

  /**
    Computes the inertia matrix of the kinematic chain.

    @param end_effector_link The name of the requested end-effector.
    @param state The current joint state.
    @param H The output inertia matrix.
    @returns False in case something goes wrong, true otherwise.
  **/
  bool getInertia(const std::string &end_effector_link,
                  const sensor_msgs::JointState &state, Eigen::MatrixXd &H);

  /**
    Computes the predicted gravity effect on the given chain.

    @param end_effector_link The name of the requested end-effector.
    @param state The current joint state.
    @param g The output gravity effort.
    @returns False in case something goes wrong, true otherwise.
  **/
  bool getGravity(const std::string &end_effector_link,
                  const sensor_msgs::JointState &state, Eigen::MatrixXd &g);

  /**
    Computes the coriolis forces exerted in the given chain.

    @param end_effector_link The name of the requested end-effector.
    @param state The current joint state.
    @param coriolis The output coriolis effort.
    @returns False in case something goes wrong, true otherwise.
  **/
  bool getCoriolis(const std::string &end_effector_link,
                   const sensor_msgs::JointState &state,
                   Eigen::MatrixXd &coriolis);

  /**
    Returns the number of joints for a given chain.

    @param end_effector_link The name of the requested end-effector.
    @param num_joints The number of joints of the associated kinematic chain.
    @returns False in case something goes wrong, true otherwise.
  **/
  bool getNumJoints(const std::string &end_effector_link,
                    unsigned int &num_joints) const;

  /**
    Adds a new segment at the end of the requested chain. WARNING: The intended
  use case is to facilitate dynamic computations when the robot has a load that
  is not included in the chain's definition, e.g., a complex gripper. KDL does
  not support removing segments, so this should not be used to set a variable
  load.

    @param end_effector_link The name of the requested end-effector.
    @param new_segment A new segment to be added at the end of a chain.
  **/
  bool addSegment(const std::string &end_effector_link,
                  KDL::Segment &new_segment);

 private:
  typedef KDL::ChainIkSolverVel IkSolverVel;
  typedef KDL::ChainIkSolverPos_LMA IkSolverPos;
  typedef KDL::ChainFkSolverPos_recursive FkSolverPos;
  typedef KDL::ChainFkSolverVel_recursive FkSolverVel;
  typedef KDL::ChainJntToJacSolver JacSolver;

  typedef std::shared_ptr<IkSolverVel> IkSolverVelPtr;
  typedef std::shared_ptr<IkSolverPos> IkSolverPosPtr;
  typedef std::shared_ptr<FkSolverPos> FkSolverPosPtr;
  typedef std::shared_ptr<FkSolverVel> FkSolverVelPtr;
  typedef std::shared_ptr<JacSolver> JacSolverPtr;
  typedef std::shared_ptr<KDL::ChainDynParam> ChainDynParamPtr;

  std::map<std::string, IkSolverVelPtr> ikvel_;
  std::map<std::string, IkSolverPosPtr> ikpos_;
  std::map<std::string, FkSolverPosPtr> fkpos_;
  std::map<std::string, FkSolverVelPtr> fkvel_;
  std::map<std::string, JacSolverPtr> jac_solver_;
  std::map<std::string, KDL::Frame> eef_to_gripping_point_;
  std::map<std::string, KDL::Frame> eef_to_sensor_point_;
  std::map<std::string, KDL::Chain> chain_;
  std::map<std::string, ChainDynParamPtr> dynamic_chain_;

  urdf::Model model_;
  ros::NodeHandle nh_;
  TFManager tf_manager_;
  KDL::Vector gravity_in_chain_base_link_;
  std::map<std::string, std::vector<std::string> >
      actuated_joint_names_;  /// list of actuated joints per arm
  std::string chain_base_link_, ikvel_solver_;
  double eps_, nso_weight_, ik_pos_tolerance_, ik_angle_tolerance_;

  /**
    Loads the manager parameters

    @return False in case something goes wrong, true otherwise
  **/
  bool getParam();

  /**
    Common part of the initialize arm methods.

    @param end_effector_link The name of the requested end-effector.
    @return False in case something goes wrong, true otherwise.
  **/
  bool initializeArmCommon(const std::string &end_effector_link);

  /**
    Queries TF to get the rigid transform between two frames

    @param base_frame The base frame of the transform.
    @param target_frame The target frame of the transform.
    @param out The rigid transform between the two frames in the KDL format.
    @return False in case something goes wrong, true otherwise.
  **/
  bool getRigidTransform(const std::string &base_frame,
                         const std::string &target_frame,
                         KDL::Frame &out) const;

  /**
    Fills in the joint arrays with the state of a given kinematic chain.
    The joint state might include joints outside of the kinematic chain, so
  there is the need to process it.

    @param current_state The robot joint state.
    @param arm The target arm index.
    @param positions The joint positions of the kinematic chain.
    @param velocities The joint velocities of the kinematic chain.
    @return True if the full joint chain was found in the current state, false
  otherwise.
  **/
  bool getChainJointState(const sensor_msgs::JointState &current_state,
                          const std::string &end_effector_link,
                          KDL::JntArray &positions,
                          KDL::JntArrayVel &velocities) const;

  /**
    Check if a chain has the given joint_name.

    @param chain The kinematic chain where to look for the joint.
    @param joint_name The joint name we wish to check.
    @return True if the joint was found in the kinematic chain, false otherwise.
  **/
  bool hasJoint(const KDL::Chain &chain, const std::string &joint_name) const;

  /**
   *  Updates the internal data structures of all solvers.
   **/
  void updateSolvers();
};

/**
  Initializes a kdl manager class with the given arm info.
  Uses the ros parameter server to obtain information.

  @param arm_info The arm information.
  @param manager Reference to the kdl manager.
  @return False if something goes wrong, true otherwise.
**/
bool setKDLManager(const ArmInfo &arm_info,
                   std::shared_ptr<KDLManager> manager);

/**
  Initializes a kdl manager class with the given arm info. Uses nullspace
optimization. Uses the ros parameter server to obtain information.

  @param arm_info The arm information.
  @param manager Reference to the kdl manager.
  @return False if something goes wrong, true otherwise.
**/
bool setKDLManagerNso(const ArmInfo &arm_info,
                      std::shared_ptr<KDLManager> manager);
}  // namespace generic_control_toolbox
#endif
