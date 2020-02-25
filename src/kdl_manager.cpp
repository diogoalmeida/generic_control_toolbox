#include <generic_control_toolbox/kdl_manager.hpp>

namespace generic_control_toolbox
{
KDLManager::KDLManager(const std::string &chain_base_link, ros::NodeHandle nh)
    : chain_base_link_(chain_base_link), nh_(nh)
{
  if (!model_.initParam("/robot_description"))
  {
    throw std::runtime_error(
        "ERROR getting robot description (/robot_description)");
  }

  getParam();
}

KDLManager::~KDLManager() {}

bool KDLManager::getParam()
{
  if (!nh_.getParam("kdl_manager/eps", eps_))
  {
    ROS_WARN("KDLManager: Missing eps parameter, setting default");
    eps_ = 0.001;
  }

  if (!nh_.getParam("kdl_manager/max_tf_attempts", max_tf_attempts_))
  {
    ROS_WARN("KDLManager: Missing max_tf_attempts parameter, setting default");
    max_tf_attempts_ = 5;
  }

  if (!nh_.getParam("kdl_manager/ikvel_solver", ikvel_solver_))
  {
    ROS_WARN("KDLManager: Missing ikvel_solver parameter, setting default");
    ikvel_solver_ = WDLS_SOLVER;
  }

  if (!nh_.getParam("kdl_manager/ik_angle_tolerance", ik_angle_tolerance_))
  {
    ROS_WARN(
        "KDLManager: Missing ik_angle_tolerance parameter, setting default");
    ik_angle_tolerance_ = 0.01;
  }

  if (!nh_.getParam("kdl_manager/ik_pos_tolerance", ik_pos_tolerance_))
  {
    ROS_WARN("KDLManager: Missing ik_pos_tolerance parameter, setting default");
    ik_pos_tolerance_ = 0.005;
  }

  if (ikvel_solver_ != WDLS_SOLVER && ikvel_solver_ != NSO_SOLVER)
  {
    ROS_ERROR_STREAM("KDLManager: ikvel_solver has value "
                     << ikvel_solver_ << "but admissible values are "
                     << WDLS_SOLVER << " and " << NSO_SOLVER);
    ROS_WARN_STREAM("KDLManager: setting ikvel_solver to " << WDLS_SOLVER);
    ikvel_solver_ = WDLS_SOLVER;
  }

  if (ikvel_solver_ == NSO_SOLVER)
  {
    if (!nh_.getParam("kdl_manager/nso_weight", nso_weight_))
    {
      ROS_WARN("KDLManager: Missing nso_weight parameter, setting default");
      nso_weight_ = 4;
    }
  }

  std::vector<double> gravity;
  if (!nh_.getParam("kdl_manager/gravity_in_base_link", gravity))
  {
    ROS_WARN_STREAM(
        "KDLManager: Missing kdl_manager/gravity_in_base_link parameter. This "
        "will affect the dynamic solvers");
    gravity_in_chain_base_link_ = KDL::Vector(0, 0, 0);
  }
  else
  {
    if (gravity.size() != 3)
    {
      ROS_WARN_STREAM("KDLManager: Got gravity vector of size "
                      << gravity.size() << ". Should have size 3");
      gravity_in_chain_base_link_ = KDL::Vector(0, 0, 0);
    }
    else
    {
      gravity_in_chain_base_link_ =
          KDL::Vector(gravity[0], gravity[1], gravity[2]);
    }
  }

  return true;
}

bool KDLManager::initializeArm(const std::string &end_effector_link)
{
  if (!initializeArmCommon(end_effector_link))
  {
    return false;
  }

  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  if (ikvel_solver_ == WDLS_SOLVER)
  {
    ikvel_[end_effector_link] = IkSolverVelPtr(
        new KDL::ChainIkSolverVel_wdls(chain_.at(end_effector_link), eps_));
  }
  else
  {
    unsigned int joint_n = chain_.at(end_effector_link).getNrOfJoints();

    if (joint_n < 6)
    {
      ROS_WARN(
          "Number of joints for kinematic chain is smaller than 6 (%d). The "
          "NSO ik solver vel has issues with under-actuated chains. Using WDLS",
          joint_n);
      ikvel_[end_effector_link] = IkSolverVelPtr(
          new KDL::ChainIkSolverVel_wdls(chain_.at(end_effector_link), eps_));
    }
    else
    {
      KDL::JntArray w(joint_n), q_min(joint_n), q_max(joint_n),
          q_vel_lim(joint_n), q_desired(joint_n);
      getJointLimits(end_effector_link, q_min, q_max, q_vel_lim);

      for (unsigned int i = 0; i < joint_n; i++)
      {
        w(i) = nso_weight_;
        q_desired(i) = (q_max(i) + q_min(i)) / 2;
      }

      ikvel_[end_effector_link] =
          IkSolverVelPtr(new KDL::ChainIkSolverVel_pinv_nso(
              chain_.at(end_effector_link), q_desired, w, eps_));
    }
  }

  return true;
}

bool KDLManager::isInitialized(const std::string &end_effector_link) const
{
  return (chain_.find(end_effector_link) != chain_.end());
}

bool KDLManager::initializeArmCommon(const std::string &end_effector_link)
{
  if (chain_.find(end_effector_link) != chain_.end())
  {
    ROS_ERROR_STREAM("Tried to initialize arm "
                     << end_effector_link
                     << ", but it was already initialized");
    return false;
  }

  KDL::Tree tree;
  KDL::Joint kdl_joint;
  KDL::Chain chain;
  kdl_parser::treeFromUrdfModel(
      model_, tree);  // convert URDF description of the robot into a KDL tree
  if (!tree.getChain(chain_base_link_, end_effector_link, chain))
  {
    ROS_ERROR_STREAM("Failed to find chain <" << chain_base_link_ << ", "
                                              << end_effector_link
                                              << "> in the kinematic tree");
    return false;
  }

  // Ready to accept the end-effector as valid
  chain_[end_effector_link] = chain;
  dynamic_chain_[end_effector_link] = ChainDynParamPtr(new KDL::ChainDynParam(
      chain_.at(end_effector_link), gravity_in_chain_base_link_));
  std::vector<std::string> new_vector;

  ROS_DEBUG_STREAM("Initializing chain for arm " << end_effector_link);
  for (unsigned int i = 0; i < chain.getNrOfSegments();
       i++)  // check for non-movable joints
  {
    kdl_joint = chain.getSegment(i).getJoint();

    if (kdl_joint.getTypeName() == "None")
    {
      continue;
    }

    ROS_DEBUG_STREAM(kdl_joint.getName());
    new_vector.push_back(kdl_joint.getName());
  }

  actuated_joint_names_[end_effector_link] = new_vector;

  // Initialize solvers
  fkpos_[end_effector_link] =
      FkSolverPosPtr(new FkSolverPos(chain_.at(end_effector_link)));
  fkvel_[end_effector_link] =
      FkSolverVelPtr(new FkSolverVel(chain_.at(end_effector_link)));
  ikpos_[end_effector_link] =
      IkSolverPosPtr(new IkSolverPos(chain_base_link_, end_effector_link));
  jac_solver_[end_effector_link] =
      JacSolverPtr(new JacSolver(chain_.at(end_effector_link)));
  eef_to_gripping_point_[end_effector_link] =
      KDL::Frame::Identity();  // Initialize a neutral transform.
  eef_to_sensor_point_[end_effector_link] =
      KDL::Frame::Identity();  // Initialize a neutral transform.

  return true;
}

bool KDLManager::setGrippingPoint(const std::string &end_effector_link,
                                  const std::string &gripping_point_frame)
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::Frame eef_to_gripping_point;

  if (!getRigidTransform(end_effector_link, gripping_point_frame,
                         eef_to_gripping_point))
  {
    return false;
  }

  eef_to_gripping_point_.at(end_effector_link) =
      eef_to_gripping_point.Inverse();
  return true;
}

bool KDLManager::setSensorPoint(const std::string &end_effector_link,
                                const std::string &sensor_point_frame)
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::Frame eef_to_sensor_point;

  if (!getRigidTransform(end_effector_link, sensor_point_frame,
                         eef_to_sensor_point))
  {
    return false;
  }

  eef_to_sensor_point_.at(end_effector_link) = eef_to_sensor_point;
  return true;
}

bool KDLManager::getJointState(const std::string &end_effector_link,
                               const Eigen::VectorXd &qdot,
                               sensor_msgs::JointState &state) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  if (chain_.at(end_effector_link).getNrOfJoints() != qdot.rows())
  {
    ROS_ERROR(
        "Joint chain for eef %s has a different number of joints than the "
        "provided",
        end_effector_link.c_str());
    return false;
  }

  Eigen::VectorXd q(chain_.at(end_effector_link).getNrOfJoints());
  int joint_index = 0;

  for (unsigned long i = 0; i < state.name.size(); i++)
  {
    if (hasJoint(chain_.at(end_effector_link), state.name[i]))
    {
      q[joint_index] = state.position[i];
      joint_index++;
    }

    if (joint_index == chain_.at(end_effector_link).getNrOfJoints())
    {
      break;
    }
  }

  if (joint_index != chain_.at(end_effector_link).getNrOfJoints())
  {
    ROS_ERROR(
        "Provided joint state does not have all of the required chain joints");
    return false;
  }

  return getJointState(end_effector_link, q, qdot, state);
}

bool KDLManager::createJointState(const std::string &end_effector_link,
                                  const Eigen::VectorXd &q,
                                  const Eigen::VectorXd &qdot,
                                  sensor_msgs::JointState &state) const
{
  sensor_msgs::JointState ret;
  if (q.rows() != qdot.rows())
  {
    ROS_ERROR(
        "Given joint state with a different number of joint positions and "
        "velocities");
    return false;
  }

  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  if (chain_.at(end_effector_link).getNrOfJoints() != qdot.rows())
  {
    ROS_ERROR(
        "Joint chain for eef %s has a different number of joints than the "
        "provided",
        end_effector_link.c_str());
    return false;
  }

  state.name.resize(actuated_joint_names_.at(end_effector_link).size());
  state.position.resize(actuated_joint_names_.at(end_effector_link).size());
  state.velocity.resize(actuated_joint_names_.at(end_effector_link).size());
  state.effort.resize(actuated_joint_names_.at(end_effector_link).size());
  for (unsigned long i = 0;
       i < actuated_joint_names_.at(end_effector_link).size(); i++)
  {
    state.name[i] = actuated_joint_names_.at(end_effector_link)[i];
    state.position[i] = q[i];
    state.velocity[i] = qdot[i];
  }

  return true;
}

bool KDLManager::getJointState(const std::string &end_effector_link,
                               const Eigen::VectorXd &q,
                               const Eigen::VectorXd &qdot,
                               sensor_msgs::JointState &state) const
{
  if (q.rows() != qdot.rows())
  {
    ROS_ERROR(
        "Given joint state with a different number of joint positions and "
        "velocities");
    return false;
  }

  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  if (chain_.at(end_effector_link).getNrOfJoints() != qdot.rows())
  {
    ROS_ERROR(
        "Joint chain for eef %s has a different number of joints than the "
        "provided",
        end_effector_link.c_str());
    return false;
  }

  bool found;

  for (unsigned long i = 0;
       i < actuated_joint_names_.at(end_effector_link).size(); i++)
  {
    found = false;
    for (unsigned long j = 0; j < state.name.size(); j++)
    {
      if (state.name[j] == actuated_joint_names_.at(end_effector_link)[i])
      {
        state.position[j] = q[i];
        state.velocity[j] = qdot[i];
        found = true;
        break;
      }
    }

    if (!found)
    {
      ROS_ERROR_STREAM("KDLManager: Missing joint "
                       << actuated_joint_names_.at(end_effector_link)[i]
                       << " from given joint state");
      return false;
    }
  }

  return true;
}

bool KDLManager::getJointState(const std::string &end_effector_link,
                               const Eigen::VectorXd &q,
                               const Eigen::VectorXd &qdot,
                               const Eigen::VectorXd &effort,
                               sensor_msgs::JointState &state) const
{
  if (!getJointState(end_effector_link, q, qdot, state))
  {
    return false;
  }

  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  if (chain_.at(end_effector_link).getNrOfJoints() != effort.rows())
  {
    ROS_ERROR_STREAM("Joint chain for eef "
                     << end_effector_link
                     << " has a different number of joints than the provided");
    return false;
  }

  bool found;
  for (unsigned long i = 0;
       i < actuated_joint_names_.at(end_effector_link).size(); i++)
  {
    found = false;
    for (unsigned long j = 0; j < state.name.size(); j++)
    {
      if (state.name[j] == actuated_joint_names_.at(end_effector_link)[i])
      {
        state.effort[j] = effort[i];
        found = true;
        break;
      }
    }

    if (!found)
    {
      ROS_ERROR_STREAM("KDLManager: Missing joint "
                       << actuated_joint_names_.at(end_effector_link)[i]
                       << " from given joint state");
      return false;
    }
  }

  return true;
}

bool KDLManager::getGrippingPoint(const std::string &end_effector_link,
                                  const sensor_msgs::JointState &state,
                                  KDL::Frame &out) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::Frame eef_pose;
  if (!getEefPose(end_effector_link, state, eef_pose))
  {
    return false;
  }

  out = eef_pose * eef_to_gripping_point_.at(end_effector_link);
  return true;
}

bool KDLManager::getSensorPoint(const std::string &end_effector_link,
                                const sensor_msgs::JointState &state,
                                KDL::Frame &out) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::Frame eef_pose;
  if (!getEefPose(end_effector_link, state, eef_pose))
  {
    return false;
  }

  out = eef_pose * eef_to_sensor_point_.at(end_effector_link);
  return true;
}

bool KDLManager::getGrippingTwist(const std::string &end_effector_link,
                                  const sensor_msgs::JointState &state,
                                  KDL::Twist &out) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::Frame gripping_pose;
  if (!getGrippingPoint(end_effector_link, state, gripping_pose))
  {
    return false;
  }

  KDL::Frame eef_pose;
  if (!getEefPose(end_effector_link, state, eef_pose))
  {
    return false;
  }

  KDL::FrameVel eef_twist;
  if (!getEefTwist(end_effector_link, state, eef_twist))
  {
    return false;
  }

  Eigen::Vector3d vel_eig, rot_eig, converted_vel, r_eig;
  KDL::Vector r = gripping_pose.p - eef_pose.p;

  vel_eig << eef_twist.GetTwist().vel.data[0], eef_twist.GetTwist().vel.data[1],
      eef_twist.GetTwist().vel.data[2];
  rot_eig << eef_twist.GetTwist().rot.data[0], eef_twist.GetTwist().rot.data[1],
      eef_twist.GetTwist().rot.data[2];
  r_eig << r.data[0], r.data[1], r.data[2];

  converted_vel = vel_eig - MatrixParser::computeSkewSymmetric(r_eig) * rot_eig;
  out.vel = KDL::Vector(converted_vel[0], converted_vel[1], converted_vel[2]);
  out.rot = eef_twist.GetTwist().rot;

  return true;
}

bool KDLManager::getEefPose(const std::string &end_effector_link,
                            const sensor_msgs::JointState &state,
                            KDL::Frame &out) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::JntArray positions(chain_.at(end_effector_link).getNrOfJoints());
  KDL::JntArrayVel velocities(chain_.at(end_effector_link).getNrOfJoints());

  if (!getChainJointState(state, end_effector_link, positions, velocities))
  {
    return false;
  }

  fkpos_.at(end_effector_link)->JntToCart(positions, out);
  return true;
}

bool KDLManager::getEefTwist(const std::string &end_effector_link,
                             const sensor_msgs::JointState &state,
                             KDL::FrameVel &out) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::JntArray positions(chain_.at(end_effector_link).getNrOfJoints());
  KDL::JntArrayVel velocities(chain_.at(end_effector_link).getNrOfJoints());
  if (!getChainJointState(state, end_effector_link, positions, velocities))
  {
    return false;
  }

  fkvel_.at(end_effector_link)->JntToCart(velocities, out);
  return true;
}

bool KDLManager::getJointLimits(const std::string &end_effector_link,
                                KDL::JntArray &q_min, KDL::JntArray &q_max,
                                KDL::JntArray &q_vel_lim) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  unsigned int joint_n = chain_.at(end_effector_link).getNrOfJoints();
  if (q_min.rows() != joint_n || q_max.rows() != joint_n ||
      q_vel_lim.rows() != joint_n)
  {
    ROS_ERROR(
        "KDLManager::getJointPositionLimits requires initialized joint arrays");
    return false;
  }

  urdf::JointConstSharedPtr joint;
  urdf::JointLimitsSharedPtr limits;
  int j = 0;
  // run through the kinematic chain joints and get the limits from the urdf
  // model
  for (unsigned int i = 0; i < chain_.at(end_effector_link).getNrOfSegments();
       i++)
  {
    if (chain_.at(end_effector_link).getSegment(i).getJoint().getType() ==
        KDL::Joint::JointType::None)
    {
      continue;
    }

    joint = model_.getJoint(
        chain_.at(end_effector_link).getSegment(i).getJoint().getName());
    limits = joint->limits;
    q_min(j) = limits->lower;
    q_max(j) = limits->upper;
    q_vel_lim(j) = limits->velocity;
    j++;
  }

  return true;
}

bool KDLManager::getJointPositions(const std::string &end_effector_link,
                                   const sensor_msgs::JointState &state,
                                   KDL::JntArray &q) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  q.resize(chain_.at(end_effector_link).getNrOfJoints());
  KDL::JntArrayVel v(q.rows());
  if (!getChainJointState(state, end_effector_link, q, v))
  {
    return false;
  }

  return true;
}

bool KDLManager::getJointPositions(const std::string &end_effector_link,
                                   const sensor_msgs::JointState &state,
                                   Eigen::VectorXd &q) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::JntArray q_kdl;

  if (!getJointPositions(end_effector_link, state, q_kdl))
  {
    return false;
  }

  q = q_kdl.data;
  return true;
}

bool KDLManager::getJointVelocities(const std::string &end_effector_link,
                                    const sensor_msgs::JointState &state,
                                    KDL::JntArray &q_dot) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  q_dot.resize(chain_.at(end_effector_link).getNrOfJoints());
  KDL::JntArray q(q_dot.rows());
  KDL::JntArrayVel v(q_dot.rows());
  if (!getChainJointState(state, end_effector_link, q, v))
  {
    return false;
  }

  q_dot = v.qdot;
  return true;
}

bool KDLManager::getInertia(const std::string &end_effector_link,
                            const sensor_msgs::JointState &state,
                            Eigen::MatrixXd &H)
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::JntArray q(chain_.at(end_effector_link).getNrOfJoints());
  KDL::JntSpaceInertiaMatrix B(chain_.at(end_effector_link).getNrOfJoints());
  dynamic_chain_.at(end_effector_link)->JntToMass(q, B);

  H = B.data;
  return true;
}

bool KDLManager::getGravity(const std::string &end_effector_link,
                            const sensor_msgs::JointState &state,
                            Eigen::MatrixXd &g)
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::JntArray q(chain_.at(end_effector_link).getNrOfJoints());
  KDL::JntArrayVel q_dot(chain_.at(end_effector_link).getNrOfJoints());
  if (!getChainJointState(state, end_effector_link, q, q_dot))
  {
    return false;
  }

  KDL::JntArray q_gravity(chain_.at(end_effector_link).getNrOfJoints());
  dynamic_chain_.at(end_effector_link)->JntToGravity(q, q_gravity);

  g = q_gravity.data;

  return true;
}

bool KDLManager::getCoriolis(const std::string &end_effector_link,
                             const sensor_msgs::JointState &state,
                             Eigen::MatrixXd &coriolis)
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::JntArray q(chain_.at(end_effector_link).getNrOfJoints());
  KDL::JntArrayVel q_dot(chain_.at(end_effector_link).getNrOfJoints());
  if (!getChainJointState(state, end_effector_link, q, q_dot))
  {
    return false;
  }

  KDL::JntArray cor(chain_.at(end_effector_link).getNrOfJoints());
  dynamic_chain_.at(end_effector_link)->JntToCoriolis(q, q_dot.qdot, cor);
  coriolis = cor.data;
  return true;
}

bool KDLManager::getRigidTransform(const std::string &base_frame,
                                   const std::string &target_frame,
                                   KDL::Frame &out) const
{
  geometry_msgs::PoseStamped base_to_target;
  base_to_target.header.frame_id = base_frame;
  base_to_target.header.stamp = ros::Time(0);
  base_to_target.pose.position.x = 0;
  base_to_target.pose.position.y = 0;
  base_to_target.pose.position.z = 0;
  base_to_target.pose.orientation.x = 0;
  base_to_target.pose.orientation.y = 0;
  base_to_target.pose.orientation.z = 0;
  base_to_target.pose.orientation.w = 1;

  int attempts;
  for (attempts = 0; attempts < max_tf_attempts_; attempts++)
  {
    try
    {
      listener_.transformPose(target_frame, base_to_target, base_to_target);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("TF exception in kdl manager: %s", ex.what());
      ros::Duration(0.1).sleep();
    }
  }

  if (attempts >= max_tf_attempts_)
  {
    ROS_ERROR(
        "KDL manager could not find the transform between frames %s and %s",
        base_frame.c_str(), target_frame.c_str());
    return false;
  }

  tf::poseMsgToKDL(base_to_target.pose, out);
  return true;
}

bool KDLManager::verifyPose(const std::string &end_effector_link,
                            const KDL::Frame &in) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  sensor_msgs::JointState dummy_state;

  for (unsigned int i = 0;
       i < actuated_joint_names_.at(end_effector_link).size(); i++)
  {
    dummy_state.name.push_back(actuated_joint_names_.at(end_effector_link)[i]);
    dummy_state.position.push_back(0);
    dummy_state.velocity.push_back(0);
    dummy_state.effort.push_back(0);
  }

  KDL::JntArray dummy_out(dummy_state.name.size());
  if (getPoseIK(end_effector_link, dummy_state, in, dummy_out))
  {
    return true;
  }

  return false;
}

bool KDLManager::getPoseIK(const std::string &end_effector_link,
                           const sensor_msgs::JointState &state,
                           const KDL::Frame &in, KDL::JntArray &out) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::JntArray positions(chain_.at(end_effector_link).getNrOfJoints());
  KDL::JntArrayVel velocities(chain_.at(end_effector_link).getNrOfJoints());
  KDL::Frame computedPose, difference;
  if (!getChainJointState(state, end_effector_link, positions, velocities))
  {
    return false;
  }

  out.resize(chain_.at(end_effector_link).getNrOfJoints());
  ikpos_.at(end_effector_link)->CartToJnt(positions, in, out);
  getPoseFK(end_effector_link, state, out,
            computedPose);  // verify if the forward kinematics of the computed
                            // solution are close to the desired pose

  difference = computedPose.Inverse() * in;
  Eigen::Vector3d quat_v;
  double quat_w;

  difference.M.GetQuaternion(quat_v[0], quat_v[1], quat_v[2], quat_w);
  double angle = 2 * atan2(quat_v.norm(), quat_w);

  if (fabs(angle) > ik_angle_tolerance_)
  {
    ROS_ERROR(
        "KDL manager could not compute pose ik for end-effector %s. Final "
        "orientation error was %.2f",
        end_effector_link.c_str(), angle);
    return false;
  }

  if (fabs(difference.p.Norm()) > ik_pos_tolerance_)
  {
    ROS_ERROR(
        "KDL manager could not compute pose ik for end-effector %s. Final "
        "position error was %.2f",
        end_effector_link.c_str(), difference.p.Norm());
    return false;
  }

  return true;
}

bool KDLManager::getGrippingPoseIK(const std::string &end_effector_link,
                                   const sensor_msgs::JointState &state,
                                   const KDL::Frame &in,
                                   KDL::JntArray &out) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::Frame pose_in_eef =
      in * eef_to_gripping_point_.at(end_effector_link).Inverse();

  return getPoseIK(end_effector_link, state, pose_in_eef, out);
}

bool KDLManager::getPoseFK(const std::string &end_effector_link,
                           const sensor_msgs::JointState &state,
                           const KDL::JntArray &in, KDL::Frame &out) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  fkpos_.at(end_effector_link)->JntToCart(in, out);
  return true;
}

bool KDLManager::getGrippingVelIK(const std::string &end_effector_link,
                                  const sensor_msgs::JointState &state,
                                  const KDL::Twist &in,
                                  KDL::JntArray &out) const
{
  KDL::Frame pgrip, peef;
  KDL::Twist modified_in, rotated_in;

  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  if (!getGrippingPoint(end_effector_link, state, pgrip))
  {
    return false;
  }

  if (!getEefPose(end_effector_link, state, peef))
  {
    return false;
  }

  // convert the input twist (in the gripping frame) to the base frame
  Eigen::Vector3d vel_eig, rot_eig, peef_eig, pgrip_eig;
  modified_in = pgrip.M * in;

  tf::vectorKDLToEigen(peef.p, peef_eig);
  tf::vectorKDLToEigen(pgrip.p, pgrip_eig);
  tf::vectorKDLToEigen(modified_in.vel, vel_eig);
  tf::vectorKDLToEigen(modified_in.rot, rot_eig);

  vel_eig = MatrixParser::computeSkewSymmetric(pgrip_eig - peef_eig) * rot_eig +
            vel_eig;

  tf::vectorEigenToKDL(vel_eig, modified_in.vel);

  if (!getVelIK(end_effector_link, state, modified_in, out))
  {
    return false;
  }

  return true;
}

bool KDLManager::getVelIK(const std::string &end_effector_link,
                          const sensor_msgs::JointState &state,
                          const KDL::Twist &in, KDL::JntArray &out) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::JntArray positions(chain_.at(end_effector_link).getNrOfJoints());
  KDL::JntArrayVel velocities(chain_.at(end_effector_link).getNrOfJoints());
  if (!getChainJointState(state, end_effector_link, positions, velocities))
  {
    return false;
  }

  out.resize(chain_.at(end_effector_link).getNrOfJoints());
  ikvel_.at(end_effector_link)->CartToJnt(positions, in, out);
  return true;
}

bool KDLManager::getJacobian(const std::string &end_effector_link,
                             const sensor_msgs::JointState &state,
                             KDL::Jacobian &out) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  KDL::JntArray positions(chain_.at(end_effector_link).getNrOfJoints());
  KDL::JntArrayVel velocities(chain_.at(end_effector_link).getNrOfJoints());
  if (!getChainJointState(state, end_effector_link, positions, velocities))
  {
    return false;
  }

  out.resize(positions.rows());
  jac_solver_.at(end_effector_link)->JntToJac(positions, out);
  return true;
}

bool KDLManager::checkStateMessage(const std::string &end_effector_link,
                                   const sensor_msgs::JointState &state) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  unsigned int processed_joints = 0, name_size, pos_size, vel_size;
  name_size = state.name.size();
  pos_size = state.position.size();
  vel_size = state.velocity.size();

  if (name_size != pos_size || name_size != vel_size)
  {
    ROS_ERROR(
        "Got joint state where the name, position and velocity dimensions "
        "(resp. %d, %d, %d) are different",
        name_size, pos_size, vel_size);
    return false;
  }

  for (unsigned long i = 0;
       i < actuated_joint_names_.at(end_effector_link).size(); i++)
  {
    for (unsigned long j = 0; j < state.name.size(); j++)
    {
      if (actuated_joint_names_.at(end_effector_link)[i] == state.name[j])
      {
        processed_joints++;
      }
    }
  }

  if (processed_joints != actuated_joint_names_.at(end_effector_link).size())
  {
    return false;
  }

  return true;
}

bool KDLManager::getChainJointState(
    const sensor_msgs::JointState &current_state,
    const std::string &end_effector_link, KDL::JntArray &positions,
    KDL::JntArrayVel &velocities) const
{
  unsigned int processed_joints = 0;
  unsigned int name_size, pos_size, vel_size;

  name_size = current_state.name.size();
  pos_size = current_state.position.size();
  vel_size = current_state.velocity.size();

  if (name_size != pos_size || name_size != vel_size)
  {
    ROS_ERROR(
        "Got joint state where the name, position and velocity dimensions "
        "(resp. %d, %d, %d) are different",
        name_size, pos_size, vel_size);
    return false;
  }

  for (unsigned long i = 0;
       i < actuated_joint_names_.at(end_effector_link).size(); i++)
  {
    for (unsigned long j = 0; j < current_state.name.size(); j++)
    {
      if (actuated_joint_names_.at(end_effector_link)[i] ==
          current_state.name[j])
      {
        positions(processed_joints) = current_state.position[j];
        velocities.q(processed_joints) = current_state.position[j];
        velocities.qdot(processed_joints) = current_state.velocity[j];
        processed_joints++;
      }
    }
  }

  if (processed_joints != actuated_joint_names_.at(end_effector_link).size())
  {
    ROS_ERROR("Failed to acquire chain joint state");
    return false;
  }

  return true;
}

bool KDLManager::hasJoint(const KDL::Chain &chain,
                          const std::string &joint_name) const
{
  for (unsigned int i = 0; i < chain.getNrOfSegments(); i++)
  {
    if (chain.segments[i].getJoint().getName() == joint_name)
    {
      return true;
    }
  }

  return false;
}

bool KDLManager::getNumJoints(const std::string &end_effector_link,
                              unsigned int &num_joints) const
{
  if (chain_.find(end_effector_link) == chain_.end())
  {
    return false;
  }

  num_joints = chain_.at(end_effector_link).getNrOfJoints();
  return true;
}

bool setKDLManager(const ArmInfo &arm_info, std::shared_ptr<KDLManager> manager)
{
  if (!manager->initializeArm(arm_info.kdl_eef_frame))
  {
    return false;
  }

  if (!manager->setGrippingPoint(arm_info.kdl_eef_frame,
                                 arm_info.gripping_frame))
  {
    return false;
  }

  ROS_DEBUG(
      "Successfully set up arm %s with eef_frame %s and gripping_frame %s",
      arm_info.name.c_str(), arm_info.kdl_eef_frame.c_str(),
      arm_info.gripping_frame.c_str());

  return true;
}
}  // namespace generic_control_toolbox
