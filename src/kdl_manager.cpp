#include <generic_control_toolbox/kdl_manager.hpp>

namespace generic_control_toolbox
{
    KDLManager::KDLManager(const std::string &chain_base_link, ros::NodeHandle nh) : chain_base_link_(chain_base_link), nh_(nh)
    {
      if(!model_.initParam("/robot_description"))
      {
        throw std::runtime_error("ERROR getting robot description (/robot_description)");
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
        ROS_WARN("KDLManager: Missing ik_angle_tolerance parameter, setting default");
        ik_angle_tolerance_ = 0.01;
      }

      if (!nh_.getParam("kdl_manager/ik_pos_tolerance", ik_pos_tolerance_))
      {
        ROS_WARN("KDLManager: Missing ik_pos_tolerance parameter, setting default");
        ik_pos_tolerance_ = 0.005;
      }

      if (ikvel_solver_ != WDLS_SOLVER && ikvel_solver_ != NSO_SOLVER)
      {
        ROS_ERROR_STREAM("KDLManager: ikvel_solver has value " << ikvel_solver_ << "but admissible values are " << WDLS_SOLVER << " and " << NSO_SOLVER);
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

      return true;
    }

    bool KDLManager::initializeArm(const std::string &end_effector_link)
    {
      if (!initializeArmCommon(end_effector_link))
      {
        return false;
      }

      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      if (ikvel_solver_ == WDLS_SOLVER)
      {
        ikvel_.push_back(std::shared_ptr<KDL::ChainIkSolverVel_wdls>(new KDL::ChainIkSolverVel_wdls(chain_[arm], eps_)));
      }
      else
      {
        unsigned int joint_n = chain_[arm].getNrOfJoints();
        KDL::JntArray w(joint_n), q_min(joint_n), q_max(joint_n), q_vel_lim(joint_n), q_desired(joint_n);
        getJointLimits(end_effector_link, q_min, q_max, q_vel_lim);

        for (unsigned int i = 0; i < joint_n; i++)
        {
          w(i) = nso_weight_;
          q_desired(i) = (q_max(i) + q_min(i))/2;
        }

        ikvel_.push_back(std::shared_ptr<KDL::ChainIkSolverVel_pinv_nso>(new KDL::ChainIkSolverVel_pinv_nso(chain_[arm], q_desired, w, eps_)));
      }

      return true;
    }

    bool KDLManager::initializeArmCommon(const std::string &end_effector_link)
    {
      int a;
      if(getIndex(end_effector_link, a))
      {
        ROS_ERROR_STREAM("Tried to initialize arm " << end_effector_link << ", but it was already initialized");
        return false;
      }

      KDL::Tree tree;
      KDL::Joint kdl_joint;
      KDL::Chain chain;
      kdl_parser::treeFromUrdfModel(model_, tree); // convert URDF description of the robot into a KDL tree
      if(!tree.getChain(chain_base_link_, end_effector_link, chain))
      {
        ROS_ERROR_STREAM("Failed to find chain <" << chain_base_link_ << ", " << end_effector_link << "> in the kinematic tree");
        return false;
      }

      // Ready to accept the end-effector as valid
      manager_index_.push_back(end_effector_link);
      chain_.push_back(chain);
      std::vector<std::string> new_vector;

      ROS_DEBUG("Initializing chain:");
      for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) // check for non-movable joints
      {
        kdl_joint = chain.getSegment(i).getJoint();

        if (kdl_joint.getTypeName() == "None")
        {
          continue;
        }

        ROS_DEBUG_STREAM(kdl_joint.getName());
        new_vector.push_back(kdl_joint.getName());
      }

      actuated_joint_names_.push_back(new_vector);

      // Initialize solvers
      fkpos_.push_back(std::shared_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(chain_.back())));
      fkvel_.push_back(std::shared_ptr<KDL::ChainFkSolverVel_recursive>(new KDL::ChainFkSolverVel_recursive(chain_.back())));
      ikpos_.push_back(std::shared_ptr<KDL::ChainIkSolverPos_LMA>(new KDL::ChainIkSolverPos_LMA(chain_.back())));
      eef_to_gripping_point_.push_back(KDL::Frame::Identity()); // Initialize a neutral transform.
      eef_to_sensor_point_.push_back(KDL::Frame::Identity()); // Initialize a neutral transform.
      jac_solver_.push_back(std::shared_ptr<KDL::ChainJntToJacSolver>(new KDL::ChainJntToJacSolver(chain_.back())));

      return true;
    }

    bool KDLManager::setGrippingPoint(const std::string &end_effector_link, const std::string &gripping_point_frame)
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::Frame eef_to_gripping_point;

      if (!getRigidTransform(end_effector_link, gripping_point_frame, eef_to_gripping_point))
      {
        return false;
      }

      eef_to_gripping_point_[arm] = eef_to_gripping_point.Inverse();
      return true;

    }

    bool KDLManager::setSensorPoint(const std::string &end_effector_link, const std::string &sensor_point_frame)
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::Frame eef_to_sensor_point;

      if (!getRigidTransform(end_effector_link, sensor_point_frame, eef_to_sensor_point))
      {
        return false;
      }

      eef_to_sensor_point_[arm] = eef_to_sensor_point;
      return true;
    }

    bool KDLManager::getJointState(const std::string &end_effector_link, const Eigen::VectorXd &qdot, sensor_msgs::JointState &state) const
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      if (chain_[arm].getNrOfJoints() != qdot.rows())
      {
        ROS_ERROR("Joint chain for eef %s has a different number of joints than the provided", end_effector_link.c_str());
        return false;
      }

      Eigen::VectorXd q(chain_[arm].getNrOfJoints());
      int joint_index = 0;

      for (unsigned long i = 0; i < state.name.size(); i++)
      {
        if (hasJoint(chain_[arm], state.name[i]))
        {
          q[joint_index] = state.position[i];
          joint_index++;
        }

        if (joint_index == chain_[arm].getNrOfJoints())
        {
          break;
        }
      }

      if (joint_index != chain_[arm].getNrOfJoints())
      {
        ROS_ERROR("Provided joint state does not have all of the required chain joints");
        return false;
      }

      return getJointState(end_effector_link, q, qdot, state);
    }

    bool KDLManager::getJointState(const std::string &end_effector_link, const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, sensor_msgs::JointState &state) const
    {
      if (q.rows() != qdot.rows())
      {
        ROS_ERROR("Given joint state with a different number of joint positions and velocities");
        return false;
      }

      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      if (chain_[arm].getNrOfJoints() != qdot.rows())
      {
        ROS_ERROR("Joint chain for eef %s has a different number of joints than the provided", end_effector_link.c_str());
        return false;
      }

      bool found;

      for (unsigned long i = 0; i < actuated_joint_names_[arm].size(); i ++)
      {
        found = false;
        for (unsigned long j = 0; j < state.name.size(); j++)
        {
          if (state.name[j] == actuated_joint_names_[arm][i])
          {
            state.position[j] = q[i];
            state.velocity[j] = qdot[i];
            found = true;
            break;
          }
        }

        if (!found)
        {
          ROS_ERROR_STREAM("KDLManager: Missing joint " << actuated_joint_names_[arm][i] << " from given joint state");
          return false;
        }
      }

      return true;
    }

    bool KDLManager::getGrippingPoint(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Frame &out) const
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::Frame eef_pose;
      if (!getEefPose(end_effector_link, state, eef_pose))
      {
        return false;
      }

      out = eef_pose*eef_to_gripping_point_[arm];
      return true;
    }

    bool KDLManager::getSensorPoint(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Frame &out) const
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::Frame eef_pose;
      if (!getEefPose(end_effector_link, state, eef_pose))
      {
        return false;
      }

      out = eef_pose*eef_to_sensor_point_[arm];
      return true;
    }

    bool KDLManager::getGrippingTwist(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Twist &out) const
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
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

      vel_eig << eef_twist.GetTwist().vel.data[0], eef_twist.GetTwist().vel.data[1], eef_twist.GetTwist().vel.data[2];
      rot_eig << eef_twist.GetTwist().rot.data[0], eef_twist.GetTwist().rot.data[1], eef_twist.GetTwist().rot.data[2];
      r_eig << r.data[0], r.data[1], r.data[2];

      converted_vel = vel_eig - parser_.computeSkewSymmetric(r_eig)*rot_eig;
      out.vel = KDL::Vector(converted_vel[0], converted_vel[1], converted_vel[2]);
      out.rot = eef_twist.GetTwist().rot;

      return true;
    }

    bool KDLManager::getEefPose(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Frame &out) const
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::JntArray positions(chain_[arm].getNrOfJoints());
      KDL::JntArrayVel velocities(chain_[arm].getNrOfJoints());

      if (!getChainJointState(state, arm, positions, velocities))
      {
        return false;
      }

      fkpos_[arm]->JntToCart(positions, out);
      return true;
    }

    bool KDLManager::getEefTwist(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::FrameVel &out) const
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::JntArray positions(chain_[arm].getNrOfJoints());
      KDL::JntArrayVel velocities(chain_[arm].getNrOfJoints());
      if (!getChainJointState(state, arm, positions, velocities))
      {
        return false;
      }

      fkvel_[arm]->JntToCart(velocities, out);
      return true;
    }

    bool KDLManager::getJointLimits(const std::string &end_effector_link, KDL::JntArray &q_min, KDL::JntArray &q_max, KDL::JntArray &q_vel_lim) const
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      unsigned int joint_n = chain_[arm].getNrOfJoints();
      if (q_min.rows() != joint_n || q_max.rows() != joint_n || q_vel_lim.rows() != joint_n)
      {
        ROS_ERROR("KDLManager::getJointPositionLimits requires initialized joint arrays");
        return false;
      }

      boost::shared_ptr<const urdf::Joint> joint;
      boost::shared_ptr<urdf::JointLimits> limits;
      int j = 0;
      // run through the kinematic chain joints and get the limits from the urdf model
      for (unsigned int i = 0; i < chain_[arm].getNrOfSegments(); i++)
      {
        if (chain_[arm].getSegment(i).getJoint().getType() == KDL::Joint::JointType::None)
        {
          continue;
        }

        joint = model_.getJoint(chain_[arm].getSegment(i).getJoint().getName());
        limits = joint->limits;
        q_min(j) = limits->lower;
        q_max(j) = limits->upper;
        q_vel_lim(j) = limits->velocity;
        j++;
      }

      return true;
    }

    bool KDLManager::getJointPositions(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::JntArray &q) const
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::JntArrayVel v(q.rows());
      if (!getChainJointState(state, arm, q, v))
      {
        return false;
      }

      return true;
    }

    bool KDLManager::getRigidTransform(const std::string &base_frame, const std::string &target_frame, KDL::Frame &out) const
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
        ROS_ERROR("KDL manager could not find the transform between frames %s and %s", base_frame.c_str(), target_frame.c_str());
        return false;
      }

      tf::poseMsgToKDL(base_to_target.pose, out);
      return true;
    }

    bool KDLManager::verifyPose(const std::string &end_effector_link, const KDL::Frame &in) const
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      sensor_msgs::JointState dummy_state;

      for (unsigned int i = 0; i < actuated_joint_names_[arm].size(); i++)
      {
        dummy_state.name.push_back(actuated_joint_names_[arm][i]);
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

    bool KDLManager::getPoseIK(const std::string &end_effector_link, const sensor_msgs::JointState &state, const KDL::Frame &in, KDL::JntArray &out) const
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::JntArray positions(chain_[arm].getNrOfJoints());
      KDL::JntArrayVel velocities(chain_[arm].getNrOfJoints());
      KDL::Frame computedPose, difference;
      if (!getChainJointState(state, arm, positions, velocities))
      {
        return false;
      }

      out.resize(chain_[arm].getNrOfJoints());
      ikpos_[arm]->CartToJnt(positions, in, out);
      getPoseFK(end_effector_link, state, out, computedPose); // verify if the forward kinematics of the computed solution are close to the desired pose

      difference = computedPose.Inverse() * in;
      Eigen::Vector3d quat_v;
      double quat_w;

      difference.M.GetQuaternion(quat_v[0], quat_v[1], quat_v[2], quat_w);
      double  angle = 2*atan2(quat_v.norm(), quat_w);

      if (fabs(angle) > ik_angle_tolerance_)
      {
        ROS_ERROR("KDL manager could not compute pose ik for end-effector %s. Final orientation error was %.2f", end_effector_link.c_str(), angle);
        return false;
      }

      if (fabs(difference.p.Norm()) > ik_pos_tolerance_)
      {
        ROS_ERROR("KDL manager could not compute pose ik for end-effector %s. Final position error was %.2f", end_effector_link.c_str(), difference.p.Norm());
        return false;
      }

      return true;
    }

    bool KDLManager::getPoseFK(const std::string &end_effector_link, const sensor_msgs::JointState &state, const KDL::JntArray &in, KDL::Frame &out) const
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      fkpos_[arm]->JntToCart(in, out);
      return true;
    }

    bool KDLManager::getGrippingVelIK(const std::string &end_effector_link, const sensor_msgs::JointState &state, const KDL::Twist &in, KDL::JntArray &out) const
    {
      KDL::Frame eef_to_base;
      KDL::Twist modified_in;
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      if (!getEefPose(end_effector_link, state, eef_to_base))
      {
        return false;
      }

      modified_in = eef_to_base*eef_to_gripping_point_[arm]*in;

      if (!getVelIK(end_effector_link, state, modified_in, out))
      {
        return false;
      }

      return true;
    }

    bool KDLManager::getVelIK(const std::string &end_effector_link, const sensor_msgs::JointState &state, const KDL::Twist &in, KDL::JntArray &out) const
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::JntArray positions(chain_[arm].getNrOfJoints());
      KDL::JntArrayVel velocities(chain_[arm].getNrOfJoints());
      if (!getChainJointState(state, arm, positions, velocities))
      {
        return false;
      }

      out.resize(chain_[arm].getNrOfJoints());
      ikvel_[arm]->CartToJnt(positions, in, out);
      return true;
    }

    bool KDLManager::getJacobian(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Jacobian &out) const
    {
      int arm;

      if (!getIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::JntArray positions(chain_[arm].getNrOfJoints());
      KDL::JntArrayVel velocities(chain_[arm].getNrOfJoints());
      if (!getChainJointState(state, arm, positions, velocities))
      {
        return false;
      }

      jac_solver_[arm]->JntToJac(positions, out);
      return true;
    }

    bool KDLManager::getChainJointState(const sensor_msgs::JointState &current_state, int arm, KDL::JntArray &positions, KDL::JntArrayVel &velocities) const
    {
      unsigned int processed_joints = 0;
      unsigned int name_size, pos_size, vel_size;

      name_size = current_state.name.size();
      pos_size = current_state.position.size();
      vel_size = current_state.velocity.size();

      if (name_size != pos_size || name_size != vel_size)
      {
        ROS_ERROR("Got joint state where the name, position and velocity dimensions (resp. %d, %d, %d) are different", name_size, pos_size, vel_size);
        return false;
      }

      for (unsigned long i = 0; i < actuated_joint_names_[arm].size(); i++)
      {
        for (unsigned long j = 0; j < current_state.name.size(); j++)
        {
          if (actuated_joint_names_[arm][i] == current_state.name[j])
          {
            positions(processed_joints) = current_state.position[j];
            velocities.q(processed_joints) = current_state.position[j];
            velocities.qdot(processed_joints) = current_state.velocity[j];
            processed_joints++;
          }
        }
      }

      if (processed_joints != actuated_joint_names_[arm].size())
      {
        ROS_ERROR("Failed to acquire chain joint state");
        return false;
      }

      return true;
    }

    bool KDLManager::hasJoint(const KDL::Chain &chain, const std::string &joint_name) const
    {
      for (unsigned int i = 0; i < chain.getNrOfSegments(); i++)
      {
        if(chain.segments[i].getJoint().getName() == joint_name)
        {
          return true;
        }
      }

      return false;
    }

    bool setKDLManager(const ArmInfo &arm_info, std::shared_ptr<KDLManager> manager)
    {
      if(!manager->initializeArm(arm_info.kdl_eef_frame))
      {
        return false;
      }

      if (!manager->setGrippingPoint(arm_info.kdl_eef_frame, arm_info.gripping_frame))
      {
        return false;
      }

      ROS_DEBUG("Successfully set up arm %s with eef_frame %s and gripping_frame %s", arm_info.name.c_str(), arm_info.kdl_eef_frame.c_str(), arm_info.gripping_frame.c_str());

      return true;
    }
}
