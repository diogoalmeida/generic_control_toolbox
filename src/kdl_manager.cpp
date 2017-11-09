#include <generic_control_toolbox/kdl_manager.hpp>

namespace generic_control_toolbox
{
    KDLManager::KDLManager(const std::string &chain_base_link) : chain_base_link_(chain_base_link)
    {
      if(!model_.initParam("/robot_description"))
      {
        throw std::runtime_error("ERROR getting robot description (/robot_description)");
      }

      eps_ = 0.001; // TODO: Make parameter
      max_tf_attempts_ = 5;
    }

    KDLManager::~KDLManager() {}

    bool KDLManager::initializeArm(const std::string &end_effector_link)
    {
      int a;
      if(getArmIndex(end_effector_link, a))
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
      end_effector_.push_back(end_effector_link);
      chain_.push_back(chain);
      std::vector<std::string> new_vector;

      for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) // check for non-movable joints
      {
        kdl_joint = chain.getSegment(i).getJoint();

        if (kdl_joint.getTypeName() == "None")
        {
          continue;
        }

        new_vector.push_back(kdl_joint.getName());
      }

      actuated_joint_names_.push_back(new_vector);

      // Initialize solvers
      ikvel_.push_back(std::shared_ptr<KDL::ChainIkSolverVel_wdls>(new KDL::ChainIkSolverVel_wdls(chain, eps_)));
      fkpos_.push_back(std::shared_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(chain)));
      fkvel_.push_back(std::shared_ptr<KDL::ChainFkSolverVel_recursive>(new KDL::ChainFkSolverVel_recursive(chain)));
      ikpos_.push_back(std::shared_ptr<KDL::ChainIkSolverPos_LMA>(new KDL::ChainIkSolverPos_LMA(chain)));
      eef_to_gripping_point_.push_back(KDL::Frame::Identity()); // Initialize a neutral transform.
      jac_solver_.push_back(std::shared_ptr<KDL::ChainJntToJacSolver>(new KDL::ChainJntToJacSolver(chain)));

      return true;
    }

    bool KDLManager::setGrippingPoint(const std::string &end_effector_link, const std::string &gripping_point_frame)
    {
      int arm;

      if (!getArmIndex(end_effector_link, arm))
      {
        return false;
      }

      // get rigid transform between end-effector and arm gripping point
      geometry_msgs::PoseStamped eef_to_gripping_point;
      eef_to_gripping_point.header.frame_id = end_effector_link;
      eef_to_gripping_point.header.stamp = ros::Time(0);
      eef_to_gripping_point.pose.position.x = 0;
      eef_to_gripping_point.pose.position.y = 0;
      eef_to_gripping_point.pose.position.z = 0;
      eef_to_gripping_point.pose.orientation.x = 0;
      eef_to_gripping_point.pose.orientation.y = 0;
      eef_to_gripping_point.pose.orientation.z = 0;
      eef_to_gripping_point.pose.orientation.w = 1;

      int attempts;
      for (attempts = 0; attempts < max_tf_attempts_; attempts++)
      {
        try
        {
          listener_.transformPose(gripping_point_frame, eef_to_gripping_point, eef_to_gripping_point);
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
        ROS_ERROR("KDL manager could not find the transform between the end-effector frame %s and gripping point %s", end_effector_link.c_str(), gripping_point_frame.c_str());
        return false;
      }

      KDL::Frame eef_to_gripping_point_kdl;
      tf::poseMsgToKDL(eef_to_gripping_point.pose, eef_to_gripping_point_kdl);
      eef_to_gripping_point_[arm] = eef_to_gripping_point_kdl.Inverse();
      return true;
    }

    bool KDLManager::getJointState(const std::string &end_effector_link, const Eigen::VectorXd &qdot, sensor_msgs::JointState &state) const
    {
      int arm;

      if (!getArmIndex(end_effector_link, arm))
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

      if (!getArmIndex(end_effector_link, arm))
      {
        return false;
      }

      if (chain_[arm].getNrOfJoints() != qdot.rows())
      {
        ROS_ERROR("Joint chain for eef %s has a different number of joints than the provided", end_effector_link.c_str());
        return false;
      }

      int joint_index = 0;

      for (unsigned long i = 0; i < state.name.size(); i++)
      {
        if (hasJoint(chain_[arm], state.name[i]))
        {
          state.position[i] = q[joint_index];
          state.velocity[i] = qdot[joint_index];
          joint_index++;
        }
      }

      if (joint_index != chain_[arm].getNrOfJoints())
      {
        ROS_ERROR("Could not find all the joints in the provided joint state message");
        return false;
      }

      return true;
    }

    bool KDLManager::getGrippingPoint(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Frame &out) const
    {
      int arm;

      if (!getArmIndex(end_effector_link, arm))
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

    bool KDLManager::getGrippingTwist(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Twist &out) const
    {
      int arm;

      if (!getArmIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::FrameVel eef_twist;
      if (!getEefTwist(end_effector_link, state, eef_twist))
      {
        return false;
      }

      out = eef_to_gripping_point_[arm].Inverse()*eef_twist.GetTwist();
      return true;
    }

    bool KDLManager::getEefPose(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Frame &out) const
    {
      int arm;

      if (!getArmIndex(end_effector_link, arm))
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

      if (!getArmIndex(end_effector_link, arm))
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

    bool KDLManager::getPoseIK(const std::string &end_effector_link, const sensor_msgs::JointState &state, const KDL::Frame &in, KDL::JntArray &out) const
    {
      int arm;

      if (!getArmIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::JntArray positions(chain_[arm].getNrOfJoints());
      KDL::JntArrayVel velocities(chain_[arm].getNrOfJoints());
      if (!getChainJointState(state, arm, positions, velocities))
      {
        return false;
      }

      ikpos_[arm]->CartToJnt(positions, in, out);
      return true;
    }

    bool KDLManager::getVelIK(const std::string &end_effector_link, const sensor_msgs::JointState &state, const KDL::Twist &in, KDL::JntArray &out) const
    {
      int arm;

      if (!getArmIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::JntArray positions(chain_[arm].getNrOfJoints());
      KDL::JntArrayVel velocities(chain_[arm].getNrOfJoints());
      if (!getChainJointState(state, arm, positions, velocities))
      {
        return false;
      }

      ikvel_[arm]->CartToJnt(positions, in, out);
      return true;
    }

    bool KDLManager::getJacobian(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Jacobian &out) const
    {
      int arm;

      if (!getArmIndex(end_effector_link, arm))
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
}
