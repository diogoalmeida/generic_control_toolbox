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
    }

    KDLManager::~KDLManager() {}

    bool KDLManager::initializeArm(const std::string &end_effector_link)
    {
      int a;
      if(getArmIndex(end_effector_link, a))
      {
        ROS_ERROR("Tried to initialize arm %s, but it was already initialized", end_effector_link.c_str());
        return false;
      }

      KDL::Tree tree;
      KDL::Joint kdl_joint;
      KDL::Chain chain;
      kdl_parser::treeFromUrdfModel(model_, tree); // convert URDF description of the robot into a KDL tree
      if(!tree.getChain(chain_base_link_, end_effector_link, chain))
      {
        ROS_ERROR("Failed to find chain <%s, %s> in the kinematic tree", chain_base_link_.c_str(), end_effector_link.c_str());
        return false;
      }
      // joint_positions.resize(chain.getNrOfJoints());
      // joint_velocities.q.resize(chain.getNrOfJoints());
      // joint_velocities.qdot.resize(chain.getNrOfJoints());

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
      jac_solver_.push_back(std::shared_ptr<KDL::ChainJntToJacSolver>(new KDL::ChainJntToJacSolver(chain)));

      return true;
    }

    bool KDLManager::getEefPose(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Frame &out)
    {
      int arm;

      if (!getArmIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::JntArray positions;
      KDL::JntArrayVel velocities;
      if (!getChainJointState(state, chain_[arm], positions, velocities))
      {
        return false;
      }

      fkpos_[arm]->JntToCart(positions, out);
      return true;
    }

    bool KDLManager::getEefTwist(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::FrameVel &out)
    {
      int arm;

      if (!getArmIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::JntArray positions;
      KDL::JntArrayVel velocities;
      if (!getChainJointState(state, chain_[arm], positions, velocities))
      {
        return false;
      }

      fkvel_[arm]->JntToCart(velocities, out);
      return true;
    }

    bool KDLManager::getPoseIK(const std::string &end_effector_link, const sensor_msgs::JointState &state, const KDL::Frame &in, KDL::JntArray &out)
    {
      int arm;

      if (!getArmIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::JntArray positions;
      KDL::JntArrayVel velocities;
      if (!getChainJointState(state, chain_[arm], positions, velocities))
      {
        return false;
      }

      ikpos_[arm]->CartToJnt(positions, in, out);
      return true;
    }

    bool KDLManager::getVelIK(const std::string &end_effector_link, const sensor_msgs::JointState &state, const KDL::Twist &in, KDL::JntArray &out)
    {
      int arm;

      if (!getArmIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::JntArray positions;
      KDL::JntArrayVel velocities;
      if (!getChainJointState(state, chain_[arm], positions, velocities))
      {
        return false;
      }

      ikvel_[arm]->CartToJnt(positions, in, out);
      return true;
    }

    bool KDLManager::getJacobian(const std::string &end_effector_link, const sensor_msgs::JointState &state, KDL::Jacobian &out)
    {
      int arm;

      if (!getArmIndex(end_effector_link, arm))
      {
        return false;
      }

      KDL::JntArray positions;
      KDL::JntArrayVel velocities;
      if (!getChainJointState(state, chain_[arm], positions, velocities))
      {
        return false;
      }

      jac_solver_[arm]->JntToJac(positions, out);
      return true;
    }

    bool KDLManager::getChainJointState(const sensor_msgs::JointState &current_state, const KDL::Chain &chain, KDL::JntArray &positions, KDL::JntArrayVel &velocities)
    {
      unsigned int processed_joints = 0;
      for (unsigned long i = 0; i < current_state.name.size(); i++)
      {
        if (hasJoint(chain, current_state.name[i]))
        {
          positions(processed_joints) = current_state.position[i];
          velocities.q(processed_joints) = current_state.position[i];
          velocities.qdot(processed_joints) = current_state.velocity[i];
          processed_joints++;
        }
      }

      if (processed_joints != chain.getNrOfJoints())
      {
        ROS_ERROR("Failed to acquire chain joint state");
        return false;
      }

      return true;
    }

    bool KDLManager::hasJoint(const KDL::Chain &chain, const std::string &joint_name)
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
