#include <gtest/gtest.h>
#include <ros/ros.h>
#include <generic_control_toolbox/joint_position_integrator.hpp>

bool assertMatrixEquality(const Eigen::MatrixXd &m1, const Eigen::MatrixXd &m2)
{
  for (int j = 0; j < m1.cols(); ++j)
  {
    for (int i = 0; i < m1.rows(); ++i)
    {
      EXPECT_NEAR(m2(i, j), m1(i, j), 1e-7);
    }
  }
}

TEST(TestJointPositionIntegrator, TestParameterLoading)
{
  ros::NodeHandle nh("~");
  ASSERT_NO_THROW(
      generic_control_toolbox::JointPositionIntegrator integrator(nh));
}

TEST(TestJointPositionIntegrator, TestIntegrationSingleValue)
{
  ros::NodeHandle nh("~");
  generic_control_toolbox::JointPositionIntegrator integrator(nh);
  KDL::JntArray q(1), q_dot(1), q_up(1);
  float dt = 0.1;
  q_dot.data << 1.0;
  q_up = integrator.update(q_dot, q, dt);
  assertMatrixEquality(q_up.data, q.data + q_dot.data * dt);
}

TEST(TestJointPositionIntegrator, TestIntegrationMultipleValues)
{
  ros::NodeHandle nh("~");
  generic_control_toolbox::JointPositionIntegrator integrator(nh);
  KDL::JntArray q(14), q_dot(14), q_up(14);
  q_dot.data << 0.01 * Eigen::VectorXd::Ones(14);
  float dt = 0.1;
  int num_integration = 10;
  for (unsigned int i = 0; i < num_integration; i++)
  {
    q_up = integrator.update(q_dot, q, dt);
  }
  assertMatrixEquality(q_up.data, q.data + num_integration * q_dot.data * dt);
}

TEST(TestJointPositionIntegrator, TestReset)
{
  ros::NodeHandle nh("~");
  generic_control_toolbox::JointPositionIntegrator integrator(nh);
  KDL::JntArray q(14), q_prev(14), q_dot(14), q_up(14);
  q_dot.data << Eigen::VectorXd::Ones(14);
  float dt = 0.1;
  q_up = integrator.update(q_dot, q, dt);
  assertMatrixEquality(q_up.data, q.data + q_dot.data * dt);
  q_prev = q;
  q = q_up;
  integrator.reset();
  q_up = integrator.update(q_dot, q_prev, dt);
  assertMatrixEquality(q_up.data, q_prev.data + q_dot.data * dt);
  integrator.reset();
  q_up = integrator.update(q_dot, q, dt);  // after a reset, we take q
  assertMatrixEquality(q_up.data, q.data + q_dot.data * dt);
}

TEST(TestJointPositionIntegrator, TestSaturatedIntegration)
{
  ros::NodeHandle nh("~");
  double max_error = 0;
  float dt = 0.1;
  generic_control_toolbox::JointPositionIntegrator integrator(nh);
  ASSERT_TRUE(nh.getParam("max_joint_error", max_error));
  KDL::JntArray q(1), q_dot(1), q_up(1);
  q_dot.data << (max_error + 0.0001) / dt;
  q_up = integrator.update(q_dot, q, dt);
  assertMatrixEquality(q_up.data, q.data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_test_node");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}