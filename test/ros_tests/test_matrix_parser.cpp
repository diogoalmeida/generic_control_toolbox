#include <gtest/gtest.h>
#include <ros/ros.h>
#include <generic_control_toolbox/matrix_parser.hpp>

void assertMatrixEqual(const Eigen::MatrixXd &m1, const Eigen::MatrixXd &m2)
{
  ASSERT_EQ(m1.cols(), m2.cols());
  ASSERT_EQ(m1.rows(), m2.rows());

  for (unsigned int i = 0; i < m1.rows(); i++)
  {
    for (unsigned int j = 0; j < m1.cols(); j++)
    {
      ASSERT_NEAR(m1(i, j), m2(i, j), DBL_EPSILON);
    }
  }
}

TEST(TestMatrixParser, TestParseSquareMatrix)
{
  ros::NodeHandle nh("~");
  Eigen::MatrixXd square_m;
  Eigen::Matrix3d expected_m;

  expected_m << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

  ASSERT_TRUE(generic_control_toolbox::MatrixParser::parseMatrixData(
      square_m, "square_matrix", nh));

  ASSERT_EQ(square_m.cols(), square_m.rows());
  ASSERT_EQ(square_m.rows(), 3);
  assertMatrixEqual(square_m, expected_m);
}

TEST(TestMatrixParser, TestParseRectangularMatrix)
{
  ros::NodeHandle nh("~");
  Eigen::MatrixXd rectangular_m;
  Eigen::Matrix<double, 3, 2> expected_m;

  expected_m << 1, 2, 3, 4, 5, 6;

  ASSERT_TRUE(generic_control_toolbox::MatrixParser::parseMatrixData(
      rectangular_m, "rectangular_matrix", nh));

  ASSERT_EQ(rectangular_m.rows(), 3);
  ASSERT_EQ(rectangular_m.cols(), 2);
  assertMatrixEqual(rectangular_m, expected_m);
}

TEST(TestMatrixParser, TestVectorParse)
{
  ros::NodeHandle nh("~");
  Eigen::MatrixXd vector_m;
  Eigen::Vector3d expected_m;

  expected_m << 1, 2, 3;

  ASSERT_TRUE(generic_control_toolbox::MatrixParser::parseMatrixData(
      vector_m, "vector", nh));

  ASSERT_EQ(vector_m.rows(), 3);
  ASSERT_EQ(vector_m.cols(), 1);
  assertMatrixEqual(vector_m, expected_m);
}

TEST(TestMatrixParser, TestFailParse)
{
  ros::NodeHandle nh("~");
  Eigen::MatrixXd fail_m;

  ASSERT_FALSE(generic_control_toolbox::MatrixParser::parseMatrixData(
      fail_m, "non_existant_ns", nh));
}

TEST(TestMatrixParser, TestComputeSkew) { ros::NodeHandle nh("~"); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_test_node");
  ros::start();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}