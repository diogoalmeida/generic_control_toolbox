#ifndef __FOLDING_UTILS__
#define __FOLDING_UTILS__

#include <math.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <stdexcept>

namespace generic_control_toolbox
{
class MatrixParser
{
 public:
  MatrixParser();
  ~MatrixParser();

  /**
    Initialize a nxn matrix with values obtained from the ros parameter
    server.

    @param M The matrix to be initialized
    @param param_name The parameter server location
    @param n The ros nodehandle used to query the parameter server

    @throw logic_error in case vals does not have square dimensions.
    @return True for success, False if parameter is not available.
  **/
  static bool parseMatrixData(Eigen::MatrixXd &M, const std::string param_name,
                              const ros::NodeHandle &n);

  /**
    Computed the skew-symmetric matrix of a 3-dimensional vector.

    @param v The 3-dimensional vector
    @return The skew-symmetric matrix
  **/
  static Eigen::Matrix3d computeSkewSymmetric(const Eigen::Vector3d &v);

 private:
  /**
    Fill in a nxn matrix with the given values.

    @param M The matrix to be filled in. Will be set to the size nxn.
    @param vals A vector with the values to fill in
    @throw logic_error in case vals does not have square dimensions.
  **/
  static void initializeEigenMatrix(Eigen::MatrixXd &M,
                                    const std::vector<double> &vals);

  /**
    Fill in a rows x cols matrix with the given values.

    @param M The matrix to be filled in. Will be set to the size nxn.
    @param vals A vector with the values to fill in
    @param rows The number of rows in the matrix.
    @param cols The number of cols.
    @throw logic_error in case the rows or cols have invalid numbers.
   **/
  static void initializeEigenMatrix(Eigen::MatrixXd &M,
                                    const std::vector<double> &vals, int rows,
                                    int cols);
};
}  // namespace generic_control_toolbox

#endif
