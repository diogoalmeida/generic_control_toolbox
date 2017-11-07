#include <generic_control_toolbox/matrix_parser.hpp>

namespace generic_control_toolbox
{
  MatrixParser::MatrixParser(){}
  MatrixParser::~MatrixParser() {}

  bool MatrixParser::parseMatrixData(Eigen::MatrixXd &M, const std::string param_name, const ros::NodeHandle &n) const
  {
    std::vector<double> vals;
    if(n.hasParam(param_name.c_str()))
    {
      if(n.hasParam((param_name + std::string("/data").c_str())))
      {
        n.getParam((param_name + std::string("/data")).c_str(), vals);
        initializeEigenMatrix(M, vals);
      }
      else
      {
        ROS_ERROR_STREAM("MatrixParser: Matrix definition " << param_name << " has no data values (" << param_name << "/data)! Shutting down...");
        return false;
      }
    }
    else
    {
      ROS_ERROR_STREAM("MatrixParser: Configuration name " << param_name << " does not exist");
      return false;
    }

    return true;
  }

  void MatrixParser::initializeEigenMatrix(Eigen::MatrixXd &M, const std::vector<double> &vals) const
  {
    double size_f, frac_part, discard;
    int size;

    size_f = std::sqrt(vals.size());
    frac_part = std::modf(size_f, &discard);

    size = (int) size_f;
    if (frac_part != 0.0)
    {
      std::stringstream errMsg;
      errMsg << "MatrixParser: Tried to initialize a square matrix with a non-square (got: " << vals.size() << ") number of values";
      throw std::logic_error(errMsg.str().c_str());
    }

    ROS_DEBUG("MatrixParser: filling matrix");

    M = Eigen::MatrixXd(size, size);

    for(int i = 0; i < size; i++)
    {
      for(int j = 0; j < size; j++)
      {
        M(i,j) = vals[i*size + j];
      }
    }
  }

  Eigen::Matrix3d MatrixParser::computeSkewSymmetric(const Eigen::Vector3d &v) const
  {
    Eigen::Matrix3d S;

    S << 0,    -v(2),  v(1),
         v(2),  0   , -v(0),
        -v(1),  v(0),  0;

    return S;
  }
}
