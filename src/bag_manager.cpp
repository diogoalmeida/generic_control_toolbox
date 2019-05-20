#include <generic_control_toolbox/bag_manager.hpp>

namespace generic_control_toolbox
{
BagManager::BagManager(const std::string &path,
                       const std::string &default_topic)
    : default_topic_(default_topic)
{
  int num = numOfFiles(path);
  std::string name = path + "_" + std::to_string(num);
  ROS_DEBUG_STREAM("Opening a new bag: " << name);
  bag_.open(name, rosbag::bagmode::Write);
}

BagManager::~BagManager() { bag_.close(); }

template <class T>
void BagManager::write(const T &msg)
{
  bag_.write(default_topic_, ros::Time::now(), msg);
}

template <class T>
void BagManager::write(const std::string &topic, const T &msg)
{
  bag_.write(topic, ros::Time::now(), msg);
}

int BagManager::numOfFiles(const std::string &path) const
{
  // from http://www.cplusplus.com/forum/beginner/70854/
  boost::filesystem::path p(path);
  boost::filesystem::directory_iterator end_iter;
  int num = 0;

  for (boost::filesystem::directory_iterator iter(p); iter != end_iter; iter++)
  {
    if (iter->path().extension() == ".bag")
    {
      num++;
    }
  }

  return num;
}
}  // namespace generic_control_toolbox
