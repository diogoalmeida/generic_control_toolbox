#ifndef __BAG_MANAGER__
#define __BAG_MANAGER__

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <boost/filesystem.hpp>

namespace generic_control_toolbox
{
/**
  Produces a rosbag with given message data.
**/
class BagManager
{
 public:
  /**
    Open a bag on the given path.

    @param path The path where to record the bag.
    @param default_topic The default topic name for the bag to record.
  **/
  BagManager(const std::string &path, const std::string &default_topic);
  ~BagManager();

  /**
    Add a message to the bag file on the default topic.

    @param msg The message to be written.
  **/
  template <class T>
  void write(const T &msg);

  /**
    Add a message to the bag file on a given topic.

    @param topic The topic name.
    @param msg The message to be written.
  **/
  template <class T>
  void write(const std::string &topic, const T &msg);

 private:
  rosbag::Bag bag_;
  std::string default_topic_;

  /**
    Returns the number of bag files in the recording directory.

    @param path Path to the recording directory.
    @return Number of files of extension "*.bag" in the directory.
  **/
  int numOfFiles(const std::string &path) const;
};

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
}  // namespace generic_control_toolbox

#endif
