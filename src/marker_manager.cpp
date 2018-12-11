#include <generic_control_toolbox/marker_manager.hpp>

namespace generic_control_toolbox
{
MarkerManager::MarkerManager() { n_ = ros::NodeHandle("~"); }

MarkerManager::~MarkerManager() {}

bool MarkerManager::addMarkerGroup(const std::string &group_key,
                                   const std::string &topic_name)
{
  if (marker_map_.find(group_key) != marker_map_.end())
  {
    ROS_ERROR_STREAM("MarkerManager: Tried to add already existing group "
                     << group_key);
    return false;
  }

  std::shared_ptr<
      realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray> >
      rt_pub(new realtime_tools::RealtimePublisher<
             visualization_msgs::MarkerArray>(n_, topic_name, 1));
  std::map<std::string, int> m;

  marker_pub_[group_key] = rt_pub;
  marker_map_[group_key] = m;
  marker_array_[group_key] = visualization_msgs::MarkerArray();
  return true;
}

bool MarkerManager::addMarker(const std::string &group_key,
                              const std::string &marker_name,
                              const std::string &ns,
                              const std::string &frame_id, MarkerType type)
{
  if (marker_map_.find(group_key) == marker_map_.end())
  {
    return false;
  }

  int id;
  if (getMarkerId(group_key, marker_name, id))
  {
    ROS_ERROR_STREAM("MarkerManager: Tried to add marker "
                     << marker_name << " that is already on the marker array");
    return false;
  }

  visualization_msgs::Marker new_marker;
  new_marker.header.frame_id = frame_id;
  new_marker.header.stamp = ros::Time();
  new_marker.ns = ns;

  if (type == sphere)
  {
    new_marker.type = new_marker.SPHERE;
  }
  else
  {
    new_marker.type = new_marker.ARROW;
  }

  new_marker.action = new_marker.ADD;
  new_marker.scale.x = 0.01;
  new_marker.scale.y = 0.01;
  new_marker.scale.z = 0.01;
  new_marker.lifetime = ros::Duration(0);
  new_marker.frame_locked = false;
  new_marker.color.r = 1.0;
  new_marker.color.a = 1.0;
  new_marker.pose.orientation.w = 1.0;

  int max_id = -1;
  for (auto const &entry : marker_map_.at(group_key))
  {
    if (entry.second > max_id)
    {
      max_id = entry.second;
    }
  }

  new_marker.id = max_id + 1;

  marker_map_.at(group_key)[marker_name] = max_id + 1;
  marker_array_.at(group_key).markers.push_back(new_marker);

  return true;
}

bool MarkerManager::setMarkerColor(const std::string &group_key,
                                   const std::string &marker_name, double r,
                                   double g, double b)
{
  if (marker_map_.find(group_key) == marker_map_.end())
  {
    return false;
  }

  int id;
  if (!getMarkerId(group_key, marker_name, id))
  {
    return false;
  }

  marker_array_.at(group_key).markers[id].color.r = r;
  marker_array_.at(group_key).markers[id].color.g = g;
  marker_array_.at(group_key).markers[id].color.b = b;
  return true;
}

bool MarkerManager::setMarkerScale(const std::string &group_key,
                                   const std::string &marker_name, double x,
                                   double y, double z)
{
  if (marker_map_.find(group_key) == marker_map_.end())
  {
    return false;
  }

  int id;
  if (!getMarkerId(group_key, marker_name, id))
  {
    return false;
  }

  marker_array_.at(group_key).markers[id].scale.x = x;
  marker_array_.at(group_key).markers[id].scale.y = y;
  marker_array_.at(group_key).markers[id].scale.z = z;
  return true;
}

bool MarkerManager::setMarkerPoints(const std::string &group_key,
                                    const std::string &marker_name,
                                    const Eigen::Vector3d &initial_point,
                                    const Eigen::Vector3d &final_point)
{
  if (marker_map_.find(group_key) == marker_map_.end())
  {
    return false;
  }

  int id;
  if (!getMarkerId(group_key, marker_name, id))
  {
    return false;
  }

  geometry_msgs::Point point;
  marker_array_.at(group_key).markers[id].points.clear();
  tf::pointEigenToMsg(initial_point, point);
  marker_array_.at(group_key).markers[id].points.push_back(point);
  tf::pointEigenToMsg(final_point, point);
  marker_array_.at(group_key).markers[id].points.push_back(point);
  return true;
}

bool MarkerManager::setMarkerPose(const std::string &group_key,
                                  const std::string &marker_name,
                                  const Eigen::Affine3d &pose)
{
  if (marker_map_.find(group_key) == marker_map_.end())
  {
    return false;
  }

  int id;
  if (!getMarkerId(group_key, marker_name, id))
  {
    return false;
  }

  tf::poseEigenToMsg(pose, marker_array_.at(group_key).markers[id].pose);
  return true;
}

void MarkerManager::publishMarkers()
{
  for (auto const &publisher : marker_pub_)
  {
    if (publisher.second->trylock())
    {
      publisher.second->msg_ = marker_array_.at(publisher.first);
      publisher.second->unlockAndPublish();
    }
  }
}

bool MarkerManager::getMarkerId(const std::string &group_key,
                                const std::string &marker_name, int &id) const
{
  if (marker_map_.find(group_key) == marker_map_.end())
  {
    return false;
  }

  for (auto const &it : marker_map_.at(group_key))
  {
    if (it.first == marker_name)
    {
      for (unsigned long i = 0; i < marker_array_.at(group_key).markers.size();
           i++)
      {
        if (marker_array_.at(group_key).markers[i].id == it.second)
        {
          id = it.second;
          return true;
        }
      }
    }
  }

  ROS_ERROR_STREAM("MarkerManager: Marker name " << marker_name
                                                 << " not found");
  return false;
}
}  // namespace generic_control_toolbox
