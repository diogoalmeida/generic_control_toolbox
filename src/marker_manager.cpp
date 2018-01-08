#include <generic_control_toolbox/marker_manager.hpp>

namespace generic_control_toolbox
{
  MarkerManager::MarkerManager()
  {
    n_ = ros::NodeHandle("~");
  }

  MarkerManager::~MarkerManager(){}

  bool MarkerManager::addMarkerGroup(const std::string &group_key, const std::string &topic_name)
  {
    int group = -1;

    if (getIndex(group_key, group))
    {
      ROS_ERROR_STREAM("MarkerManager: Tried to add already existing group " << group_key);
      return false;
    }

    manager_index_.push_back(group_key);
    std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray> > rt_pub(new realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray>(n_, topic_name, 1));
    std::map<std::string, int> m;

    marker_pub_.push_back(rt_pub);
    marker_map_.push_back(m);
    marker_array_.push_back(visualization_msgs::MarkerArray());
    return true;
  }

  bool MarkerManager::addMarker(const std::string &group_key, const std::string &marker_name, const std::string &ns, const std::string &frame_id, MarkerType type)
  {
    int group_id, id;

    if (!getIndex(group_key, group_id))
    {
      return false;
    }

    if (getMarkerId(group_key, marker_name, id))
    {
      ROS_ERROR_STREAM("MarkerManager: Tried to add marker " << marker_name << " that is already on the marker array");
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
    for (auto const &entry : marker_map_[group_id])
    {
      if (entry.second > max_id)
      {
        max_id = entry.second;
      }
    }

    new_marker.id = max_id + 1;

    marker_map_[group_id][marker_name] = max_id + 1;
    marker_array_[group_id].markers.push_back(new_marker);

    return true;
  }

  bool MarkerManager::setMarkerColor(const std::string &group_key, const std::string &marker_name, double r, double g, double b)
  {
    int id, group_id;

    if (!getIndex(group_key, group_id))
    {
      return false;
    }

    if (!getMarkerId(group_key, marker_name, id))
    {
      return false;
    }

    marker_array_[group_id].markers[id].color.r = r;
    marker_array_[group_id].markers[id].color.g = g;
    marker_array_[group_id].markers[id].color.b = b;
    return true;
  }

  bool MarkerManager::setMarkerScale(const std::string &group_key, const std::string &marker_name, double x, double y, double z)
  {
    int id, group_id;

    if (!getIndex(group_key, group_id))
    {
      return false;
    }

    if (!getMarkerId(group_key, marker_name, id))
    {
      return false;
    }

    marker_array_[group_id].markers[id].scale.x = x;
    marker_array_[group_id].markers[id].scale.y = y;
    marker_array_[group_id].markers[id].scale.z = z;
    return true;
  }

  bool MarkerManager::setMarkerPoints(const std::string &group_key, const std::string &marker_name, const Eigen::Vector3d &initial_point, const Eigen::Vector3d &final_point)
  {
    int id, group_id;

    if (!getIndex(group_key, group_id))
    {
      return false;
    }

    if (!getMarkerId(group_key, marker_name, id))
    {
      return false;
    }

    geometry_msgs::Point point;
    marker_array_[group_id].markers[id].points.clear();
    tf::pointEigenToMsg(initial_point, point);
    marker_array_[group_id].markers[id].points.push_back(point);
    tf::pointEigenToMsg(final_point, point);
    marker_array_[group_id].markers[id].points.push_back(point);
    return true;
  }

  bool MarkerManager::setMarkerPose(const std::string &group_key, const std::string &marker_name, const Eigen::Affine3d &pose)
  {
    int id, group_id;

    if (!getIndex(group_key, group_id))
    {
      return false;
    }

    if (!getMarkerId(group_key, marker_name, id))
    {
      return false;
    }

    tf::poseEigenToMsg(pose, marker_array_[group_id].markers[id].pose);
    return true;
  }

  void MarkerManager::publishMarkers()
  {
    for (unsigned int i = 0; i < marker_pub_.size(); i ++)
    {
      if (marker_pub_[i]->trylock())
      {
        marker_pub_[i]->msg_ = marker_array_[i];
        marker_pub_[i]->unlockAndPublish();
      }
    }
  }

  bool MarkerManager::getMarkerId(const std::string &group_key, const std::string &marker_name, int &id) const
  {
    int group_id;

    if (!getIndex(group_key, group_id))
    {
      return false;
    }

    for (auto const &it : marker_map_[group_id])
    {
      if (it.first == marker_name)
      {
        for (unsigned long i = 0; i < marker_array_[group_id].markers.size(); i++)
        {
          if (marker_array_[group_id].markers[i].id == it.second)
          {
            id = it.second;
            return true;
          }
        }
      }
    }

    ROS_ERROR_STREAM("MarkerManager: Marker name " << marker_name << " not found");
    return false;
  }
}
