#include <generic_control_toolbox/marker_manager.hpp>

namespace generic_control_toolbox
{
  MarkerManager::MarkerManager(){}
  MarkerManager::~MarkerManager(){}

  bool MarkerManager::addMarker(const std::string &marker_name, const std::string &ns, const std::string &frame_id, MarkerType type)
  {
    int id;
    if (getMarkerId(marker_name, id))
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

    int max_id = 0;
    for (auto const &entry : marker_map_)
    {
      if (entry.second > max_id)
      {
        max_id = entry.second;
      }
    }

    new_marker.id = max_id;

    marker_map_[marker_name] = max_id;
    marker_array_.markers.push_back(new_marker);

    return true;
  }

  bool MarkerManager::setMarkerColor(const std::string &marker_name, double r, double g, double b)
  {
    int id;

    if (!getMarkerId(marker_name, id))
    {
      return false;
    }

    marker_array_.markers[id].color.r = r;
    marker_array_.markers[id].color.g = g;
    marker_array_.markers[id].color.b = b;
    return true;
  }

  bool MarkerManager::setMarkerScale(const std::string &marker_name, double x, double y, double z)
  {
    int id;

    if (!getMarkerId(marker_name, id))
    {
      return false;
    }

    marker_array_.markers[id].scale.x = x;
    marker_array_.markers[id].scale.y = y;
    marker_array_.markers[id].scale.z = z;
    return true;
  }

  bool MarkerManager::setMarkerPoints(const std::string &marker_name, const Eigen::Vector3d &initial_point, const Eigen::Vector3d &final_point)
  {
    int id;

    if (!getMarkerId(marker_name, id))
    {
      return false;
    }

    geometry_msgs::Point point;
    marker_array_.markers[id].points.clear();
    tf::pointEigenToMsg(initial_point, point);
    marker_array_.markers[id].points.push_back(point);
    tf::pointEigenToMsg(final_point, point);
    marker_array_.markers[id].points.push_back(point);
    return true;
  }

  bool MarkerManager::setMarkerPose(const std::string &marker_name, const Eigen::Affine3d &pose)
  {
    int id;

    if (!getMarkerId(marker_name, id))
    {
      return false;
    }

    tf::poseEigenToMsg(pose, marker_array_.markers[id].pose);
    return true;
  }

  bool MarkerManager::getMarkerId(const std::string &marker_name, int &id) const
  {
    for (auto const &it : marker_map_)
    {
      if (it.first == marker_name)
      {
        for (unsigned long i = 0; i < marker_array_.markers.size(); i++)
        {
          if (marker_array_.markers[i].id == it.second)
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
