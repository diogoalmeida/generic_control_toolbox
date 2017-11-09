#ifndef __MARKER_MANAGER__
#define __MARKER_MANAGER__

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace generic_control_toolbox
{
  enum MarkerType {sphere, arrow};
  /**
    Maintains a set of common resources for filling in a MarkerArray msg
  **/
  class MarkerManager
  {
  public:
    MarkerManager();
    ~MarkerManager();

    /**
      Initializes a marker in the marker array with default color
      and scale.

      @param marker_name Name for indexing purposes.
      @param ns the namespace of the new marker.
      @param frame_id The frame on which the marker is expressed.
      @param type The marker type.
      @return False is marker_name is already added.
    **/
    bool addMarker(const std::string &marker_name, const std::string &ns, const std::string &frame_id, MarkerType type);

    /**
      Sets the indexed marker color.

      @param marker_name Name for indexing purposes.
      @return False if marker_name is not found.
    **/
    bool setMarkerColor(const std::string &marker_name, double r, double g, double b);

    /**
      Sets the marker scale.

      @param marker_name Name for indexing purposes.
      @return false if marker_name is not found.
    **/
    bool setMarkerScale(const std::string &marker_name, double x, double y, double z);

    /**
      Fills a marker with the given initial and end point. Clears existing points.

      @param marker_name Name for indexing purposes.
      @param initial_point Initial point of the marker
      @param final_point Initial point of the marker
      @return False if marker_name is not found or if marker type does not allow to set points.
    **/
    bool setMarkerPoints(const std::string &marker_name, const Eigen::Vector3d &initial_point, const Eigen::Vector3d &final_point);

    /**
      Fills a marker with the given pose.

      @param marker_name Name for indexing purposes.
      @param pose The marker pose.
      @return False if marker_name is not found or if marker type does not allow to set pose.
    **/
    bool setMarkerPose(const std::string &marker_name, const Eigen::Affine3d &pose);
  private:
    std::map<std::string, int> marker_map_;
    visualization_msgs::MarkerArray marker_array_;

    /**
      Returns the marker id indexed by the marker_name

      @return False if marker not found.
    **/
    bool getMarkerId(const std::string &marker_name, int &id) const;
  };
}
#endif
