#ifndef MAP_SERVER_2D_H_
#define MAP_SERVER_2D_H_

#include <boost/assign.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h> 
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <occupancy_grid_utils/combine_grids.h>

#include <lvr_tools/Mesh.h>
#include <lvr_tools/BoundingBox.h>
#include <lvr_tools/GetSingleSliceGrid.h>
#include <lvr_tools/GetMultiSliceGrid.h>
#include <postgis_control/Floor.h>
#include <postgis_control/GetRegionBBoxes.h>
#include <postgis_control/GetRegionFaces.h>
#include <postgis_control/GetRoomFaces.h>
#include <postgis_control/GetStructure.h>
#include <postgis_control/GetRoomBBoxes.h>
#include <postgis_control/GetBBoxMarker.h>
#include <postgis_control/GetBBoxOfBBoxes.h>
#include <map_server_2d/GetLocalizationMap.h>
#include <map_server_2d/GetNavigationMap.h>
#include <map_server_2d/GetGlobalLocalizationMap.h>
#include <map_server_2d/GetGlobalNavigationMap.h>

using namespace std;
using namespace occupancy_grid_utils;

namespace map_server_2d
{

typedef boost::shared_ptr<nav_msgs::OccupancyGrid> GridPtr;
typedef boost::shared_ptr<nav_msgs::OccupancyGrid const> GridConstPtr;

	/**
	 * @brief  The MapServer2D interfaces between the semantic map and the robot navigation methods by turning the environments geometry and semantics into 2d grid maps used by the
	 * standard components of the navigation stack.
	 */

class MapServer2D
{
private:
	ros::NodeHandle nh;

	/// internal parameters
	string map_frame;
	string dimension;
	double single_slice_value;
	double multi_slice_min_value;
	double multi_slice_max_value;
	double multi_slice_resolution;
	double grid_resolution;
	double grid_unoccupied_default;
	double default_padding;
	vector<string> mesh_region_names;
	vector<double> mesh_region_padding;
	vector<string> blocked_region_names;
	vector<double> blocked_region_padding;	
	XmlRpc::XmlRpcValue blocked_regions;
	XmlRpc::XmlRpcValue mesh_regions;

	/// service clients
	ros::ServiceClient get_region_bboxes_client_;
	ros::ServiceClient get_region_faces_client_;
	ros::ServiceClient get_room_faces_client_;
	ros::ServiceClient get_structure_client_;
	ros::ServiceClient get_single_slice_grid_client_;
	ros::ServiceClient get_multi_slice_grid_client_;
	ros::ServiceClient get_bbox_marker_client_;
	ros::ServiceClient get_bbox_of_bboxes_client_;
	ros::ServiceClient get_room_bboxes_client_;

	/// service servers
	ros::ServiceServer get_localization_map_srv_;
	ros::ServiceServer get_navigation_map_srv_;
	ros::ServiceServer get_global_localization_map_srv_;
	ros::ServiceServer get_global_navigation_map_srv_;
	ros::ServiceServer publish_localization_map_srv_;
	ros::ServiceServer publish_navigation_map_srv_;

	/// publishers
	ros::Publisher navmap_pub_;
	ros::Publisher locmap_pub_;
	ros::Publisher navmap_marker_pub_;
	ros::Publisher locmap_marker_pub_;

public:
	/**
	 * @brief Constructor
	 */
	MapServer2D(ros::NodeHandle n);

	/**
	 * @brief Destructor
	 */
	~MapServer2D() { }

	/**
	 * @brief method to extract all parameters from the parameter server
	 * @return status if setup was successfull
	 */
	bool setupParameters();

	/**
	 * @brief method to connect all services required to implement the node's functionality
	 * @return status if setup was successfull
	 */
	bool setupServices();

	/**
	 * @brief service method to create a grid map from a set of blocked regions
	 *
	 * @param bboxes set of the bounding boxes that need to be blocked during the grid genereation process
	 * @param frame for the resulting grid map
	 * @param padding for the areas
	 * @return grid map that contains all blocked regions
	 */
	nav_msgs::OccupancyGrid fGetBlockedAreasGrid(vector<lvr_tools::BoundingBox> bboxes, string frame, double padding);

	/**
	 * @brief method to create a occupancy grid from a set of bounding boxes
	 *
	 * @param frame that the map should be relative to
	 * @param time stamp
	 * @param bboxes bouding boxes of blocked areas
	 * @param points that define the intersection segments created by slicing the mesh
	 * @param resolution of the resulting map to convert from the continouus to the discrete
	 * @param unoccupied_default value to be used for potentially free space
	 * @return nav_msgs::OccupancyGrid that can be used for robot navigation and localization
	 */
	nav_msgs::OccupancyGrid createOccupancyGrid(string frame, ros::Time time, lvr_tools::BoundingBox bboxes, double resolution, double unoccupied_default, double padding);

	/**
	 * @brief implementation of bresenham's alogrithm to draw section segments onto the grid maps
	 *
	 * @param xa of point a
	 * @param ya of point a 
	 * @param xb of point b
	 * @param yb of point b 
	 * @param data array that contains the grid values and will be written with the segments
	 * @param width stepsize of the array
	 */
	void drawLineBresenham(int x1, int y1, int x2, int y2, vector<signed char>& data, int width);

	/**
	 * @brief service method to create a localization map from the semantic map
	 *
	 * @param request of the map_server_2d::GetLocalizationMap ROS Service, see definition for details 
	 * @param response of the map_server_2d::GetLocalizationMap ROS Service, see definition for details
	 */
	bool srvGetLocalizationMap(map_server_2d::GetLocalizationMap::Request& request,  map_server_2d::GetLocalizationMap::Response& response);

	/**
	 * @brief service method to create a navigation map from the semantic map
	 *
	 * @param request of the map_server_2d::GetNavigationMap ROS Service, see definition for details 
	 * @param response of the map_server_2d::GetNavigationMap ROS Service, see definition for details
	 */
	bool srvGetNavigationMap(map_server_2d::GetNavigationMap::Request& request,  map_server_2d::GetNavigationMap::Response& response);

	/**
	 * @brief service method to create a gloal localization map from the semantic map
	 *
	 * @param request of the map_server_2d::GetGlobalLocalizationMap ROS Service, see definition for details 
	 * @param response of the map_server_2d::GetGlobalLocalizationMap ROS Service, see definition for details
	 */
	bool srvGetGlobalLocalizationMap(map_server_2d::GetGlobalLocalizationMap::Request& request,  map_server_2d::GetGlobalLocalizationMap::Response& response);

	/**
	 * @brief service method to create a gloal navigation map from the semantic map
	 *
	 * @param request of the lvr_tools::GetGlobalNavigationMap ROS Service, see definition for details 
	 * @param response of the lvr_tools::GetGlobalNavigationMap ROS Service, see definition for details
	 */
	bool srvGetGlobalNavigationMap(map_server_2d::GetGlobalNavigationMap::Request& request,  map_server_2d::GetGlobalNavigationMap::Response& response);

	};

}
#endif /* MAP_SERVER_2D_H_ */
