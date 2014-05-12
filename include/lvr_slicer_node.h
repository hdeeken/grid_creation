#ifndef LVR_SLICER_NODE_H_
#define LVR_SLICER_NODE_H_

#include <io/PLYIO.hpp>
#include <io/Timestamp.hpp>
#include <io/Progress.hpp>
#include <io/DataStruct.hpp>
#include <io/Model.hpp>
#include <io/MeshBuffer.hpp>
#include <io/ModelFactory.hpp>
#include <slicer/MeshSlicer.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <lvr_tools/Face.h>
#include <lvr_tools/TriangleMeshGeometry.h>
#include <lvr_tools/GetSingleSliceGrid.h>
#include <lvr_tools/GetMultiSliceGrid.h>
#include <lvr_tools/GetSingleSliceSegments.h>
#include <lvr_tools/GetMultiSliceSegments.h>
#include <lvr_ros_converter.h>

using namespace std;
using namespace lvr;
using namespace nav_msgs;
using namespace visualization_msgs;

namespace lvr_slicer_node
{

/**
 * @brief  The LVR Slicer Node provides a ROS wrapper to the LVR functionality to slice triangle meshes and create occupancy grid maps from them.
 *          The node provides services to create grids from single slices or multiple slices projected into one grid, which can be used 
 *          in robot navigation. Further a callback mode can be enabled that takes in meshes from a topic and outputs grid maps and the respective markers.
 */

class LvrSlicerNode
{

private:
/// node handle
ros::NodeHandle nh;

/// topic to output the grid of a mesh that will be sliced once, only used in callback mode
string single_slice_grid_topic;
/// topic to output the markers of a the mesh that will be sliced once, only used in debug mode
string single_slice_marker_topic;
/// topic to output the grid of a mesh that will be sliced multiple times, only used in callback mode
string multi_slice_grid_topic;
/// topic to output the markers of a the mesh that will be sliced multiple times, only used in debug mode
string multi_slice_marker_topic;

/// subscriber to input the mesh that will be sliced, only used in callback mode
ros::Subscriber mesh_sub;
/// publisher to output the grid of a mesh that will be sliced once, only used in callback mode
ros::Publisher single_slice_grid_pub;
/// publisher to output the markers of a the mesh that will be sliced multiple times, only used in debug mode
ros::Publisher single_slice_marker_pub;
/// publisher to output the grid of a mesh that will be sliced multiple times, only used in callback mode
ros::Publisher multi_slice_grid_pub;
/// publisher to output the markers of a the mesh that will be sliced multiple times, only used in debug mode
ros::Publisher multi_slice_marker_pub;

/// server for the single slice grid service
ros::ServiceServer get_single_slice_grid_srv;
/// server for the multi slice grid service
ros::ServiceServer get_multi_slice_grid_srv;
/// server for the single slice segments service
ros::ServiceServer get_single_slice_segments_srv;
/// server for the multi slice segments service
ros::ServiceServer get_multi_slice_segments_srv;

/// converter object to transform between LVR and ROS types
lvr_ros_converter::LvrRosConverter converter;

/// LVR object that is used to do the acutal slicing process 
MeshSlicer slicer;

/// parameter to set verbosity of the slicing process within the lvr class 
bool verbose_lvr;
/// parameter to enable the callback functionality
bool activate_callback;
/// dimension that orients the plane
string dimension;
///  value set the intersecting plane on the given dimension
double single_slice_value;
///  min value for the intersecting planes on the given dimension
double multi_slice_min_value;
///  max value for the intersecting planes on the given dimension
double multi_slice_max_value;
///  resolution value for the intersecting planes on the given dimension
double multi_slice_resolution;
///  resolution value for the resulting grid map
double grid_resolution;
///  default value for unoccupied values on the resulting grid map
double grid_unoccupied_default;

/**
 * @brief method to convert a list of float points into a list of geometry_msgs::Point
 *
 * @param input vector of float values representing section points
 * @return output vector of ROS points
 */
vector<geometry_msgs::Point> convertToGeometryMsgsPoints(vector<float> input);

/**
 * @brief service method to create a grid map from a triangle mesh by slicing it with one specific intersection plane
 *
 * @param request of the lvr_tools::GetSingleSliceGrid ROS Service, see definition for details 
 * @param response of the lvr_tools::GetSingleSliceGrid ROS Service, see definition for details
 */
 
lvr_tools::GetSingleSliceGrid::Response createSingleSliceGrid(lvr_tools::GetSingleSliceGrid::Request request);

/**
 * @brief service method to create a grid map from a triangle mesh by slicing it multiple times and project them onto one plane
 *
 * @param request of the lvr_tools::GetMultiSliceGrid ROS Service, see definition for details 
 * @return response of the lvr_tools::GetMultiSliceGrid ROS Service, see definition for details
 */

lvr_tools::GetMultiSliceGrid::Response createMultiSliceGrid(lvr_tools::GetMultiSliceGrid::Request request);

/**
 * @brief method to create a occupancy grid from a set of points
 *
 * @param frame that the map should be relative to
 * @param time stamp
 * @param origin pose
 * @param points that define the intersection segments created by slicing the mesh
 * @param resolution of the resulting map to convert from the continouus to the discrete
 * @param unoccupied_default value to be used for potentially free space
 * @return nav_msgs::OccupancyGrid that can be used for robot navigation and localization
 */

nav_msgs::OccupancyGrid createOccupancyGrid(string frame, ros::Time time, geometry_msgs::Pose origin, vector<geometry_msgs::Point> points, double resolution, double unoccupied_default);

/**
 * @brief service method to create a markers from a triangle mesh by slicing it with one specific intersection plane
 *
 * @param frame that the map should be relative to
 * @param pose
 * @param points that define the intersection segments created by slicing the mesh
 * @param rgba color for the marker
 * @return visualization_msgs::MarkerArray that visualizes the section segments and the sections points
 */ 
 
visualization_msgs::MarkerArray createMarkerArray(string frame, geometry_msgs::Pose pose, vector<geometry_msgs::Point> points, std_msgs::ColorRGBA rgba);

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

void drawLineBresenham(int xa, int ya, int xb, int yb, vector<signed char>& data, int width);

public:

/**
  * @brief Constructor
  */
LvrSlicerNode(ros::NodeHandle n);

/**
  * @brief Destructor
  */
~LvrSlicerNode() {};

/**
 * @brief calls the creation of single and multi slice grids based on mesh data coming from a topic, mainly for debug and visualization purposes
 *
 * @param mesh a ROS lvr_tools::TriangleMeshGeometry Message
 */
void mesh_callback(const lvr_tools::TriangleMeshGeometry::ConstPtr& mesh);

/**
 * @brief service method to create a grid map from a triangle mesh by slicing it with one specific intersection plane
 *
 * @param request of the lvr_tools::GetSingleSliceGrid ROS Service, see definition for details 
 * @param response of the lvr_tools::GetSingleSliceGrid ROS Service, see definition for details
 */
 
bool srvGetSingleSliceGrid(lvr_tools::GetSingleSliceGrid::Request& request,  lvr_tools::GetSingleSliceGrid::Response& response);

/**
 * @brief service method to create a grid map from a triangle mesh by slicing it with multiple intersection planes, along one axis
 *
 * @param request of the lvr_tools::GetSingleSliceGrid ROS Service, see definition for details 
 * @param response of the lvr_tools::GetSingleSliceGrid ROS Service, see definition for details
 */
bool srvGetMultiSliceGrid(lvr_tools::GetMultiSliceGrid::Request& request,  lvr_tools::GetMultiSliceGrid::Response& response);


/**
 * @brief service method to create a list of line segments from a triangle mesh by slicing it with one specific intersection plane
 *
 * @param request of the lvr_tools::GetSingleSliceSegments ROS Service, see definition for details 
 * @param response of the lvr_tools::GetSingleSliceSegments ROS Service, see definition for details
 */
 
bool srvGetSingleSliceSegments(lvr_tools::GetSingleSliceSegments::Request& request,  lvr_tools::GetSingleSliceSegments::Response& response);

/**
 * @brief service method to create a list of line segments from a triangle mesh by slicing it with multiple intersection planes, along one axis
 *
 * @param request of the lvr_tools::GetMultiSliceSegments ROS Service, see definition for details 
 * @param response of the lvr_tools::GetMultiSliceSegments ROS Service, see definition for details
 */
bool srvGetMultiSliceSegments(lvr_tools::GetMultiSliceSegments::Request& request,  lvr_tools::GetMultiSliceSegments::Response& response);

};

}

#endif /* LVR_SLICER_NODE_H_ */
