/*
 * Studygroup MUFFIN Packages - Robot Operating System
 *
 * Copyright (C) 2013 University of Osnabrück, Germany
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * lvr_slicer_node.cpp
 *
 *  Created on: 22.08.2013
 *      Author: Henning Deeken    <hdeeken@uos.de>
 *              Ann-Katrin Häuser <ahaeuser@uos.de>
 */

#include <lvr_slicer_node.h> 

namespace lvr_slicer_node{

LvrSlicerNode::LvrSlicerNode(ros::NodeHandle n)
{
	nh = n;
	ros::NodeHandle nh_ns("~");
	nh_ns.param("verbose_lvr", verbose_lvr, false);
	nh_ns.param("activate_callback", activate_callback, false);

	nh_ns.param("dimension", dimension, string("z"));
	nh_ns.param("single_slice_value", single_slice_value, 1.0);
	nh_ns.param("multi_slice_min_value", multi_slice_min_value, 0.0);
	nh_ns.param("multi_slice_max_value", multi_slice_max_value, 1.0);
	nh_ns.param("multi_slice_resolution", multi_slice_resolution, 0.05);
	nh_ns.param("grid_resolution", grid_resolution, 0.05);
	nh_ns.param("grid_unoccupied_default", grid_unoccupied_default, 1.0);

	// advertise the services for slicing
	get_single_slice_grid_srv = nh.advertiseService("get_single_slice_grid", &LvrSlicerNode::srvGetSingleSliceGrid, this);
	get_multi_slice_grid_srv = nh.advertiseService("get_multi_slice_grid", &LvrSlicerNode::srvGetMultiSliceGrid, this);

	ROS_INFO("LVR Slicer Node");
	ROS_INFO("Advertising GetSingleSliceGrid Service on 'get_single_slice_grid'.");
	ROS_INFO("Advertising GetMultiSliceGrid Service  on 'get_multi_slice_grid'. \n");

	// if the node is in callback mode, connect the respective topics
	if(activate_callback)
	{
		mesh_sub  = nh.subscribe("/mesh", 1, &LvrSlicerNode::mesh_callback, this);
		single_slice_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/single_grid", 1);
		single_slice_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/single_grid_marker", 1);
		multi_slice_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/multi_grid", 1);
		multi_slice_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/multi_grid_marker", 1);
		ROS_INFO("Publising SingleSliceGrids and Markers on %s and %s.", single_slice_grid_topic.c_str(), single_slice_marker_topic.c_str());
		ROS_INFO("Publising MultiSliceGrids an Markers on %s and %s.", multi_slice_grid_topic.c_str(), multi_slice_marker_topic.c_str());
	}
}

vector<float> LvrSlicerNode::createSingleSliceSegments(lvr_tools::Mesh mesh, double dimension, double value)
{
	// convert the incoming ros message into a lvr mesh buffer
	MeshBufferPtr input_mesh = boost::make_shared<MeshBuffer>(converter.convertMeshMessageToMeshBuffer(mesh.mesh));

	// setup the lvr class object with the requested parameters
	slicer.clear();
	slicer.setVerbosity(verbose_lvr);
	slicer.setDimension(dimension);
	slicer.setValue(value);

	// slice the mesh once
	vector<float> slice = slicer.addMeshAndCompute2dSlice(input_mesh);
	return slice;
}

vector<float> LvrSlicerNode::createMultiSliceSegments(lvr_tools::Mesh mesh, double dimension, double min_value, double max_value, double resolution)
{
	// convert the incoming ros message into a lvr mesh buffer
	MeshBufferPtr input_mesh = boost::make_shared<MeshBuffer>(converter.convertMeshMessageToMeshBuffer(mesh.mesh));

	// setup the lvr class object with the requested parameters
	slicer.clear();
	slicer.setVerbosity(verbose_lvr);
	slicer.setDimension(dimension);
	slicer.setMinValue(min_value);
	slicer.setMaxValue(max_value);
	slicer.setResolution(resolution);

	// slice the mesh multiple times
	vector<float> projection = slicer.addMeshAndCompute2dProjection(input_mesh);
	return projection;
}

void LvrSlicerNode::mesh_callback(const lvr_tools::TriangleMeshGeometry::ConstPtr& mesh)
{
	// create a single slice grid
	lvr_tools::GetSingleSliceGrid::Request single_slice_grid_request;
	lvr_tools::GetSingleSliceGrid::Response single_slice_response;

	single_slice_grid_request.dimension = dimension;
	single_slice_grid_request.value = single_slice_value;
	single_slice_grid_request.mesh = *mesh;
	single_slice_grid_request.grid_resolution = grid_resolution;
	single_slice_grid_request.grid_unoccupied_default = grid_unoccupied_default;

	single_slice_response = createSingleSliceGrid(single_slice_grid_request);

	single_slice_grid_pub.publish(single_slice_response.grid);
	single_slice_marker_pub.publish(single_slice_response.marker);

	// create a multi slice grid
	lvr_tools::GetMultiSliceGrid::Request multi_slice_grid_request;
	lvr_tools::GetMultiSliceGrid::Response multi_slice_response;

	multi_slice_grid_request.dimension = dimension;
	multi_slice_grid_request.min_value = multi_slice_min_value;
	multi_slice_grid_request.max_value = multi_slice_max_value;
	multi_slice_grid_request.mesh = *mesh;
	multi_slice_grid_request.slicing_resolution = multi_slice_resolution;
	multi_slice_grid_request.grid_resolution = grid_resolution;
	multi_slice_grid_request.grid_unoccupied_default = grid_unoccupied_default;

	multi_slice_response = createMultiSliceGrid(multi_slice_grid_request);

	multi_slice_grid_pub.publish(multi_slice_response.grid);
	multi_slice_marker_pub.publish(multi_slice_response.marker);
}

bool LvrSlicerNode::srvGetSingleSliceGrid(lvr_tools::GetSingleSliceGrid::Request& request, lvr_tools::GetSingleSliceGrid::Response& response)
{
	vectory<float> slice = createSingleSliceSegments(request.mesh, request.dimension, request.value);
	vector<geometry_msgs::Point> slice_points = convertToGeometryMsgsPoints(slice);

	response.grid = createOccupancyGrid(request.mesh.header.frame_id, request.mesh.header.stamp, request.mesh.pose, slice_points, request.grid_resolution, request.grid_unoccupied_default);
	response.marker = createMarkerArray(request.mesh.header.frame_id, request.mesh.pose, slice_points, request.color);

	return true;
}

bool LvrSlicerNode::srvGetMultiSliceGrid(lvr_tools::GetMultiSliceGrid::Request& request, lvr_tools::GetMultiSliceGrid::Response& response)
{
	vectory<float> projection = createMultiSliceSegments(request.mesh, request.dimension, request.min_value, request.max_value, request.slicing_resolution);
	vector<geometry_msgs::Point> projection_points = convertToGeometryMsgsPoints(projection);

	response.grid = createOccupancyGrid(request.mesh.header.frame_id, request.mesh.header.stamp, request.mesh.pose, projection_points, request.grid_resolution, request.grid_unoccupied_default);
	response.marker = createMarkerArray(request.mesh.header.frame_id, request.mesh.pose, projection_points, request.color);

	return true;
}

bool LvrSlicerNode::srvGetSingleSliceSegments(lvr_tools::GetSingleSliceSegments::Request& request, lvr_tools::GetSingleSliceSegments::Response& response)
{
	vectory<float> slice = createSingleSliceSegments(request.mesh, request.dimension, request.value);
	vector<geometry_msgs::Point> slice_points = convertToGeometryMsgsPoints(slice);

	response.segments = slice;
	response.marker = createMarkerArray(request.mesh.header.frame_id, request.mesh.pose, slice_points, request.color);

	return true;
}

bool LvrSlicerNode::srvGetMultiSliceSegments(lvr_tools::GetMultiSliceSegments::Request& request, lvr_tools:GetMultiSliceSegments::Response& response)
{
	vectory<float> projection = createMultiSliceSegments(request.mesh, request.dimension, request.min_value, request.max_value, request.slicing_resolution);
	vector<geometry_msgs::Point> projection_points = convertToGeometryMsgsPoints(projection);

	response.segments = projection;
	response.marker = createMarkerArray(request.mesh.header.frame_id, request.mesh.pose, projection_points, request.color);

	return true;
}

// move to lib
vector<geometry_msgs::Point> LvrSlicerNode::convertToGeometryMsgsPoints(vector<float> input)
{
	vector<geometry_msgs::Point> output;

	for(unsigned int i = 0; i < input.size(); i+=3)
	{
		geometry_msgs::Point point;
		point.x = input.at(i);
		point.y = input.at(i+1);
		point.z = input.at(i+2);
		output.push_back(point);
	}

	return output;
}

// move to lib
void LvrSlicerNode::drawLineBresenham(int x1, int y1, int x2, int y2, vector<signed char>& data, int width)
{
	int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;
	dx=x2-x1;
	dy=y2-y1;
	dx1=fabs(dx);
	dy1=fabs(dy);
	px=2*dy1-dx1;
	py=2*dx1-dy1;

	if(dy1<=dx1)
	{
		if(dx>=0)
		{
			x=x1;
			y=y1;
			xe=x2;
		}
		else
		{
		x=x2;
		y=y2;
		xe=x1;
		}

		data.at(y * width + x) = (signed char) 100;
		for(i=0;x<xe;i++)
		{
			x=x+1;
			if(px<0)
			{
				px=px+2*dy1;
			}
			else
			{
				if((dx<0 && dy<0) || (dx>0 && dy>0))
				{
					y=y+1;
				}
				else
				{
					y=y-1;
				}
				px=px+2*(dy1-dx1);
			}
			data.at(y * width + x) = (signed char) 100;
		}
	}
	else
	{
		if(dy>=0)
		{
			x=x1;
			y=y1;
			ye=y2;
		}
		else
		{
			x=x2;
			y=y2;
			ye=y1;
		}

		data.at(y * width + x) = (signed char) 100;

		for(i=0;y<ye;i++)
		{
			y=y+1;
			if(py<=0)
			{
				py=py+2*dx1;
			}
			else
			{
				if((dx<0 && dy<0) || (dx>0 && dy>0))
				{
					x=x+1;
				}
				else
				{
					x=x-1;
				}
				py=py+2*(dx1-dy1);
			}
			data.at(y * width + x) = (signed char) 100;
		}
	}
}

// move to lib
nav_msgs::OccupancyGrid LvrSlicerNode::createOccupancyGrid(string frame, ros::Time time, geometry_msgs::Pose origin, vector<geometry_msgs::Point> points, double resolution, double unoccupied_default)
{
	nav_msgs::OccupancyGrid result;

	// determine 2d bounding area 
	// note the plane is called ij from here on, since it is not necessarily aligned with one of the global planes (e.g, XY)

	float min_i = FLT_MAX;
	float min_j = FLT_MAX;
	float max_i = FLT_MIN;
	float max_j = FLT_MIN;

	// extract the relevant dimensions and determine min and max boundaries
	vector<float> points_2d;
	for(unsigned int i = 0; i < points.size(); i++)
	{
		geometry_msgs::Point point;
		point = points.at(i);
		if(dimension.compare("x") == 0)
		{
			points_2d.push_back(point.y);
			points_2d.push_back(point.z);
			if (point.y < min_i)
				min_i = point.y;
			if (point.y > max_i)
				max_i = point.y;
			if (point.z < min_j)
				min_j = point.z;
			if (point.z > max_j)
				max_j = point.z;
		}
		else if(dimension.compare("y") == 0)
		{
			points_2d.push_back(point.x);
			points_2d.push_back(point.z);
			if (point.x < min_i)
				min_i = point.x;
			if (point.x > max_i)
				max_i = point.x;
			if (point.z < min_j)
				min_j = point.z;
			if (point.z > max_j)
				max_j = point.z;
		}
		else if(dimension.compare("z") == 0)
		{
			points_2d.push_back(point.x);
			points_2d.push_back(point.y);
			if (point.x < min_i)
				min_i = point.x;
			if (point.x > max_i)
				max_i = point.x;
			if (point.y < min_j)
				min_j = point.y;
			if (point.y > max_j)
				max_j = point.y;
		}
	}

	// calculate the size of the map depending on min and max boundaries and the resolution. 
	// note the +2 is an arbitrary chosen padding to the grid
	int width  = static_cast<int>((max_i - min_i) / resolution) + 2;
	int height = static_cast<int>((max_j - min_j) / resolution) + 2;  

	vector<signed char> data;
	data.resize(width * height, unoccupied_default); 

	// iterate the points an draw them onto the plane, taking the resolution and padding into account (+1)
	for(unsigned int i = 0; i < points_2d.size(); i+=4)
	{
		int a_x, a_y, b_x, b_y;
		try {
			a_x = (int) ((points_2d.at(i)    - min_i)  / resolution ) + 1;
			a_y = (int) ((points_2d.at(i+1)  - min_j)  / resolution ) + 1;
			b_x = (int) ((points_2d.at(i+2)  - min_i)  / resolution ) + 1; 
			b_y = (int) ((points_2d.at(i+3)  - min_j)  / resolution ) + 1;
			drawLineBresenham(a_x, a_y, b_x, b_y, data, width);
		}
		catch(std::out_of_range e)
		{
			cout << e.what()<<endl;
		}
	}

	// setup the outgoing ROS message 
	result.info.map_load_time = time;
	result.info.resolution = resolution;
	origin.position.x = min_i;
	origin.position.y = min_j;
	origin.orientation.w = 1.0;
	result.info.origin = origin; 
	result.info.width = width;
	result.info.height = height; 
	result.data = data;
	result.header.frame_id = frame;
	//result.header.stamp = ros::Time::now();

	return result;
}

// move to lib
visualization_msgs::MarkerArray LvrSlicerNode::createMarkerArray(string frame, geometry_msgs::Pose pose, vector<geometry_msgs::Point> points, std_msgs::ColorRGBA rgba)
{
	visualization_msgs::MarkerArray marker;

	// create line markers that show the section segments
	visualization_msgs::Marker lines;
	lines.header.frame_id = frame;
	//lines.header.stamp = ros::Time::now();
	lines.ns = "slice_lines";
	lines.id = 0;
	lines.type = visualization_msgs::Marker::LINE_LIST;
	lines.action = visualization_msgs::Marker::ADD;
	lines.pose = pose;

	geometry_msgs::Vector3 scale;
	// make lines of 5x5x5cm 
	scale.x = 0.05;
	scale.y = 0.05;
	scale.z = 0.05;
	lines.scale = scale;
	lines.color = rgba;
	lines.points = points;

	// create point markers that show the individual sections
	visualization_msgs::Marker point_marker;
	point_marker.header.frame_id = frame;
	//point_marker.header.stamp = ros::Time::now();
	point_marker.ns = "slice_points";
	point_marker.id = 0;
	point_marker.type = visualization_msgs::Marker::POINTS;
	point_marker.action = visualization_msgs::Marker::ADD;
	point_marker.pose = pose;
	point_marker.scale = scale;
	point_marker.color = rgba;
	point_marker.points = points;

	// add to marker array and return
	marker.markers.push_back(point_marker);
	marker.markers.push_back(lines);
	return marker;
}

} 

// Let the magic happen...

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lvr_slicer_node");
	ros::NodeHandle n;
	lvr_slicer_node::LvrSlicerNode node(n);

	// using a loop rate frees cpu capacity, however not necessary
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
