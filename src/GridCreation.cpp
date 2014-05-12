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
 * GridCreation.cpp
 *
 *  Created on: 12.5.2015
 *      Author: Henning Deeken    <hdeeken@uos.de>
 *              Sebastian Pütz    <spuetz@uos.de>
 */

#include <GridCreation.hpp>
std::vector<geometry_msgs::Point> GridCreation::convertToGeometryMsgsPoints(
    std::vector<float> input)
{
    std::vector<geometry_msgs::Point> output;

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

void GridCreation::drawLineBresenham(int x1, int y1, int x2, int y2, std::vector<signed char>& data, int width)
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

bool GridCreation::createOccupancyGrid(
    std::string frame,
    ros::Time time,
    geometry_msgs::Pose origin,
    std::vector<float> points,
    double resolution,
    double unoccupied_default,
    nav_msgs::OccupancyGrid& grid)
{
    if(points.size() % 4 != 0)
    {
        ROS_ERROR("wrong number of points in createOccupancyGrid!");
        return false;
    }

    nav_msgs::OccupancyGrid result;

    // determine 2d bounding area 

    float min_x = FLT_MAX;
    float min_y = FLT_MAX;
    float max_x = FLT_MIN;
    float max_y = FLT_MIN;

    // extract the relevant dimensions and determine min and max boundaries
    std::vector<float> points_2d;

    std::vector<float>::iterator x_iter1, x_iter2, y_iter1, y_iter2;

    for(    x_iter1 = points.begin(),
        y_iter1 = points.begin() + 1,
        x_iter2 = points.begin() + 2,
        y_iter2 = points.begin() + 3;

        y_iter2 != points_2d.end();
    
        x_iter1 += 4,
        y_iter1 += 4,
        x_iter2 += 4,
        y_iter2 += 4)
    {
        min_x = std::min(*x_iter1, min_x);
        min_y = std::min(*y_iter1, min_y);
        min_x = std::min(*x_iter2, min_x);
        min_y = std::min(*y_iter2, min_y);

        max_x = std::max(*x_iter1, max_x);
        max_y = std::max(*y_iter1, max_y);
        max_x = std::max(*x_iter2, max_x);
        max_y = std::max(*y_iter2, max_y);
    }

    // calculate the size of the map depending on min and max
    // boundaries and the resolution. 
    // note the +2 is an arbitrary chosen padding to the grid
    int width  = static_cast<int>((max_x - min_x) / resolution) + 2;
    int height = static_cast<int>((max_y - min_y) / resolution) + 2;  

    std::vector<signed char> data;
    data.resize(width * height, unoccupied_default); 

    // iterate the points an draw them onto the plane,
    // taking the resolution and padding into account (+1)
    
    int a_x, a_y, b_x, b_y;
    
    for(    x_iter1 = points.begin(),
        y_iter1 = points.begin() + 1,
        x_iter2 = points.begin() + 2,
        y_iter2 = points.begin() + 3;

        y_iter2 != points_2d.end();
    
        x_iter1 += 4,
        y_iter1 += 4,
        x_iter2 += 4,
        y_iter2 += 4)
    {
        a_x = (int) ((*x_iter1 - min_x) / resolution ) + 1;
        a_y = (int) ((*y_iter1 - min_y) / resolution ) + 1;
        b_x = (int) ((*x_iter2 - min_x) / resolution ) + 1; 
        b_y = (int) ((*y_iter2 - min_y) / resolution ) + 1;
        GridCreation::drawLineBresenham(a_x, a_y, b_x, b_y, data, width);
    }

    // setup the outgoing ROS message 
    grid.info.map_load_time = time;
    grid.info.resolution = resolution;
    origin.position.x = min_x;
    origin.position.y = min_y;
    origin.orientation.w = 1.0;
    grid.info.origin = origin; 
    grid.info.width = width;
    grid.info.height = height; 
    grid.data = data;
    grid.header.frame_id = frame;
    //grid.header.stamp = ros::Time::now();

    return true;
}

visualization_msgs::MarkerArray GridCreation::createMarkerArray(
    std::string frame,
    geometry_msgs::Pose pose,
    std::vector<geometry_msgs::Point> points,
    std_msgs::ColorRGBA rgba)
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


bool GridCreation::creatSegments(		lvr_tools::BoundingBox bbox,
										double resolution,
										double padding,
										std::vector<float> points)
	{
		// determine 2d bounding area 
		// note the plane is called ij from here on
		// z-coord should be zero
	
		tf::Quaternion orientation( bbox.pose.pose.orientation.x,
									bbox.pose.pose.orientation.y,
									bbox.pose.pose.orientation.z,
									bbox.pose.pose.orientation.w);

		tf::Vector3 position(
							bbox.pose.pose.position.x,
							bbox.pose.pose.position.y,
							bbox.pose.pose.position.z);

		// to rotate the edges with padding
		tf::Transform transform(orientation);

		tf::Vector3 minX (- padding, 0, 0);
		tf::Vector3 maxX (bbox.x_edge + padding, 0, 0);
		tf::Vector3 minY (0, - padding, 0);
		tf::Vector3 maxY (0, bbox.y_edge + padding, 0);

		tf::Vector3 a = transform * tf::Vector3(-padding, -padding, 0);
		tf::Vector3 b = transform * tf::Vector3(bbox.x_edge + padding , -padding, 0);
		tf::Vector3 c = transform * tf::Vector3(bbox.x_edge + padding , bbox.y_edge + padding, 0);
		tf::Vector3 d = transform * tf::Vector3(-padding, bbox.y_edge + padding, 0);
		
		tf::Vector3 rel_height = d-a;
		tf::Vector3 rel_width = b-a;
		
		float minXMap = std::min( std::min(a.x(), b.x() ), std::min( c.x(), d.x() ) );
		float minYMap = std::min( std::min(a.y(), b.y() ), std::min( c.y(), d.y() ) );

		tf::Vector3 minPos(minXMap, minYMap, 0);

		float parts = std::ceil(rel_height.length() / (resolution *0.33)); 
		
		tf::Vector3 increment = rel_height / parts;
		tf::Vector3 start, end;

		int i;
		// minPos less then or equal zero
		for(i = 0, start = -minPos , end = rel_width - minPos;
			i < parts ;
			start += increment, end += increment, i++)
		{
			
			points.push_back(start.x());
			points.push_back(start.y());
			points.push_back(end.x());
			points.push_back(end.y());
		}

		return true;
	}

