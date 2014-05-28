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
 * grid_creation.cpp
 *
 *  Created on: 12.5.2015
 *      Author: Henning Deeken    <hdeeken@uos.de>
 *              Sebastian Pütz    <spuetz@uos.de>
 */

#include <grid_creation.h>

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
    std::vector<double> points,
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

    double min_x = DBL_MAX;
    double min_y = DBL_MAX;
    double max_x = DBL_MIN;
    double max_y = DBL_MIN;

    // extract the relevant dimensions and determine min and max boundaries

    for(int i =0; i < points.size();i+=4)
    {
        min_x = std::min(points[i], min_x);
        min_y = std::min(points[i+1], min_y);
        min_x = std::min(points[i+2], min_x);
        min_y = std::min(points[i+3], min_y);
        
        max_x = std::max(points[i], max_x);
        max_y = std::max(points[i+1], max_y);
        max_x = std::max(points[i+2], max_x);
        max_y = std::max(points[i+3], max_y);
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
    for(int i =0; i < points.size();i+=4)
    {
        a_x = (int) ((points[i] - min_x) / resolution ) + 1;
        a_y = (int) ((points[i+1] - min_y) / resolution ) + 1;
        b_x = (int) ((points[i+2] - min_x) / resolution ) + 1; 
        b_y = (int) ((points[i+3] - min_y) / resolution ) + 1;
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
    grid.header.stamp = ros::Time::now();

    return true;
}

bool GridCreation::createBoxSegments(lvr_tools::BoundingBox bbox,
                                        double resolution,
                                        double padding,
                                        std::vector<double>& segments,
                                        std::vector<geometry_msgs::Point>& points)
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

        tf::Vector3 minX (bbox.pose.pose.position.x - padding, 0, 0);
        tf::Vector3 maxX (bbox.x_edge + padding, 0, 0);
        tf::Vector3 minY (0, - padding, 0);
        tf::Vector3 maxY (0, bbox.y_edge + padding, 0);

        tf::Vector3 a = transform* tf::Vector3(bbox.pose.pose.position.x - padding, bbox.pose.pose.position.y - padding, 0);
        tf::Vector3 b = transform*tf::Vector3(bbox.pose.pose.position.x + bbox.x_edge + padding , bbox.pose.pose.position.y - padding, 0);
        tf::Vector3 d = transform*tf::Vector3(bbox.pose.pose.position.x - padding, bbox.pose.pose.position.y + bbox.y_edge + padding, 0);
        tf::Vector3 c = transform*tf::Vector3(bbox.pose.pose.position.x + bbox.x_edge + padding , bbox.pose.pose.position.y + bbox.y_edge + padding, 0);
        /*tf::Vector3 a = transform * tf::Vector3(-padding, -padding, 0);
        tf::Vector3 b = transform * tf::Vector3(bbox.x_edge + padding , -padding, 0);
        tf::Vector3 c = transform * tf::Vector3(bbox.x_edge + padding , bbox.y_edge + padding, 0);
        tf::Vector3 d = transform * tf::Vector3(-padding, bbox.y_edge + padding, 0);*/

            geometry_msgs::Point ap;
            ap.x = a.x();
            ap.y = a.y();
            ap.z = a.z();

            geometry_msgs::Point bp;
            bp.x = b.x();
            bp.y = b.y();
            bp.z = b.z();

            geometry_msgs::Point cp;
            cp.x = c.x();
            cp.y = c.y();
            cp.z = c.z();

            geometry_msgs::Point dp;
            dp.x = d.x();
            dp.y = d.y();
            dp.z = d.z();

        //points.push_back(ap);
        //points.push_back(bp);
        //points.push_back(cp);
        //points.push_back(dp);

        tf::Vector3 rel_height = d-a;
        tf::Vector3 rel_width = b-a;

        double minXMap = std::min( std::min(a.x(), b.x() ), std::min( c.x(), d.x() ) );
        double minYMap = std::min( std::min(a.y(), b.y() ), std::min( c.y(), d.y() ) );

        tf::Vector3 minPos(minXMap, minYMap, 0);

        //double parts_x = std::ceil(rel_width.length() / resolution); 
        double parts_y = std::ceil(rel_height.length() / resolution); 
        //tf::Vector3 increment_x = rel_width / parts_x;
        tf::Vector3 increment_y = rel_height / parts_y;

        tf::Vector3 start, end;
        int i;

        for(i = 0, start = minPos , end = minPos + rel_width;
            i < parts_y ; start += increment_y, end += increment_y, i++) 
        {
            geometry_msgs::Point start_p;
            start_p.x = start.x();
            start_p.y = start.y();
            start_p.z = 0.0;
            geometry_msgs::Point end_p;
            end_p.x = end.x();
            end_p.y = end.y();
            end_p.z = 0.0;
            points.push_back(start_p);
            points.push_back(end_p);

            segments.push_back(start.x());
            segments.push_back(start.y());
            segments.push_back(end.x());
            segments.push_back(end.y());
        }
            //start += increment_y;
            //end += increment_y;

        return true;
    }

visualization_msgs::Marker GridCreation::createLineMarker(
    std::string name,
    std::string frame,
    geometry_msgs::Pose pose,
    std::vector<geometry_msgs::Point> points,
    std_msgs::ColorRGBA rgba,
    geometry_msgs::Vector3 scale)
{
    // create line markers that show the section segments
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale = scale;
    marker.color = rgba;
    marker.points = points;

    return marker;
}

visualization_msgs::Marker GridCreation::createPointMarker(
    std::string name,
    std::string frame,
    geometry_msgs::Pose pose,
    std::vector<geometry_msgs::Point> points,
    std_msgs::ColorRGBA rgba,
    geometry_msgs::Vector3 scale)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale = scale;
    marker.color = rgba;
    marker.points = points;

    return marker;
}

/*
 void GridCreation::printBBox(lvr_tools::BoundingBox bbox)
{
    cout << "Frame: " << bbox.pose.header.frame_id << endl;
    cout << "Position: " << bbox.pose.pose.position.x << " " << bbox.pose.pose.position.y << " " << bbox.pose.pose.position.z << endl;
    cout << "Orientation: " << bbox.pose.pose.orientation.x << " " << bbox.pose.pose.orientation.y << " " << bbox.pose.pose.orientation.z << " " << bbox.pose.pose.orientation.w << endl;
    cout << "Extension: " << bbox.x_edge << " " << bbox.y_edge << " " << bbox.z_edge << endl;
}
*/

visualization_msgs::Marker GridCreation::createBoxLabel(lvr_tools::BoundingBox bbox, std::string text, std::string name, std::string frame, geometry_msgs::Vector3 scale,std_msgs::ColorRGBA color)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.pose =  bbox.pose.pose;
    marker.pose.position.x += bbox.x_edge/2;
    marker.pose.position.y += bbox.y_edge/2;
    marker.pose.position.z += bbox.z_edge/2;
    marker.scale = scale;
    marker.color = color;
    marker.text = text;

    return marker;
}

visualization_msgs::Marker GridCreation::createBoxMarker(lvr_tools::BoundingBox bbox, std::string name, std::string frame, geometry_msgs::Vector3 scale,std_msgs::ColorRGBA color)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.pose =  bbox.pose.pose;
    marker.scale = scale;
    marker.color = color;

    // all points
    geometry_msgs::Point zero;
        zero.x = 0; zero.y = 0; zero.z = 0;
    geometry_msgs::Point one;
        one.x = bbox.x_edge; one.y = 0; one.z = 0;
    geometry_msgs::Point two;
        two.x = bbox.x_edge; two.y = 0; two.z = bbox.z_edge;
    geometry_msgs::Point three;
        three.x = 0; three.y = 0; three.z = bbox.z_edge;
    geometry_msgs::Point four;
        four.x = 0; four.y = bbox.y_edge; four.z = 0;
    geometry_msgs::Point five;
        five.x = bbox.x_edge; five.y = bbox.y_edge; five.z = 0;
    geometry_msgs::Point six;
        six.x = bbox.x_edge; six.y = bbox.y_edge; six.z = bbox.z_edge;
    geometry_msgs::Point seven;
        seven.x = 0; seven.y = bbox.y_edge; seven.z = bbox.z_edge;

    // all edges
    //zero
    marker.points.push_back(zero);
    marker.points.push_back(one);
    //one
    marker.points.push_back(one);
    marker.points.push_back(two);
    //two
    marker.points.push_back(two);
    marker.points.push_back(three);
    //three
    marker.points.push_back(zero);
    marker.points.push_back(three);
    //four
    marker.points.push_back(four);
    marker.points.push_back(five);
    //five
    marker.points.push_back(five);
    marker.points.push_back(six);
    //six
    marker.points.push_back(six);
    marker.points.push_back(seven);
    //seven
    marker.points.push_back(four);
    marker.points.push_back(seven);
    //eight
    marker.points.push_back(zero);
    marker.points.push_back(four);
    //nine
    marker.points.push_back(one);
    marker.points.push_back(five);
    //ten
    marker.points.push_back(two);
    marker.points.push_back(six);
    //eleven
    marker.points.push_back(three);
    marker.points.push_back(seven);

    return marker;
}
