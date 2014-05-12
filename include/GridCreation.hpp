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
 * GridCreation.hpp
 *
 *  Created on: 12.5.2015
 *      Author: Henning Deeken    <hdeeken@uos.de>
 *              Sebastian Pütz    <spuetz@uos.de>
 */

#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <lvr_tools/BoundingBox.h>

class GridCreation{

    public:
        static vector<geometry_msgs::Point> convertToGeometryMsgsPoints(
            vector<float> input);

        static void drawLineBresenham(int x1, int y1, int x2, int y2, 
            vector<signed char>& data, int width)
        
        static bool createOccupancyGrid(
            string frame,
            ros::Time time,
            geometry_msgs::Pose origin,
            vector<float> points,
            double resolution,
            double unoccupied_default,
            nav_msgs::OccupancyGrid& grid);

        static visualization_msgs::MarkerArray createMarkerArray(
            string frame,
            geometry_msgs::Pose pose,
            vector<geometry_msgs::Point> points,
            std_msgs::ColorRGBA rgba);

		bool creatSegments(
			lvr_tools::BoundingBox bbox,
			double resolution,
			double padding,
			std::vector<float> points);

};
