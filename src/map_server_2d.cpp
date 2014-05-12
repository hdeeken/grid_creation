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
 * map_server_2d.cpp
 *
 *  Created on: 15.12.2013
 *      Author: Henning Deeken <hdeeken@uos.de>
 */
 
#include <map_server_2d.h>

namespace map_server_2d
{
	MapServer2D::MapServer2D(ros::NodeHandle n)
	{
		nh = n;
		// read parameters
		if(setupParameters())
		{
			ROS_INFO("Parameter Setup succeded.");
		}
		else
		{
			ROS_ERROR("Check the yaml file!");
			nh.shutdown();
			return;
		}

		// connect required services
		if(setupServices())
		{
			ROS_INFO("Services Setup succeded.");
		}
		else
		{
			ROS_ERROR("Check the services setups");
			nh.shutdown();
			return;
		}

		// advertise publishers
		locmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map_server/locmap", 1);
		locmap_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/map_server/locmap_marker", 1); 
		navmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map_server/navmap", 1);
		navmap_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/map_server/navmap_marker", 1);

		// advertise services
		get_localization_map_srv_ = nh.advertiseService("get_localization_map", &MapServer2D::srvGetLocalizationMap, this);
		get_navigation_map_srv_ = nh.advertiseService("get_navigation_map", &MapServer2D::srvGetNavigationMap, this);
		get_global_localization_map_srv_ = nh.advertiseService("get_global_localization_map", &MapServer2D::srvGetGlobalLocalizationMap, this);
		get_global_navigation_map_srv_ = nh.advertiseService("get_global_navigation_map", &MapServer2D::srvGetGlobalNavigationMap, this);

		ROS_INFO("Map Server 2D is running.");
	}

	//
	// SETUP
	//

	bool MapServer2D::setupServices()
	{
		ROS_INFO("Waiting for GetMultiSliceGrid Service to be advertised.");
		ros::service::waitForService("get_multi_slice_grid");
		ROS_INFO("Found GetMultiSliceGrid Service!");
		get_multi_slice_grid_client_ = nh.serviceClient<lvr_tools::GetMultiSliceGrid>("get_multi_slice_grid");

		ROS_INFO("Waiting for GetSingleSliceGrid Service to be advertised.");
		ros::service::waitForService("get_single_slice_grid");
		ROS_INFO("Found GetSingleSliceGrid Service!");
		get_single_slice_grid_client_ = nh.serviceClient<lvr_tools::GetSingleSliceGrid>("get_single_slice_grid");

		ROS_INFO("Waiting for GetRegionBBoxes Service to be advertised.");
		ros::service::waitForService("get_region_bboxes");
		ROS_INFO("Found GetRegionBBoxes Service!");
		get_region_bboxes_client_ = nh.serviceClient<postgis_control::GetRegionBBoxes>("get_region_bboxes");

		ROS_INFO("Waiting for GetRoomFaces Service to be advertised.");
		ros::service::waitForService("get_room_faces");
		ROS_INFO("Found GetRoomFaces Service!");
		get_room_faces_client_ = nh.serviceClient<postgis_control::GetRoomFaces>("get_room_faces");

		ROS_INFO("Waiting for GetRegionFaces Service to be advertised.");
		ros::service::waitForService("get_region_faces");
		ROS_INFO("Found GetRegionFaces Service!");
		get_region_faces_client_ = nh.serviceClient<postgis_control::GetRegionFaces>("get_region_faces");

		ROS_INFO("Waiting for GetStructure Service to be advertised.");
		ros::service::waitForService("get_structure");
		ROS_INFO("Found GetStructure Service!");
		get_structure_client_ = nh.serviceClient<postgis_control::GetStructure>("get_structure");

		ROS_INFO("Waiting for GetBBoxMarker Service to be advertised.");
		ros::service::waitForService("get_bbox_marker");
		ROS_INFO("Found GetBboxMarker Service!");
		get_bbox_marker_client_ = nh.serviceClient<postgis_control::GetBBoxMarker>("get_bbox_marker");

		ROS_INFO("Waiting for GetRoomBBoxes Service to be advertised.");
		ros::service::waitForService("get_room_bboxes");
		ROS_INFO("Found GetRoomBBoxes Service!");
		get_room_bboxes_client_ = nh.serviceClient<postgis_control::GetRoomBBoxes>("get_room_bboxes");

		ROS_INFO("Waiting for GetBBoxOfBBoxes Service to be advertised.");
		ros::service::waitForService("get_bbox_of_bboxes");
		ROS_INFO("Found GetBBoxOfBBoxes Service!");
		get_bbox_of_bboxes_client_ = nh.serviceClient<postgis_control::GetBBoxOfBBoxes>("get_bbox_of_bboxes");

		return true;
	}

	bool MapServer2D::setupParameters()
	{
		bool status = true;

		if (!nh.getParam("/map_frame", map_frame))
		{
			status = false;
			ROS_WARN("/map_frame parameter is missing.");
		}

		if (!nh.getParam("/dimension", dimension))
		{
			status = false;
			ROS_WARN("/dimension parameter is missing.");
		}

		if (!nh.getParam("/single_slice_value", single_slice_value))
		{
			status = false;
			ROS_WARN("/single_slice_value parameter is missing.");
		}

		if (!nh.getParam("/multi_slice_min_value", multi_slice_min_value))
		{
			status = false;
			ROS_WARN("/multi_slice_min_value parameter is missing.");
		}

		if (!nh.getParam("/multi_slice_max_value", multi_slice_max_value))
		{
			status = false;
			ROS_WARN("/multi_slice_max_value parameter is missing.");
		}

		if (!nh.getParam("/multi_slice_resolution", multi_slice_resolution))
		{
			status = false;
			ROS_WARN("/multi_slice_resolutio parameter is missing.");
		}

		if (!nh.getParam("/grid_resolution", grid_resolution))
		{
			status = false;
			ROS_WARN("/grid_resolution parameter is missing.");
		}

		if (!nh.getParam("/grid_unoccupied_default", grid_unoccupied_default))
		{
			status = false;
			ROS_WARN("/grid_unoccupied_default parameter is missing.");
		}

		if (!nh.getParam("/default_padding", default_padding))
		{
			status = false;
			ROS_WARN("/default_padding parameter is missing.");
		}

		if (!nh.getParam("/blocked_regions", blocked_regions))
		{
			status = false;
			ROS_WARN("/blocked_regions parameter is missing.");
		}
		else
		{
			ROS_ASSERT(blocked_regions.getType() == XmlRpc::XmlRpcValue::TypeArray); 

			for(int i = 0; i < blocked_regions.size(); i++)
			{
				if(blocked_regions[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
				{
					status = false;
					ROS_WARN("Parameter blocked_region %d is not a struct", i);
					continue;
				}

				XmlRpc::XmlRpcValue& blocked_region = blocked_regions[i];
				XmlRpc::XmlRpcValue& name = blocked_region["name"];

				if(name.getType() != XmlRpc::XmlRpcValue::TypeString)
				{
					status = false;
					ROS_WARN("The 'name' parameter in the blocked_region struct %d is not a string", i);
					continue;
				}

				XmlRpc::XmlRpcValue& padding = blocked_region["padding"];
				if(padding.getType() != XmlRpc::XmlRpcValue::TypeDouble)
				{
					status = false;
					ROS_WARN("The 'padding' parameter in the blocked_region struct %d is not a double", i);
					continue;
				}

				blocked_region_names.push_back(static_cast<string>(name));
				blocked_region_padding.push_back(static_cast<double>(padding));
			}
		}

		if (!nh.getParam("/mesh_regions", mesh_regions))
		{
			status = false;
			ROS_WARN("/mesh_regions parameter is missing.");
		}
		else
		{
			ROS_ASSERT(mesh_regions.getType() == XmlRpc::XmlRpcValue::TypeArray); 

			for(int i = 0; i < mesh_regions.size(); i++)
			{
				if(mesh_regions[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
				{
					status = false;
					ROS_WARN("Parameter mesh_region %d is not a struct", i);
					continue;
				}

				XmlRpc::XmlRpcValue& mesh_region = mesh_regions[i];
				XmlRpc::XmlRpcValue& name = mesh_region["name"];
				if(name.getType() != XmlRpc::XmlRpcValue::TypeString)
				{
					status = false;
					ROS_WARN("The 'name' parameter in the mesh_region struct %d is not a string", i);
					continue;
				}
				mesh_region_names.push_back(static_cast<string>(name));

				/*XmlRpc::XmlRpcValue& padding = mesh_region["padding"];
				if(padding.getType() != XmlRpc::XmlRpcValue::TypeDouble)
				{
					status = false;
					ROS_WARN("The 'padding' parameter in the blocked_region struct %d is not a double", i);
					continue;
				}
				blocked_region_padding.push_back(static_cast<double>(padding));
				*/
			}
		}
		
		mesh_region_names.clear();
		blocked_region_names.clear();

		return status;
	}

	//
	// SERVICES
	//

	bool MapServer2D::srvGetLocalizationMap(map_server_2d::GetLocalizationMap::Request& request,  map_server_2d::GetLocalizationMap::Response& response)
	{
		postgis_control::GetRoomFaces get_room_faces;
		get_room_faces.request.room_ids.push_back(request.room_id);
		// get all faces of the chosen room
		if (get_room_faces_client_.call(get_room_faces))
		{
			get_room_faces.response.mesh.mesh.header.frame_id = request.frame;
				std_msgs::ColorRGBA color;
				color.r = (float) 0.0;
				color.g = (float) 1.0;
				color.b = (float) 1.0;
				color.a = (float) 1.0;

			lvr_tools::GetSingleSliceGrid get_single_slice_grid;
			get_single_slice_grid.request.dimension = dimension;
			get_single_slice_grid.request.value = single_slice_value;
			get_single_slice_grid.request.grid_resolution = grid_resolution;
			get_single_slice_grid.request.grid_unoccupied_default = grid_unoccupied_default;
			get_single_slice_grid.request.mesh = get_room_faces.response.mesh.mesh;
			get_single_slice_grid.request.color = color;

			//extract 2d grid map
			if (get_single_slice_grid_client_.call(get_single_slice_grid))
			{
				get_single_slice_grid.response.grid.header.frame_id = request.frame;
				response.grid = get_single_slice_grid.response.grid;
				response.marker = get_single_slice_grid.response.marker;
			}
			else
			{
				ROS_ERROR("Error making GetSingleSliceGrid service call\n") ;
				return false;
			}
		}
		else
		{
			ROS_ERROR("Error making GetRoomFaces service call\n") ;
			return false;
		}
		return true;
	}

	bool MapServer2D::srvGetNavigationMap(map_server_2d::GetNavigationMap::Request& request,  map_server_2d::GetNavigationMap::Response& response)
	{
		std::vector<GridConstPtr> partial_grid_ptrs;
		std::vector<nav_msgs::OccupancyGrid> partial_grids;

		// get all meshes of desired regions in the chosen room, if any and turn them into a grid maps
		for (unsigned int j = 0; j < request.mesh_regions.size(); j++)
		{
			ROS_DEBUG("Search mesh of region %s in room %s...", request.mesh_regions[j].c_str(), request.room_id.c_str()) ;
			postgis_control::GetRegionFaces get_region_faces;
			get_region_faces.request.room_id = request.room_id;
			get_region_faces.request.region_label =  request.mesh_regions[j];

			nav_msgs::OccupancyGrid base_grid;
			if (get_region_faces_client_.call(get_region_faces))
			{
				get_region_faces.response.mesh.mesh.header.frame_id = request.frame;
				std_msgs::ColorRGBA color;
				color.r = (float) 1.0;
				color.g = (float) 0.1;
				color.b = (float) 0.5;
				color.a = (float) 1.0;

				ROS_DEBUG("Got %d faces for room %s.", get_region_faces.response.mesh.mesh.faces.size(), request.room_id.c_str()) ;
				lvr_tools::GetMultiSliceGrid get_multi_slice_grid;
				get_multi_slice_grid.request.dimension = dimension;
				get_multi_slice_grid.request.min_value = multi_slice_min_value;
				get_multi_slice_grid.request.max_value = multi_slice_max_value;
				get_multi_slice_grid.request.slicing_resolution = multi_slice_resolution;
				get_multi_slice_grid.request.grid_resolution = grid_resolution;
				get_multi_slice_grid.request.grid_unoccupied_default = grid_unoccupied_default;
				get_multi_slice_grid.request.mesh = get_region_faces.response.mesh.mesh;
				get_multi_slice_grid.request.color = color;

				if (get_multi_slice_grid_client_.call(get_multi_slice_grid))
				{
					ROS_DEBUG("Created grid from rooms %s.", request.room_id.c_str()) ;
					get_multi_slice_grid.response.grid.header.frame_id = request.frame;
					get_multi_slice_grid.response.grid.info.origin.orientation.w=1.0;
					geometry_msgs::Pose origin = get_multi_slice_grid.response.grid.info.origin;

					ROS_DEBUG("MAP GRID ORIGIN");
					ROS_DEBUG("(%f %f %f) (%f %f %f %f)",
															//current_transform.response.pose.header.frame_id.c_str(),
															origin.position.x,
															origin.position.y,
															origin.position.z,
															origin.orientation.x,
															origin.orientation.y,
															origin.orientation.z,
															origin.orientation.w);

					// store grid map in buffer
					partial_grids.push_back(get_multi_slice_grid.response.grid);

					///TODO remove publishing
					ROS_DEBUG("Publishing map on navmap") ;
					navmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map_server/navmap", 1);
					locmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map_server/locmap", 1);

					navmap_pub_.publish(get_multi_slice_grid.response.grid);
					navmap_marker_pub_.publish(get_multi_slice_grid.response.marker);
				}
				else
				{
					ROS_ERROR("Error making GetMultiSliceGrid service call\n") ;
					return false;
				}
			}
			else
			{
				ROS_ERROR("Error making GetRoomFaces service call\n") ;
				return false;
			}
		}

		
		std::vector<lvr_tools::BoundingBox> bboxes;
		// get all bounding boxes of desired regions in the chosen room, if any and turn them into a grid maps
		for (unsigned int k = 0; k < request.box_regions.size(); k++)
		{
			ROS_DEBUG("Search for bbox of type %s in room %s...", request.box_regions[k].c_str(), request.room_id.c_str()) ;

			postgis_control::GetRegionBBoxes get_region_bboxes;
			//get_region_bboxes.request.frame = request.frame;
			get_region_bboxes.request.region_label = request.box_regions[k];
			get_region_bboxes.request.room_id = request.room_id;

			if (get_region_bboxes_client_.call(get_region_bboxes))
			{
				ROS_DEBUG("Found %d BBoxes of blocked area type %s in room %s.", (int) get_region_bboxes.response.bboxes.size(), request.box_regions[k].c_str(), request.room_id.c_str()) ;
				if( get_region_bboxes.response.bboxes.size() > 0)
				{
				nav_msgs::OccupancyGrid blocked_areas_grid;
				blocked_areas_grid = fGetBlockedAreasGrid(get_region_bboxes.response.bboxes, request.frame, request.box_padding[k]);
				ROS_DEBUG("Created grid with padding %f.", request.box_padding[k]);
				// push grid maps into buffer
				partial_grids.push_back(blocked_areas_grid);
				geometry_msgs::Pose origin_ = blocked_areas_grid.info.origin;
				ROS_DEBUG("AREA GRID ORIGIN");

				ROS_DEBUG("(%f %f %f) (%f %f %f %f)",
											//current_transform.response.pose.header.frame_id.c_str(),
													origin_.position.x,
													origin_.position.y,
													origin_.position.z,
													origin_.orientation.x,
													origin_.orientation.y,
													origin_.orientation.z,
													origin_.orientation.w);
					/// TODO remove, when working
					ROS_DEBUG("Publishing map on locmap") ;
					locmap_pub_.publish(blocked_areas_grid);
					///
				}
			}
		}
		ROS_DEBUG("Got %d partial grid", partial_grids.size()) ;

		if(partial_grids.size() > 0)
		{
			// iterate the grid map buffer and convert them into the shared_pointers used in the occupancy_grid_utils::combineGrids() method
			for(unsigned int i = 0; i < partial_grids.size(); i++)
			{
				partial_grid_ptrs.push_back(boost::make_shared<nav_msgs::OccupancyGrid>(partial_grids[i]));
			}
			// combine grids into one and return result
			GridPtr combined_grid = combineGrids( partial_grid_ptrs, grid_resolution);
			
			ROS_DEBUG("Created grid from %d partial grids.", partial_grids.size()) ;
			combined_grid->header.frame_id = request.frame;
			response.grid = *combined_grid;
			geometry_msgs::Pose origin__ = response.grid.info.origin;
			ROS_DEBUG("RESULT GRID ORIGIN");
			ROS_DEBUG("(%f %f %f) (%f %f %f %f)",
												origin__.position.x,
												origin__.position.y,
												origin__.position.z,
												origin__.orientation.x,
												origin__.orientation.y,
												origin__.orientation.z,
												origin__.orientation.w);
			}
			return true;
	}

	bool MapServer2D::srvGetGlobalLocalizationMap(map_server_2d::GetGlobalLocalizationMap::Request& request,  map_server_2d::GetGlobalLocalizationMap::Response& response)
	{
		ROS_DEBUG("GetGlobalLocalizationMap SRV was called...");

		vector<string> room_labels;
		postgis_control::GetStructure get_structure;

		// get building structure
		if (get_structure_client_.call(get_structure))
		{
			// extract the desired floor
			for(unsigned int i = 0; i < get_structure.response.floors.size(); i++)
			{
				postgis_control::Floor floor;
				floor = get_structure.response.floors[i];

				// extract the desired floor
				if(floor.id.compare(request.floor_id) == 0)
				{
					// prepare a list of all contained rooms
					for (unsigned int j = 0; j < floor.rooms.size(); j++)
					{
						room_labels.push_back(floor.rooms[j].id);
					}
				}
			}
		}
		else
		{
			ROS_ERROR("Error making GetStructure service call\n") ;
			return false;
		}

		// get mesh of all rooms contained in the current floor

		postgis_control::GetRoomFaces get_room_faces;
		//get_room_faces.request.frame = request.frame;
		get_room_faces.request.room_ids = room_labels;
		// get all faces of the chosen floor's rooms
		if (get_room_faces_client_.call(get_room_faces))
		{
			get_room_faces.response.mesh.mesh.header.frame_id = request.frame;
				std_msgs::ColorRGBA color;
				color.r = (float) 0.0;
				color.g = (float) 1.0;
				color.b = (float) 1.0;
				color.a = (float) 1.0;
			
			lvr_tools::GetSingleSliceGrid get_single_slice_grid;
			get_single_slice_grid.request.dimension = dimension;
			get_single_slice_grid.request.value = single_slice_value;
			get_single_slice_grid.request.grid_resolution = grid_resolution;
			get_single_slice_grid.request.grid_unoccupied_default = grid_unoccupied_default;
			get_single_slice_grid.request.mesh = get_room_faces.response.mesh.mesh;
			get_single_slice_grid.request.color = color;
			//extract 2d grid map
			if (get_single_slice_grid_client_.call(get_single_slice_grid))
			{
				get_single_slice_grid.response.grid.header.frame_id = request.frame;
				response.grid = get_single_slice_grid.response.grid;
				response.marker = get_single_slice_grid.response.marker;
			}
			else
			{
				ROS_ERROR("Error making GetSingleSliceGrid service call\n") ;
				return false;
			}
		}
		else
		{
			ROS_ERROR("Error making GetRoomFaces service call\n") ;
			return false;
		}
		return true;
	}

	bool MapServer2D::srvGetGlobalNavigationMap(map_server_2d::GetGlobalNavigationMap::Request& request,  map_server_2d::GetGlobalNavigationMap::Response& response)
	{
		ROS_DEBUG("GetGlobalNavigationMap SRV was called...");

		std::vector<GridConstPtr> partial_grid_ptrs;
		std::vector<nav_msgs::OccupancyGrid> partial_grids;
		vector<string> room_labels;

		postgis_control::GetStructure get_structure;
		if (get_structure_client_.call(get_structure))
		{
			ROS_DEBUG("Got Structure, extract floor...");
			for(unsigned int i = 0; i < get_structure.response.floors.size(); i++)
			{
				postgis_control::Floor floor;
				floor = get_structure.response.floors[i];
				if(floor.id.compare(request.floor_id) == 0)
				{
					ROS_DEBUG("Found floor %s, extract rooms...",request.floor_id.c_str());
					// iterate rooms of current floor
					for (unsigned int j = 0; j < floor.rooms.size(); j++)
					{
						ROS_DEBUG("Calc nav for room %s...",floor.rooms[j].id.c_str());
						room_labels.push_back(floor.rooms[j].id);

						map_server_2d::GetNavigationMap get_navigation_map;
						get_navigation_map.request.room_id = floor.rooms[j].id;
						get_navigation_map.request.frame = request.frame;
						get_navigation_map.request.mesh_regions = request.mesh_regions;
						get_navigation_map.request.box_regions = request.box_regions;
						get_navigation_map.request.box_padding = request.box_padding;

						//extract 2d grid map
						if (srvGetNavigationMap(get_navigation_map.request, get_navigation_map.response))
						{
							ROS_DEBUG("Sucessfully created grid for floor: %s", get_navigation_map.request.room_id.c_str());
							// store map in buffer
							partial_grids.push_back(get_navigation_map.response.grid);
						}
						else
						{
							ROS_ERROR("Error making GetNavigationMap service call\n") ;
						}
					}
				}
			}
			// combine grids of all rooms into one grid for the floor
			ROS_DEBUG("Got %d partial grid", partial_grids.size()) ;
			if(partial_grids.size() > 0)
			{
				for(unsigned int i = 0; i < partial_grids.size(); i++)
				{
					partial_grid_ptrs.push_back(boost::make_shared<nav_msgs::OccupancyGrid>(partial_grids[i]));
				}

				GridPtr combined_grid = combineGrids( partial_grid_ptrs, grid_resolution);
				ROS_DEBUG("Created grid from %d partial grids.", partial_grids.size()) ;
				combined_grid->header.frame_id = request.frame;
				response.grid = *combined_grid;
				geometry_msgs::Pose origin__ = response.grid.info.origin;
				ROS_DEBUG("RESULT GRID ORIGIN");
				ROS_DEBUG("(%f %f %f) (%f %f %f %f)",
													origin__.position.x,
													origin__.position.y,
													origin__.position.z,
													origin__.orientation.x,
													origin__.orientation.y,
													origin__.orientation.z,
													origin__.orientation.w);
				}
				return true;
			}
			else
			{
				ROS_ERROR("Error making GetStructure service call\n") ;
				return false;
			}
	}

	//
	// FUNCTIONALITY
	//

	// move to lib
	nav_msgs::OccupancyGrid MapServer2D::fGetBlockedAreasGrid(vector<lvr_tools::BoundingBox> bboxes, string frame, double padding)
	{
		vector<GridConstPtr> grids;

		for(size_t i = 0; i < bboxes.size(); i++)
		{
			nav_msgs::OccupancyGrid grid = createOccupancyGrid(frame, ros::Time::now() , bboxes[i] , grid_resolution, grid_unoccupied_default, padding);
			boost::shared_ptr<nav_msgs::OccupancyGrid> grid_ptr = boost::make_shared<nav_msgs::OccupancyGrid>(grid);
			grids.push_back(grid_ptr);
		}

		GridPtr combined = combineGrids(grids, grid_resolution);
		combined->header.frame_id = frame;
		return *combined;
	}

	// move to lib
	nav_msgs::OccupancyGrid MapServer2D::createOccupancyGrid(string frame,
										ros::Time time,
										lvr_tools::BoundingBox bbox,
										double resolution,
										double unoccupied_default,
										double padding)
	{

		// determine 2d bounding area 
		// note the plane is called ij from here on
		// z-coord should be zero
		
		tf::Quaternion orientation(bbox.pose.pose.orientation.x,
									bbox.pose.pose.orientation.y,
									bbox.pose.pose.orientation.z,
									bbox.pose.pose.orientation.w);


		tf::Vector3 position(bbox.pose.pose.position.x,
							bbox.pose.pose.position.y,
							bbox.pose.pose.position.z);

		// to rotate the edges with padding
		tf::Transform transform(orientation);

		float bbox_width  = bbox.x_edge + 2 * padding;
		float bbox_height = bbox.y_edge + 2 * padding;

		tf::Vector3 minX (- padding, 0, 0);
		tf::Vector3 maxX (bbox.x_edge + padding, 0, 0);
		tf::Vector3 minY (0, - padding, 0);
		tf::Vector3 maxY (0, bbox.y_edge + padding, 0);

		/*ROS_INFO("minX x: %f y: %f", minX.x(), minX.y());
		ROS_INFO("maxX x: %f y: %f", maxX.x(), maxX.y());
		ROS_INFO("minY x: %f y: %f", minY.x(), minY.y());
		ROS_INFO("maxY x: %f y: %f", maxY.x(), maxY.y());*/

		tf::Vector3 a = transform * tf::Vector3(-padding, -padding, 0);
		tf::Vector3 b = transform * tf::Vector3(bbox.x_edge + padding , -padding, 0);
		tf::Vector3 c = transform * tf::Vector3(bbox.x_edge + padding , bbox.y_edge + padding, 0);
		tf::Vector3 d = transform * tf::Vector3(-padding, bbox.y_edge + padding, 0);
		
		tf::Vector3 rel_height = d-a;
		tf::Vector3 rel_width = b-a;
		
		/*ROS_INFO("a :  x: %f y: %f", a.x(), a.y());
		ROS_INFO("b :  x: %f y: %f", b.x(), b.y());
		ROS_INFO("c :  x: %f y: %f", c.x(), c.y());
		ROS_INFO("d :  x: %f y: %f", d.x(), d.y());*/
		
		//ROS_INFO("height:  x: %f y: %f", rel_height.x(), rel_height.y());
		//ROS_INFO("width :  x: %f y: %f", rel_width.x(), rel_width.y());

		float minXMap = std::min( std::min(a.x(), b.x() ), std::min( c.x(), d.x() ) );
		float minYMap = std::min( std::min(a.y(), b.y() ), std::min( c.y(), d.y() ) );
		float maxXMap = std::max( std::max(a.x(), b.x() ), std::max( c.x(), d.x() ) );
		float maxYMap = std::max( std::max(a.y(), b.y() ), std::max( c.y(), d.y() ) );

		//ROS_INFO("minx: %f maxx: %f \n miny: %f maxy: %f",minXMap, maxXMap, minYMap, maxYMap);

		tf::Vector3 minPos(minXMap, minYMap, 0);
		//ROS_INFO("minPos : x: %f y: %f", minPos.x(), minPos.y());

		// calculate the size of the map depending on the resolution. + 1 and +2 is an padding 
		int width  = static_cast<int>((maxXMap - minXMap) / resolution ) + 2;
		int height = static_cast<int>((maxYMap - minYMap) / resolution ) + 2;

		//ROS_INFO("width: %d height: %d", width, height);

		vector<signed char> data;
		data.resize(width * height, unoccupied_default); 


		float parts = std::ceil(rel_height.length() / (resolution *0.33)); 
		//ROS_INFO("segment x: %f y: %f", rel_height.x(), rel_height.y());
		//ROS_INFO("segment length: %f", rel_height.length());
		//ROS_INFO("resolution: %f", resolution);
		//ROS_INFO("parts: %f", parts);
		
		tf::Vector3 increment = rel_height / parts;
		//ROS_INFO("increment x: %f y: %f", increment.x(), increment.y());

		tf::Vector3 start, end;
		int i;
		//ROS_INFO("before loop");
		
		//ROS_INFO("start x: %f y: %f", (-minPos).x(), (-minPos).y());

		//ROS_INFO("end x: %f y: %f", (rel_width-minPos).x(), (rel_width-minPos).y());
		
		// minPos less then or equal zero
		for(i = 0, start = -minPos , end = rel_width - minPos;
			i < parts ;
			start += increment, end += increment, i++)
		{
			try
			{
				int x1 = static_cast<int>( start.x() / resolution );
				int y1 = static_cast<int>( start.y() / resolution );
				int x2 = static_cast<int>( end.x()   / resolution );
				int y2 = static_cast<int>( end.y()   / resolution );
				//ROS_INFO("Draw from: %d %d to %d %d", x1, y1, x2, y2);
				drawLineBresenham(x1, y1, x2, y2, data, width);
			}
			catch(std::out_of_range e)
			{
				cout << e.what()<<endl;
			}
		}
		//ROS_INFO("after loop");
		geometry_msgs::Pose origin;
		origin.position.x = minXMap + position.x();
		origin.position.y = minYMap + position.y();//
		origin.orientation.x = 0;
		origin.orientation.y = 0;
		origin.orientation.z = 0;
		origin.orientation.w = 1;

		nav_msgs::OccupancyGrid result;
		result.info.map_load_time = time;
		result.info.resolution = resolution;
		result.info.origin = origin; 
		result.info.width = width;
		result.info.height = height; 
		result.data = data;
		result.header.frame_id = frame;
		result.header.stamp = ros::Time::now();

		return result;
	}

	// move to lib
	void MapServer2D::drawLineBresenham(int x1, int y1, int x2, int y2, vector<signed char>& data, int width)
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

}

// Let the magic happen...

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_server_2d");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	map_server_2d::MapServer2D node(n);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
