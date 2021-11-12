//ROS
#include <ros/ros.h>
#include <ros/package.h>

//C++
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Geometry>

//Custom made
#include "munans.h"
#include "quickmath.h"
#include "Log.h"

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"factor_graph");
	ros::NodeHandle factorGraph;
	tf::TransformBroadcaster robot_pose_broadcaster;

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

// 2) Importing Data
	 ///////////////

    // 2.1) Accessing data files

		std::ifstream MapData;
		std::ifstream OdometryData;
		std::ifstream VisualData;
		std::ifstream PathData;

		MapData.open(filePath + "/src/Data/map_3D.txt");
		OdometryData.open(filePath + "/src/Data/odometry_3D.txt");
		VisualData.open(filePath + "/src/Data/visual_3D.txt");
		PathData.open(filePath + "/src/Data/path_3D.txt");

	// Temporary data holders

		double mapdata[7]={};
		double pathdata[7]={};
		double odometrydata[8]={};
		double visualdata[7]={};

	// 2.2.) Acquiring data

	//Map

		if (MapData.is_open()){

			FG_log.info("Map data file opened successfully!\n");

			int i = 0;

			while(MapData >> mapdata[0] >> mapdata[1] >> mapdata[2] >> mapdata[3] >> mapdata[4] >> mapdata[5] >> mapdata[6]){

				map[i].pose.get(mapdata);
				i++;
			};
		}
		else{

			FG_log.error("Unable to read file!\n");
		};

	//Robot odometry

		if (OdometryData.is_open()){

			FG_log.info("Odometry data file opened successfully!\n");

			int i = 0;

			while(OdometryData >> odometrydata[0] >> odometrydata[1] >> odometrydata[2] >> odometrydata[3] >> odometrydata[4] >> odometrydata[5] >> odometrydata[6] >> odometrydata[7]){

				robot[i].odometry.get(odometrydata);
				i++;
			};
		}
		else{

			FG_log.error("Unable to read file!\n");
		};

	//Robot path

		if (PathData.is_open()){

			FG_log.info("Path data file opened successfully!\n");

			int i = 0;

			while(PathData >> pathdata[0] >> pathdata[1] >> pathdata[2] >> pathdata[3] >> pathdata[4] >> pathdata[5] >> pathdata[6]){

				robot[i].pose.get(pathdata);
				i++;
			};
		}
		else{

			FG_log.error("Unable to read file!\n");
		};

	//Visual feedback

		if (VisualData.is_open()){

			FG_log.info("Visual data file opened successfully!\n");

			int j = 0;

			while(j<no_of_way_points){

				if ((j != origin) & ((j-origin)%feedback_interval ==0)){

					int feedback_node = (j-origin)/feedback_interval;

					VisualData >> visualdata[0] >> visualdata[1] >> visualdata[2] >> visualdata[3] >> visualdata[4] >> visualdata[5] >> visualdata[6];

					robot[j].visual.get(visualdata);

				};

				j++;
			};
		}
		else{

			FG_log.error("Unable to read file!\n");
		};

	return 0;
};		