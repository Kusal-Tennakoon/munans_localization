//============================================================================
// Name        : factor_graph.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : ROS node responsible for building and solving the factor graph
//============================================================================

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
 											HEADER FILES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Trigger.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "munans_localization/EssentialMatrixConstraint.h"

//C++
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Geometry>

// GTSAM

	//non-linear
	#include <gtsam/nonlinear/NonlinearFactorGraph.h>
	#include <gtsam/nonlinear/Values.h>
	#include <gtsam/nonlinear/Marginals.h>
	#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

	//inference
	#include <gtsam/inference/Symbol.h>
	#include <gtsam/inference/Key.h>

	//geometry
	#include <gtsam/geometry/Pose3.h>
	#include <gtsam/geometry/Point3.h>
	#include <gtsam/geometry/Rot3.h>
	#include <gtsam/geometry/EssentialMatrix.h>
	#include <gtsam/geometry/Unit3.h>

	//slam
	#include <gtsam/slam/PriorFactor.h>
	#include <gtsam/slam/BetweenFactor.h>
	#include <gtsam/slam/EssentialMatrixConstraint.h>

	//sam
	#include <gtsam/sam/BearingRangeFactor.h>

//OpenCV
#include <opencv2/opencv.hpp>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"

//Custom made
#include "munans.h"
#include "quickmath.h"
#include "Log.h"

#define DEFINE_GLOBALS // Prevents the variables getting redefined
#include "variables.h" // Contains all the variables related to visualization
#include "Visualizer.h"

//Namespaces
using namespace gtsam;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											CONTROL VARIABLES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

const double pi = 3.14;

const int origin = 1; //ID of the origin . Actual ID of the node -1.
const int no_of_nodes = 50; //No. of nodes
const int no_of_way_points = 45; //No. of nodes
const int feedback_interval = 5; //Interval at which visual feedback is taken
const int Simulation_delay = 1; //Simulation delay (seconds)
const int loop_rate = 100; //ROS loop rate (Hertz)
const int initial_noise_factor = 0.5;
int covariance_ellipse_scale = 1;
const double ground_clearance = 0.25;//0.25;
std::string correction = "off";
std::string param_name;

bool VFB_Display = false;

int map_node_id = 1;
int init_node_id = 1;
int traj_node_id = 1;
int odom_node_id = 2;
int visual_node_id = 1;
int vv_id = 1;
int arrow_no = 1;
// int node_id = 1;

// std::vector<std::vector<int>> feedback{{2,2},{7,7},{12,12},{16,16},{22,22},{28,29},{34,35},{42,43}};
// std::vector<std::vector<int>> feedback{{2,2},{7,7},{12,12},{16,16},{34,34},{42,42}};
// std::vector<std::vector<int>> feedback{{7,7},{12,12},{14,14},{17,18},{19,19},{22,22},{27,28},{33,35},{37,39},{39,41},{41,42},{43,45}};
// std::vector<std::vector<int>> feedback{{12,12},{19,19},{33,35},{39,41},{43,45}};
// std::vector<std::vector<int>> feedback{{2,2},{4,4},{6,6},{8,8},{10,10},{12,12},{14,14},{16,16},{18,18},{20,20},{22,22},{24,24},{26,26},{28,28},{30,30},{32,32},{34,34},{36,36},{38,38},{40,40},{42,42},{44,44}};
// std::vector<std::vector<int>> feedback{{2,2},{6,6},{8,8},{10,10},{14,14},{18,18},{20,20},{22,22},{24,24},{30,30},{32,32},{34,34},{36,36},{38,38},{44,44}};
std::vector<std::vector<int>> feedback{{2,2},{6,6},{8,8},{10,10},{14,14},{18,18},{20,20},{22,22},{24,24},{30,30},{32,32},{34,34},{36,36},{44,44}};
// std::vector<std::vector<int>> virtual_views{{15,17},{26,25},{35,37},{37,38},{39,42},{44,47}};
std::vector<std::vector<int>> virtual_views{{4,4},{12,12},{16,16},{26,27},{39,42}};//{26,27},{16,16},{26,25},{39,42}};

// std::vector<Pose3> vv_poses{Pose3(Rot3::RzRyRx(1.225496*pi/180,-0.557623*pi/180,9.042715*pi/180),Point3(0.114423,0.006128,-0.993413)),
//                             Pose3(Rot3::RzRyRx(-0.389511*pi/180,1.235215*pi/180,1.013144*pi/180),Point3(-0.866018,0.500014,-0.000000)),
//                             Pose3(Rot3::RzRyRx(-0.064135*pi/180,0.024092*pi/180,20.074999*pi/180),Point3(0.992234,-0.003024,-0.124345)),
//                             Pose3(Rot3::RzRyRx(-10.822292*pi/180,1.637628*pi/180,-2.349849*pi/180),Point3(-0.866025,-0.500000,-0.000000)),
// 							Pose3(Rot3::RzRyRx(3.458252*pi/180,0.217267*pi/180,-17.655217*pi/180),Point3(0.500000,-0.866025,0.000000))
//                             };  

// std::vector<Pose3> vv_poses{Pose3(Rot3::RzRyRx(1.225496*pi/180,-0.557623*pi/180,9.042715*pi/180),Point3(0.186104,0.007915,-0.982498)),
//                             Pose3(Rot3::RzRyRx(-0.389511*pi/180,1.235215*pi/180,1.013144*pi/180),Point3(-0.401354,0.509015,-0.761459)),
//                             Pose3(Rot3::RzRyRx(-0.064135*pi/180,0.024092*pi/180,20.074999*pi/180),Point3(0.992234,-0.003024,-0.124345)),
//                             Pose3(Rot3::RzRyRx(-10.822292*pi/180,1.637628*pi/180,-2.349849*pi/180),Point3(-0.866025,-0.500000,-0.000000)),
// 							Pose3(Rot3::RzRyRx(3.458252*pi/180,0.217267*pi/180,-17.655217*pi/180),Point3(0.500000,-0.866025,0.000000))
//                             };  				



			

std::vector<Pose3> vv_poses{Pose3(Rot3::RzRyRx(-0.0339524*pi/180,-2.621748*pi/180,1.118157*pi/180),Point3(-0.999279,0.000841,-0.037969)),
							Pose3(Rot3::RzRyRx(-6.409915*pi/180,-4.755079*pi/180,2.186194*pi/180),Point3(0.893299,-0.379870,-0.240240)),
							Pose3(Rot3::RzRyRx(-1.780183*pi/180,-4.702769*pi/180,2.811288*pi/180),Point3(0.476411,0.195571,-0.857196)),														
							Pose3(Rot3::RzRyRx(-5.665284*pi/180,-3.240220*pi/180,3.214727*pi/180),Point3(0.751677,0.178127,-0.635022)),							
							Pose3(Rot3::RzRyRx(0.708671*pi/180,-3.451330*pi/180,1.804527*pi/180),Point3(0.994320,0.020495,-0.104441)),                            							
                            }; 

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//0) Parameters
    ///////////

	const double eix = 0.001; //Noise in initial x
    const double eiy = 0.001; //Noise in initial y
    const double eiz = 0.001; //Noise in initial z
    const double eithx = 0.0001; //Noise in initial theta about x-axis
    const double eithy = 0.0001; //Noise in initial theta about y-axis
    const double eithz = 0.001*pi/3; //Noise in initial theta about z-axis

    const double ix = 0; // initial x of robot
    const double iy = 0; // initial y of robot
    const double iz = 0; // initial z of robot
    const double ithx = 0; // initial theta about x-axis of robot
    const double ithy = 0; // initial theta about y-axis of robot
    const double ithz = 0; // initial theta about z-axis of robot

    const double ex = 0.05; //Odometry noise in x
    const double ey = 0.05; //Odometry noise in y
    const double ez = 0.05; //Odometry noise in z
    const double ethx = 0.0001; //Odometry noise in theta about x-axis
    const double ethy = 0.0001; //Odometry noise in theta about y-axis
    const double ethz = 0.01*pi/3; //Odometry noise in theta about z-axis

    const double evx = 0.01 ; //Noise in visual x
    const double evy = 0.01; // Noise in visual y
    const double evz = 0.01; // Noise in visual z
    const double evthx = 0.01*pi/6; //Noise in visual theta about x-axis
    const double evthy = 0.01*pi/6; //Noise in visual theta about y-axis
    const double evthz = 0.001*pi/6; //Noise in visual theta about z-axis. This should be a high value

    // Noise models
    noiseModel::Diagonal::shared_ptr priorNoiseMap = noiseModel::Diagonal::Sigmas((Vector(6) << eithx,eithy,eithz,eix,eiy,eiz).finished());
    noiseModel::Diagonal::shared_ptr odomNoise = noiseModel::Diagonal::Sigmas((Vector(6) << ethx,ethy,ethz,ex,ey,ez).finished());
    noiseModel::Diagonal::shared_ptr priorNoisePose = noiseModel::Diagonal::Sigmas((Vector(6) << eithx,eithy,eithz,eix,eiy,eiz).finished());
    // noiseModel::Diagonal::shared_ptr visualNoise = noiseModel::Diagonal::Sigmas((Vector(6) << evthx,evthy,evthz,evx,evy,evz).finished());
	noiseModel::Diagonal::shared_ptr visualNoise = noiseModel::Diagonal::Sigmas((Vector(5) << evthx,evthy,evthz,evx,evy).finished());

    //Graph container
    NonlinearFactorGraph graph;

    // Initial value holders
	Values initial;

    // Creating object instances to store data
	Log::log FG_log; //Log object
	munans::Map map ; //Map object
	munans::Robot robot; //Robot object

	// Object arrays
	std::vector<Pose3> Map_poses;
	std::vector<Pose3> accel_poses;
	std::vector<Pose3> traj_poses;
	std::vector<Pose3> odom_poses;	
	std::vector<Pose3> result_poses;

	munans::errorStats errorStats;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


munans::errorStats computeErrorStats(std::vector<Pose3> result_poses,std::vector<Pose3> traj_poses){
	
	munans::errorStats stats;
	std::vector<double> pos_errors, rot_errors;
	double delta_x = 0,delta_y = 0,delta_z = 0,delta_t = 0,delta_th = 0,dt2 = 0,dth2 = 0;
	double dt_tot = 0;
	double dth_tot = 0;
	// Matrix33 delta_R;
	tf:: Quaternion q_sol, q_gt;

    pos_errors.clear();
    rot_errors.clear();

	for (int i=0 ; i<=traj_node_id-1;i++){

		delta_x = result_poses.at(i).translation().x() - traj_poses.at(i).translation().x();
		delta_y = result_poses.at(i).translation().y() - traj_poses.at(i).translation().y();
		delta_z = result_poses.at(i).translation().z() - traj_poses.at(i).translation().z(); 

		dt2 = pow(delta_x,2) + pow(delta_y,2) + pow(delta_z,2);
		dt_tot = dt_tot + dt2;;
		delta_t = sqrt(dt2);
		pos_errors.push_back(delta_t);

		q_sol.setW(result_poses.at(i).rotation().toQuaternion().w());
		q_sol.setX(result_poses.at(i).rotation().toQuaternion().x());
		q_sol.setY(result_poses.at(i).rotation().toQuaternion().y());
		q_sol.setZ(result_poses.at(i).rotation().toQuaternion().z());
		q_sol.normalize();						

		q_gt.setW(traj_poses.at(i).rotation().toQuaternion().w());
		q_gt.setX(traj_poses.at(i).rotation().toQuaternion().x());
		q_gt.setY(traj_poses.at(i).rotation().toQuaternion().y());
		q_gt.setZ(traj_poses.at(i).rotation().toQuaternion().z());
		q_gt.normalize();

		delta_th = std::abs(tfDegrees(q_sol.angle(q_gt)));
		dth_tot = dth_tot + std::pow(delta_th,2);

		rot_errors.push_back(delta_th);
	};

	stats.Position.min = *std::min_element(pos_errors.begin(),pos_errors.end());
	stats.Position.max = *std::max_element(pos_errors.begin(),pos_errors.end());
	stats.Position.RMSE = sqrt(dt_tot/pos_errors.size());
	stats.Orientation.min = *std::min_element(rot_errors.begin(),rot_errors.end());
	stats.Orientation.max = *std::max_element(rot_errors.begin(),rot_errors.end());;
	stats.Orientation.RMSE = sqrt(dth_tot/rot_errors.size());

	Log::printVec(result_poses);
	Log::printVec(traj_poses);


	return stats;	
};

void solveGraph(){

	// graph.print("\nFactor Graph");
	// initial.print("\nInitial Values");

// 4) Optimization using LM optimizer

	FG_log.info("Optimization started!\n");
	LevenbergMarquardtOptimizer optimizer(graph, initial);
	Values result = optimizer.optimize();
		
	Marginals marginals(graph, result);

	FG_log.info("Optimization complete!\n");
	
// 5) Print result
	  /////////////

  // 5.2) Printing optimized trajectory
  // ----------------------------------

 		// FG_log.println("\n Optimized Result ");
		// FG_log.println("-------------");

  		// result.print("solution");


  // 5.3) Printing Covariance Ellipses
  // ---------------------------------

		FG_log.println("\nSolution");
		FG_log.println("--------");

		result_poses.clear();

  		for (int i=1 ; i <=traj_node_id ; i++){

			FG_log.print("Robot pose " + std::to_string(i) + "\n");
			FG_log.print("---------------\n");
			FG_log.print("\n");

			print(Pose3(result.at<Pose3>(Symbol('r',i))).translation().x(),"x = \t");
			print(Pose3(result.at<Pose3>(Symbol('r',i))).translation().y(),"y = \t");
			print(Pose3(result.at<Pose3>(Symbol('r',i))).translation().z(),"z = \t");
			print(Pose3(result.at<Pose3>(Symbol('r',i))).rotation().xyz().x(),"thx = \t");
			print(Pose3(result.at<Pose3>(Symbol('r',i))).rotation().xyz().y(),"thy = \t");
			print(Pose3(result.at<Pose3>(Symbol('r',i))).rotation().xyz().z(),"thz = \t");

			FG_log.print("\n");
			print(marginals.marginalCovariance(Symbol('r',i)), "Covariance \n");
			FG_log.print("\n");

			result_poses.push_back(Pose3(result.at<Pose3>(Symbol('r',i))));
	  	
	  	};


  // 5.4) Plotting data in rviz
  // ---------------------------------

		Visualizer::clearOptimizedPose();
		// Pose3 temp_pose = Pose3(Rot3::identity(),Point3(0,0,0));

  		for (int i=1 ; i <=traj_node_id ; i++){

		    // Visualizer::collectOptimizedPose(Pose3(result.at<Pose3>(Symbol('r',i))),Pose3(initial.at<Pose3>(Symbol('r',i))),marginals.marginalCovariance(Symbol('r',i)),i);
			Visualizer::collectOptimizedPose(Pose3(result.at<Pose3>(Symbol('r',i))),traj_poses.at(i-1),marginals.marginalCovariance(Symbol('r',i)),i);
			// Visualizer::collectOptimizedPose(Pose3(result.at<Pose3>(Symbol('r',i))),temp_pose,marginals.marginalCovariance(Symbol('r',i)),i);
	  	};
		
		Visualizer::publishOptimized();

		errorStats = computeErrorStats(result_poses,traj_poses); 
	
		FG_log.println("\nError Statistics");
		FG_log.println("----------------");

		FG_log.println("\nPosition");
		FG_log.println("--------");

		FG_log.println("\n\tMin = " + std::to_string(errorStats.Position.min) + " m");
		FG_log.println("\tMax = " + std::to_string(errorStats.Position.max) + " m");	
		FG_log.println("\tRMSE = " + std::to_string(errorStats.Position.RMSE) + " m");			

		FG_log.println("\nOrientation");
		FG_log.println("-----------");

		FG_log.println("\n\tMin = " + std::to_string(errorStats.Orientation.min) + " deg");
		FG_log.println("\tMax = " + std::to_string(errorStats.Orientation.max) + " deg");	
		FG_log.println("\tRMSE = " + std::to_string(errorStats.Orientation.RMSE) + " deg");					

	}


void map_server_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

	Rot3 Ri = Rot3::quaternion(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z); //Rotation matrix 
	Point3 Ti = Point3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z); //Translation vector

	Map_poses.push_back(Pose3(Ri,Ti));

	graph.emplace_shared<PriorFactor<Pose3> >(Symbol('m',map_node_id), Pose3(Ri,Ti), priorNoiseMap);

	// 4.7.2) Initial values for the map nodes
	initial.insert(Symbol('m',map_node_id), Pose3(Ri,Ti));

	//Accumulating the map nodes

	Visualizer::collectMapNode(Pose3(Ri,Ti),map_node_id);				

	//Publishing the map
	Visualizer::publishMap();

	FG_log.info("Map node " + std::to_string(map_node_id) + " added!\n");

	map_node_id++;
  
}

void accelerometer_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

	Rot3 Ri = Rot3::quaternion(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z); //Rotation matrix 
	Point3 Ti = Point3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z+ground_clearance); //Translation vector	

	accel_poses.push_back(Pose3(Ri,Ti));	

	// Pose3 initialPose = Pose3(Ri,Ti).retract(initial_noise_factor*Eigen::Matrix<double,1,6>::Random());
	Pose3 initialPose = Pose3(Ri,Ti);
	initial.insert(Symbol('r',init_node_id), initialPose);

	// Puslishing the initial pose
	Visualizer::collectInitialPose(initialPose,init_node_id);
	Visualizer::publishInitial();

	FG_log.info("Robot node " + std::to_string(init_node_id)+ " initial value added!\n");
	FG_log.warning("init_node_id = " + std::to_string(init_node_id));
	init_node_id++;
	FG_log.warning("init_node_id = " + std::to_string(init_node_id));

}

void trajectory_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){


	Rot3 Ri = Rot3::quaternion(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z); //Rotation matrix 
	Point3 Ti = Point3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z+ground_clearance); //Translation vector	

	traj_poses.push_back(Pose3(Ri,Ti));	

	Pose3 trajectoryPose = Pose3(Ri,Ti);

	// Puslishing the initial pose
	Visualizer::collectTrajectoryPose(trajectoryPose,traj_node_id);	
	Visualizer::publishTrajectory();

	// Visualizer::publishRobotPose(trajectoryPose,ros::Time::now());
    if ((visual_node_id <= feedback.size()) && (traj_node_id == feedback.at(visual_node_id-1).at(0))){

        Pose3 rob_node = traj_poses.at(feedback.at(visual_node_id-1).at(0)-1);
        Pose3 map_node = Map_poses.at(feedback.at(visual_node_id-1).at(1)-1);

        Pose3 visual_pose = rob_node.inverse() * map_node;	
        Visualizer::collectVisualFeedback(accel_poses.at(feedback.at(visual_node_id-1).at(0)-1),Map_poses.at(feedback.at(visual_node_id-1).at(1)-1),arrow_no);
        Visualizer::publishVisualFeedback();
        EssentialMatrix EssentialMat = EssentialMatrix::FromPose3(visual_pose);	

        FG_log.info("Essential matrix calculated!");			

    // 3.7.4) Adding the Essential Matrix Constraint
        
        graph.emplace_shared<EssentialMatrixConstraint>(Symbol('r',feedback.at(visual_node_id-1).at(0)),Symbol('m',feedback.at(visual_node_id-1).at(1)),EssentialMat,visualNoise);
        // graph.emplace_shared<BetweenFactor<Pose3> >(Symbol('r',visual_node_id),Symbol('m',visual_node_id), visual_pose,visualNoise);
        FG_log.info("Robot node " + std::to_string(feedback.at(visual_node_id-1).at(0))+ " -> map node " + std::to_string(feedback.at(visual_node_id-1).at(1))+ " Essential Matrix Constraint added!");
        
        FG_log.warning("visual_node_id = " + std::to_string(visual_node_id));			
        visual_node_id++;
		arrow_no++;
        FG_log.warning("visual_node_id = " + std::to_string(visual_node_id));

        solveGraph();        

    }else if((vv_id <= virtual_views.size()) && (traj_node_id == virtual_views.at(vv_id-1).at(0))){

        Pose3 visual_pose = vv_poses.at(vv_id-1);	
        Visualizer::collectVisualFeedback(accel_poses.at(virtual_views.at(vv_id-1).at(0)-1),Map_poses.at(virtual_views.at(vv_id-1).at(1)-1),arrow_no);
        Visualizer::publishVisualFeedback();
        EssentialMatrix EssentialMat = EssentialMatrix::FromPose3(visual_pose);	

        FG_log.info("Essential matrix calculated!");			

    // 3.7.4) Adding the Essential Matrix Constraint
        
        graph.emplace_shared<EssentialMatrixConstraint>(Symbol('r',virtual_views.at(vv_id-1).at(0)),Symbol('m',virtual_views.at(vv_id-1).at(1)),EssentialMat,visualNoise);
        // graph.emplace_shared<BetweenFactor<Pose3> >(Symbol('r',visual_node_id),Symbol('m',visual_node_id), visual_pose,visualNoise);
        FG_log.info("Robot node " + std::to_string(virtual_views.at(vv_id-1).at(0))+ " -> map node " + std::to_string(virtual_views.at(vv_id-1).at(1))+ " Essential Matrix Constraint added!");

        FG_log.warning("vv_id = " + std::to_string(vv_id));			
        vv_id++;
		arrow_no++;
        FG_log.warning("vv_id = " + std::to_string(vv_id));
        //visual_node_id++;
        solveGraph();        

    };
	FG_log.warning("traj_node_id = " + std::to_string(traj_node_id));	
	traj_node_id++;
	FG_log.warning("traj_node_id = " + std::to_string(traj_node_id));		
}

void odometer_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

	if (odom_node_id > 1){	

		Rot3 Ri = Rot3::quaternion(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z); //Rotation matrix 
		Point3 Ti = Point3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z+ground_clearance); //Translation vector	

		odom_poses.push_back(Pose3(Ri,Ti));

		graph.emplace_shared<BetweenFactor<Pose3> >(Symbol('r',odom_node_id-1), Symbol('r',odom_node_id), Pose3(Ri,Ti), odomNoise);

		FG_log.info("Robot node " + std::to_string(odom_node_id-1)+ " -> " + std::to_string(odom_node_id) + " Odometry added!");
		FG_log.warning("odom_node_id = " + std::to_string(odom_node_id));
		odom_node_id++;
		FG_log.warning("odom_node_id = " + std::to_string(odom_node_id));
	}else{
		FG_log.warning("odom_node_id = " + std::to_string(odom_node_id));		
		odom_node_id++;
		FG_log.warning("odom_node_id = " + std::to_string(odom_node_id));
	};

}

// void camera_callback(const munans_localization::EssentialMatrixConstraint::ConstPtr &msg){

// 	// Rot3 Ri = Rot3::quaternion(msg->EssentialMatrix.orientation.w,msg->EssentialMatrix.orientation.x,msg->EssentialMatrix.orientation.y,msg->EssentialMatrix.orientation.z);
// 	// Point3 Ti = Point3(traj_poses.at(msg->RobotPoseID-1).x(),traj_poses.at(msg->RobotPoseID-1).y(),traj_poses.at(msg->RobotPoseID-1).z());
// 	// Pose3 visual_pose = Pose3(Ri,Ti);

// 	// Pose3 map_node = traj_poses.at(msg->RobotPoseID-1);
// 	// Pose3 rob_node = Map_poses.at(msg->MapPoseID-1);
// 	// Pose3 visual_pose = rob_node.inverse() * map_node;
// 	// Visualizer::collectVisualFeedback(traj_poses.at(msg->RobotPoseID-1),Map_poses.at(msg->MapPoseID-1),visual_node_id);
// 	// Visualizer::collectCameraPose(visual_pose,visual_node_id);
// 	// Visualizer::publishVisualFeedback();
// 	// Visualizer::publishCameraPose();

// 	EssentialMatrix EssentialMat = EssentialMatrix::FromPose3(visual_pose);

// 	FG_log.info("Essential matrix calculated!");

// // 3.7.4) Adding the Essential Matrix Constraint

// 	graph.emplace_shared<EssentialMatrixConstraint>(Symbol('r',msg->RobotPoseID),Symbol('m',msg->MapPoseID),EssentialMat,visualNoise);
// 	// graph.emplace_shared<BetweenFactor<Pose3> >(Symbol('r',visual_node_id),Symbol('m',visual_node_id), visual_pose,visualNoise);
// 	FG_log.info("Robot node " + std::to_string(msg->RobotPoseID)+ " -> map node " + std::to_string(msg->MapPoseID)+ " Essential Matrix Constraint added!");
// 	visual_node_id++;
// }


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"factor_graph");
	ros::NodeHandle factorGraph;

	ros::Rate loop_rate(loop_rate);
	FG_log.info("Factor Graph launched!");

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

	// Initializing the visualizer
	Visualizer::Initialize(factorGraph);

	// Get parameters
	//---------------
	bool ok1 = ros::param::get("/visual_feedback_toggle", correction);
	bool ok2 = ros::param::get("/covariance_ellipse_scale", covariance_ellipse_scale);

// 3) Factor Graph Creation
	 /////////////////////

  // 3.4) Building the graph
  // +++++++++++++++++++++++

    // 3.4.1) Adding Priors
	// --------------------

      // 3.4.1.1) Starting point of the trajectory
	  // -----------------------------------------
	        Rot3 Ri = Rot3::RzRyRx(0,0,0); //Rotation matrix 
	        Point3 Ti = Point3(0,0,0); //Translation vector
			
	        graph.emplace_shared<PriorFactor<Pose3> >(Symbol('r',1), Pose3(Ri,Ti), priorNoisePose);
			initial.insert(Symbol('r',1), Pose3(Ri,Ti));
			accel_poses.push_back(Pose3(Ri,Ti));
			// traj_poses.push_back(Pose3(Ri,Ti));
			// traj_node_id++;
			FG_log.warning("init_node_id = " + std::to_string(init_node_id));
			init_node_id++;
			FG_log.warning("init_node_id = " + std::to_string(init_node_id));
			// visual_node_id++;
			Visualizer::collectInitialPose(Pose3(Ri,Ti),1);		
			// Visualizer::collectTrajectoryPose(Pose3(Ri,Ti),1);
			Visualizer::publishInitial;
			// Visualizer::publishTrajectory;
	        FG_log.info("Robot prior added!\n");
	
	while(ros::ok()){
		
      // 3.4.1.2) Map_poses nodes
	  // ------------------

	        ros::Subscriber map_server_sub = factorGraph.subscribe("factor_graph/map_server", 1000, map_server_callback);

	  // 3.5) Adding trajectory values
	  // --------------------------

	        ros::Subscriber trajectory_sub = factorGraph.subscribe("factor_graph/trajectory", 1000, trajectory_callback);

	  // 3.5) Adding initial values
	  // --------------------------

	        ros::Subscriber accelerometer_sub = factorGraph.subscribe("factor_graph/accelerometer", 1000, accelerometer_callback);

	  // 3.6) Adding odometry
	  // --------------------

			ros::Subscriber odometer_sub = factorGraph.subscribe("factor_graph/odometer", 1000, odometer_callback);

	  // 3.7) Adding visual feedback
	  // ---------------------------

			// ros::Subscriber visualFB_sub = factorGraph.subscribe("factor_graph/visual_feedback", 1000, camera_callback);

	  // 3.8) Graph solver
	  // -----------------
	
			// ros::ServiceServer solver = factorGraph.advertiseService("factor_graph/solve_graph", solve);
			// ros::Subscriber solver = factorGraph.subscribe("solver/trigger", 1, graph_solver);

		ros::spin();
		// loop_rate.sleep();

	};

    return 0;
};

