//==============================================================================
// Name        : cam2cam.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : A code for obtaining the relative pose between two cameras when
//               the poses of the cameras relative to two reference frames and
//               the transformation between the reference frames are known
//==============================================================================

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
 											HEADER FILES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//C++
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>

//Custom made
#include "munans.h"
#include "Log.h"

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											CONTROL VARIABLES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

const float PI = 3.141593;

// Camera frame w.r.t robot's body frame
float t12_x = 0.5;
float t12_y = 0.0;
float t12_z = 0.458;
float th12_x = -90.0;
float th12_y = 0.0;
float th12_z = 120 - 90.0;

// map camera frame w.r.t robot's camera frame
float t23_x = 0.000000;
float t23_y = -0.000975;
float t23_z = 1.000000;
float th23_x = 4.615869;
float th23_y = 10.460597;
float th23_z = 5.898056;

// Map camera frame w.r.t map frame
float t34_x = 0.0;
float t34_y = 0.0;
float t34_z = 1.4;
float th34_x = -90.0;
float th34_y = 0.0;
float th34_z = 120 - 90.0;

// Robot to Map transformation
float t14_x = 0.0;
float t14_y = 0.0;
float t14_z = 0.0;
tfScalar th14_x = 0.0;
tfScalar th14_y = 0.0;
tfScalar th14_z = 0.0;

// Map to Robot transformation
float t41_x = 0.0;
float t41_y = 0.0;
float t41_z = 0.0;
tfScalar th41_x = 0.0;
tfScalar th41_y = 0.0;
tfScalar th41_z = 0.0;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

Log::log CC_log; //Log object

tf::Vector3 t_12, t_23, t_34, t_14, t_41;
tf::Matrix3x3 R_12, R_23, R_34, R_14, R_41;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

float to_Degrees(float angle){
    angle = tfDegrees(angle);
    if (abs(angle) >= 360){
        angle = ((int)angle%360);
    };

    return angle;
};

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

	CC_log.info("Camera to camera transformation launched!");

    std::string filePath = ros::package::getPath("munans_localization");

    // 1.Camera 1 pose w.r.t to reference frame 1

        // 1.1.Translation vector
        t_12.setX(t12_x);
        t_12.setY(t12_y);
        t_12.setZ(t12_z);

        // 1.2.Rotation matrix
        R_12.setRPY(tfRadians(th12_x),tfRadians(th12_y),tfRadians(th12_z));

    // 2.Camera 1 pose w.r.t to reference frame 1

        // 2.1.Translation vector
        t_23.setX(t23_x);
        t_23.setY(t23_y);
        t_23.setZ(t23_z);

        // 2.2.Rotation matrix
        R_23.setRPY(tfRadians(th23_x),tfRadians(th23_y),tfRadians(th23_z));

    // 3.Camera 1 pose w.r.t to reference frame 1

        // 3.1.Translation vector
        t_34.setX(t34_x);
        t_34.setY(t34_y);
        t_34.setZ(t34_z);

        // 3.2.Rotation matrix
        R_34.setRPY(tfRadians(th34_x),tfRadians(th34_y),tfRadians(th34_z));

    // 4.Calulation

        // 4.1.Translation vector
        t_14 = t_12 + R_12 * t_23 - R_12 * R_23 *R_34.transpose() * t_34;

        // 4.2.Rotation matrix
        R_14 = R_12 * R_23 * R_34.transpose();

        // 4.3 Decomposing the translation vector rotation matrix

        // 4.3.1 Camera 1 to camera 2
            t14_x = t_14.getX();
            t14_y = t_14.getY();
            t14_z = t_14.getZ();

            R_14.getRPY(th14_x,th14_y,th14_z);

        // 4.3.1 Camera 1 to camera 2

            R_41 = R_14.transpose();
            t_41 = R_14.transpose() * (-t_14);

            t41_x = t_41.getX();
            t41_y = t_41.getY();
            t41_z = t_41.getZ();

            R_41.getRPY(th41_x,th41_y,th41_z);

    // 5.Printing the results

        CC_log.println("\nTransformation of Map w.r.t Robot");
        CC_log.println("-----------------------------------------\n");

        CC_log.info("t14_x = " + std::to_string(t14_x));
        CC_log.info("t14_y = " + std::to_string(t14_y));
        CC_log.info("t14_z = " + std::to_string(t14_z));
        CC_log.info("th14_x = " + std::to_string(tfDegrees(th14_x)));
        CC_log.info("th14_y = " + std::to_string(tfDegrees(th14_y)));
        CC_log.info("th14_z = " + std::to_string(tfDegrees(th14_z)));

        // CC_log.println("\nTransformation of Robot w.r.t Map");
        // CC_log.println("-----------------------------------------\n");        

        // CC_log.info("t41_x = " + std::to_string(t41_x));
        // CC_log.info("t41_y = " + std::to_string(t41_y));
        // CC_log.info("t41_z = " + std::to_string(t41_z));
        // CC_log.info("th41_x = " + std::to_string(tfDegrees(th41_x)));
        // CC_log.info("th41_y = " + std::to_string(tfDegrees(th41_y)));
        // CC_log.info("th41_z = " + std::to_string(tfDegrees(th41_z)));

};