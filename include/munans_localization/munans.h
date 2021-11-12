//====================================================================================
// Name        : munans.h
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : The library consisting of the essential functions used in the system
//====================================================================================

#ifndef MUNANS_H
#define MUNANS_H

#include <iostream>
#include <string>
#include <array>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <fstream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include "quickmath.h"
#include "Log.h"

namespace munans {
/* Data classes*/
/*/////////////*/

/* Class of poses data
 * 		ID -  ID of the location (node)
 * 		x - x coordinate
 * 		y - y coordinate
 * 		z - z coordinate
 *	 	thx - rotation about x-axis
 * 		thy - rotation about y- axis
 * 		thz - rotation about z-axis

 * get () - Function to obtain and store pose data
 * 		data - a vector containing ID, x , y , z , thx , thy , thz
 */
class Pose {
public:
	int ID = 0;
	double x = 0;
	double y = 0;
	double z = 0;
	double thx = 0;
	double thy = 0;
	double thz = 0;

	void get(double *data);
};

/* Class of Odomtery data
 * 		IDprevious - ID of the previous location (node)
 * 		IDcurrent - ID of the current location (node)
 * 		x - translation along the x-axis
 * 		y - translation along the y-axis
 * 		z - translation along the z-axis
 * 		thx - roll
 * 		thy - pitch
 * 		thz - yaw
 *
 * get () - Function to obtain and store odometry data
 * 		data - a vector containing IDprevious , IDcurrent, x , y , z , thx , thy , thz
 */
class Odometry {
public:
	int IDprevious = 0;
	int IDcurrent = 0;
	double x = 0;
	double y = 0;
	double z = 0;
	double thx = 0;
	double thy = 0;
	double thz = 0;

	void get(double *data);
};

/* Class of visual feedback data
 * 		ID -  ID of the location (node)
 * 		x - x coordinate
 * 		y - y coordinate
 * 		z - z coordinate
 * 		thx - rotation about x-axis
 * 		thy - rotation about y- axis
 * 		thz - rotation about z-axis
 *
 * get () - Function to obtain and store visual feedback data
 * 		data - a vector containing ID, x , y , z , thx , thy , thz
 */
class Visual {
public:
	int ID = 0;
	double x = 0;
	double y = 0;
	double z = 0;
	double thx = 0;
	double thy = 0;
	double thz = 0;

	void get(double *data);
};

/*Class of map objects
 *		pose - Pose class object
 */
class Map {
public:

	Pose pose;
};

/*Class of robot objects
 *		pose - Pose class object
 *		odometry - Odometry class object
 *		visual - Visual class object
 */
class Robot {
public:

	Pose pose;
	Odometry odometry;
	Visual visual;
};

/* Data structures*/
/*////////////////*/

/* Data structure to store the Essential matrix data
 * 		t_x - x component of the translation vector
 * 		t_y - y component of the translation vector
 * 		t_z - z component of the translation vector
 * 		thx - rotaion about the x-axis
 * 		thy - rotaion about the y-axis
 * 		thz - rotaion about the z-axis
 */
struct EssentialMatrix {
	float t_x;
	float t_y;
	float t_z;
	float thx;
	float thy;
	float thz;
};

/* Data structure to store verification parameters of a second stage verification
 *	 	FundamentalMat - Fundamental matrix
 * 		Pmatch -
 *  	no_of_inliers - no of inliers
 */
struct verificationParams {
	cv::Mat FundamentalMat;
	double P_match;
	int no_of_inliers;
};

/* Data structure to store closest node data of a place recognition
 *	 	FundamentalMat - Fundamental matrix
 * 		ID - Identification parameter (Number of the node) of the closest node
 */
struct closestNode {
	cv::Mat FundamentalMat;
	int ID;
	bool success;
};

/* Data structure to store the feature points and descriptors of an image
 * 		kp -  Key points (Feature points)
 * 		des - Descriptors
 */
struct features {
	std::vector<cv::KeyPoint> kp;
	cv::Mat des;
};

/* Data structure to store the performance metrics of a learning algorithm
 * 		accuracy -  accuracy
 * 		precision - precision
 * 		recall - recall
 * 		sensitivity - sensitivity
 * 		specificity - specificity
 * 		F1-Score - F1 Score
 */
struct performanceMetrics {
	double accuracy;
	double precision;
	double recall;
	double sensitivity;
	double specificity;
	double F1_Score;
};

/* Data structure to store statistics
 * 		min - minimum
 * 		max - maximum
 * 		RMSE - Root Mean Square Error
 */
struct Statistics{
	double min;
	double max;
	double RMSE;
};

/* Data structure to store error statistics corresponding to position and orientation
 * 		Position - Error statistics of the orientation
 * 		Orientation - Error statistics of the orientation
 */
struct errorStats{
	Statistics Position;
	Statistics Orientation;
};

/* Evaluate functions*/
/*///////////////////*/

namespace Evaluate {

/*Function to calculate accuracy
 * 		TP - True Positive
 * 		TN - True Negative
 * 		FP - False Positive
 * 		FN - false Negative
 * 		returns accuracy value
 */
double accuracy(int TP, int TN, int FP, int FN);

/*Function to calculate precision
 * 		TP - True Positive
 * 		FP - False Positive
 * 		returns precision value
 */
double precision(int TP, int FP);

/*Function to calculate recall
 * 		TP - True Positive
 * 		FN - false Negative
 * 		returns recall value
 */
double recall(int TP, int FN);

/*Function to calculate sensitivity
 * 		TP - True Positive
 * 		FP - False Positive
 * 		FN - false Negative
 * 		returns sensitivity value
 */
double sensitivity(int TP, int FP, int FN);

/*Function to calculate specificity
 * 		FP - False Positive
 * 		TN - True Negative
 * 		returns specificity value
 */
double specificity(int FP, int TN);

/*Function to calculate F1-score
 * 		returns F1-score
 */
double F1Score(double precision, double recall);

/* confusionToMetrics() - Function to compute the performance metrics for a given confusion matrix
 * 		confusion_matrix - confusion matrix reshaped into a 1-D array
 * 		cols - no. of columns in the original matrix
 * 		query_index - index of the desired value
 * 		returns : accuracy , precision , recall , sensitivity , specificity , F1_Score
 */
performanceMetrics confusionToPerformance(Qmath::Matrix<int> confusion_matrix,
		int query_index);

/* isPresent() - Function to check the presence/absence of a query node among the putative matches
 * 		node - the desired node
 * 		putatives - vector consisting of the putative matches
 * 		returns : "Present" / "Absent"
 */
std::string isPresent(int node, std::vector<int> putatives);

}
;

/* Visual Vocabulary functions*/
/*////////////////////////////*/

namespace VisualVocabulary {
//	 void makeDictionary(double* data, std::string& file_name, std::string& sheet_name);
//
//
//	 void getDictionary(std::string& file_name, std::string& sheet_name);

/* isGoodMatch() - Function to determine whether a matching Visual Word ia a good match
 * 		distance_closest - distance to the closest match
 * 		distance_next_closest - distance to the second closest match
 * 		match_thresh - matching threshold
 * 		returns : 0 - bad match , 1 - good match
 */
int isGoodMatch(double distance_closest, double distance_next_closest,
		double match_thresh);

/* tfIdf() - Function to compute the tf-idf scores for the voting array
 * 		scores - array consisting of the scores reshaped into a 1-D array
 * 		cols - no. of columns in the original 2-D array
 * 		returns : weighted votes for each node
 */
void tfIdf(Qmath::Matrix<double> scores, std::vector<double> &similarityScores);

/* Function to compute the normalized likelihood for a given distribution
 * 		z - distribution of values
 * 		returns : normalized Likelihood values for each value in z
 */
void normalizeLikelihood(Qmath::Vector<double> &z);
}
;

/* Debugging functions*/
/*////////////////////*/

namespace Debug {

/* Function to plot matching features
 * 		image - image on which the points are to be plotted in the form of a openCV matrix
 * 		points - feature points to be plotted in the form of an openCV keypoint vector
 * 		color - colour of the points
 * 		returns : an openCV matrix with the points plotted on it
 */
cv::Mat plotColouredFeatures(cv::Mat image, std::vector<cv::KeyPoint> &points,
		char colour);

/* debugImage() - Function to draw corresponding feature matches and singular features between two images
 * 		image1 - First image in the form of an openCV matrix
 * 		image2 - Second image in the form of an openCV matrix
 * 		keypoints1 - Feature points of the first image in the form of a OpenCV keypoint vector
 * 		keypoints1 - Feature points of the second image in the form of a OpenCV keypoint vector
 * 		des1 - Descriptors of the first image in the form of an openCV matrix
 * 		des2 - Descriptors of the second image in the form of an openCV matrix
 * 		returns : an openCV matrix with the matching features marked
 */
cv::Mat debugImage(cv::Mat image1, std::vector<cv::KeyPoint> keypoints1,
		cv::Mat des1, cv::Mat image2, std::vector<cv::KeyPoint> keypoints2,
		cv::Mat des2);

/* plotInliersOutliers() - Function to draw corresponding feature matches and singular features between two images
 * 		image1 - First image in the form of an openCV matrix
 * 		image2 - Second image in the form of an openCV matrix
 * 		keypoints1 - Feature points of the first image in the form of a OpenCV keypoint vector
 * 		keypoints1 - Feature points of the second image in the form of a OpenCV keypoint vector
 * 		des1 - Descriptors of the first image in the form of an openCV matrix
 * 		des2 - Descriptors of the second image in the form of an openCV matrix
 * 		match_mask - Inliers and outliers between image1 and image2 in the form of a Opencv Matrix
 * 		returns : an openCV matrix with inliers marked green and outliers in red
 */
cv::Mat plotInliersOutliers(cv::Mat image1,
		std::vector<cv::KeyPoint> keypoints1, cv::Mat des1, cv::Mat image2,
		std::vector<cv::KeyPoint> keypoints2, cv::Mat des2,cv::Mat match_mask);
		

/* showImage() - Function to display an image in a named window
 * 		window_name - desired name for the image window
 * 		image - image to be displayed as an openCV matrix
 */
void showImage(std::string window_name, cv::Mat image);

}
;

/* Place Recognition functions*/
/*////////////////////////////*/

namespace PlaceRecognition {

/* captureImage() - Function to capture images
 * 		image - image on which the points are to be plotted in the form of a openCV matrix
 * 		returns : image in the form of an openCV matrix
 */
cv::Mat captureImage(int image_ID);

/* getSIFTdes() - Function extract SIFT descriptors from a given image
 * 		image - input image in the form of an openCV matrix
 * 		no_of_features - desired no. of features
 * 		returns : descriptors in the form of an openCV matrix
 */
munans::features getSIFTdes(cv::Mat image, int no_of_features);

/* getDenseSIFTdes() - Function extract Dense SIFT descriptors from a given image
 * 		image - input image in the form of an openCV matrix
 * 		step_size - space between keypoints in pixels
 * 		returns : descriptors in the form of an openCV matrix
 */
munans::features getDenseSIFTdes(cv::Mat image, int step_size);

// /* getASIFTdes() - Function extract Affine SIFT descriptors from a given image
//  * 		image - input image in the form of an openCV matrix
//  * 		no_of_features - desired no. of features
//  * 		returns : descriptors in the form of an openCV matrix
//  */

// munans::features getASIFTdes(cv::Mat image,int no_of_features);

/* getSURFdes() - Function extract SURF descriptors from a given image
 * 		image - input image in the form of an openCV matrix
 * 		no_of_features - desired no. of features
 * 		returns : descriptors in the form of an openCV matrix
 */
munans::features getSURFdes(cv::Mat image, int no_of_features);

/* getBRIEFdes() - Function extract BRIEF descriptors from a given image
 * 		image - input image in the form of an openCV matrix
 * 		no_of_features - desired no. of features
 * 		returns : descriptors in the form of an openCV matrix
 */
munans::features getBRIEFdes(cv::Mat image, int no_of_features);

/* getBRIEFVV() - Function extract BRIEF descriptors from a given image upon generating virtual views
 * 		image_name - name of the input image
 * 		tilt - tilt angle
 * 		longitude -  longitude angle
 * 		returns : descriptors in the form of an openCV matrix
 */
munans::features getBRIEFVV(std::string image_name, int tilt, int longitude);

/* countMatches() - Function to count the no of matches between two sets of descriptors
 * 		des1 - first set of descriptors in the form of an openCV matrix
 * 		des2 - second set of descriptors in the form of an openCV matrix
 * 		returns : the no. of matches
 */
int countMatches(cv::Mat des1, cv::Mat des2);

/* verifyGeometric() - Function to perform a RANSAC based geometric verification between two given images
 * 		query_image - query image in the form of an openCV matrix
 * 		putative_image - putative image in the form of an openCV matrix
 * 		returns :
 *	  		FundamentalMatrix in the form of an openCV matrix
 *			P_match
 *			the no_of_inliers
 */
munans::verificationParams verifyGeometric(cv::Mat query_image,
		cv::Mat putative_image);

/* getPutative() - Function to extract putatives from a given set of nodes
 * 		vec - voting array of a query node
 * 		no_of_putatives - desired no. of putative matches
 * 		returns : a vector containing the node ID's of the putative matches
 */
void getPutative(Qmath::Vector<int> vec, int no_of_putatives,
		std::vector<int> &putatives);

/* computeRotationTranslation() - Function to compute rotation matrix and translation vector for a given essential matrix
 * 		EssentialMat - Essential matrix in the form of an openCV matrix
 * 		returns : one possible Rotation matrix, another possible rotation matrix , translation vector upto a scale
 */
EssentialMatrix computeRotationTranslation(cv::InputArray EssentialMat);

/* EssentialMatrix() - Function to compute the essential matrix for a given Fundamental matrix and camera matrix
 * 		FundMat - fundamental matrix in the form of an openCV matrix
 * 		K1 - First camera matrix in the form of an openCV matrix
 * 		K2 - Second camera matrix in the form of an openCV matrix
 * 		returns : Essential matrix in the form of an openCV matrix
 */
//This function won't be necessary if the images coordinated are .Then the camera matrices becomes identity. Hence E = F
cv::Mat getEssentialMatrix(cv::InputArray FundMat, cv::InputArray K1,
		cv::InputArray K2);

/* Detect() - Detect the closest node to a given query image
 * 		image - image of the query node
 * 		returns : the Fundamental Matrix and the ID of the closest node
 */
closestNode Detect(cv::Mat img_sub,std::string containerPath);
}
;

}
;

#endif
