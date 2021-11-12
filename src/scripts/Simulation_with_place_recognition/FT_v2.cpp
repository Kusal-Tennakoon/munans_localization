//============================================================================
// Name        : Function_Tests.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : Testing Functions
//============================================================================
#include <iostream>
#include <string>
#include <array>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <fstream>
#include <algorithm>
#include "munans.h"
#include "quickmath.h"
#include "Log.h"

using namespace std;

int origin = 3 - 1; //ID of the origin . Actual ID of the node -1.
int no_of_nodes = 47; //No. of node
int feedback_interval = 5; //Interval at which visual feedback is taken

int main() {

	munans::Map map; // Map object
	munans::Robot robot; //Robot object
	Log::log Log;
//    // Test matrix 1
//    Qmath::Matrix<int> TestMat(4,3);
//
//    double counter = 1;
//    for (int i = 0 ; i <= 3 ; i++){
//        for (int j = 0 ; j <= 2 ; j++){
//        	TestMat.Set(i,j,counter);
//        	counter++;
//        };
//    };
//
//    // Test matrix 2
//    Qmath::Matrix<int> TestMat2(4,3);
//
//     double arr[4][3] = {{10 , 5 , 12} , {8 , 4 , 9} , {7 , 3 , 2} , {1 , 6 , 11}};
//
//    for (int i = 0 ; i <= 3 ; i++){
//        for (int j = 0 ; j <= 2 ; j++){
//        	TestMat2.Set(i,j,arr[i][j]);
//        };
//    };
//
//	std::vector<int>  arr2{10 , 5 , 12 , 8 , 4 , 9 , 7 , 3 , 2 , 1 , 6 , 11};
//	Qmath::Vector<int> TestVec(arr2);
//
//
//
////  1 . Data import check
//
//    // Accessing data files
//
//		ifstream MapData;
//		ifstream OdometryData;
//		ifstream VisualData;
//		ifstream PathData;
//
//		MapData.open("/home/umbra/Documents/Autonomous_Trajectory_Correction/map_3D.txt");
//		OdometryData.open("/home/umbra/Documents/Autonomous_Trajectory_Correction/odometry_1_3D.txt");
//		VisualData.open("/home/umbra/Documents/Autonomous_Trajectory_Correction/visual_3D.txt");
//		PathData.open("/home/umbra/Documents/Autonomous_Trajectory_Correction/loop_1_3D.txt");
//
//	// Temporary data holders
//
//		double mapdata[] = {};
//		double pathdata[] = {};
//		double odometrydata[] = {};
//		double visualdata[] = {};
//
//	// Acquiring data
//
//	//Map
//
//		if (MapData.is_open()){
//
//			Log.info("Map data file opened successfully..");
//
//			while(MapData >> mapdata[0] >> mapdata[1] >> mapdata[2] >> mapdata[3] >> mapdata[4] >> mapdata[5] >> mapdata[6]){
//
//				map.pose.get(mapdata);
//			};
//		}
//		else{
//
//			Log.error("Unable to read file !");
//		};
//
//	//Robot odometry
//
//		if (OdometryData.is_open()){
//
//			Log.info("Odometry data file opened successfully..");
//
//			while(OdometryData >> odometrydata[0] >> odometrydata[1] >> odometrydata[2] >> odometrydata[3] >> odometrydata[4] >> odometrydata[5] >> odometrydata[6] >> odometrydata[7]){
//
//				robot.odometry.get(odometrydata);
//			};
//		}
//		else{
//
//			Log.error("Unable to read file !");
//		};
//
//	//Robot path
//
//		if (PathData.is_open()){
//
//			Log.info("Path data file opened successfully..");
//
//			while(PathData >> pathdata[0] >> pathdata[1] >> pathdata[2] >> pathdata[3] >> pathdata[4] >> pathdata[5] >> pathdata[6]){
//
//				robot.pose.get(pathdata);
//			};
//		}
//		else{
//
//			Log.error("Unable to read file !");
//		};
//
//		//Visual Data
//
//		if (VisualData.is_open()){
//
//			Log.info("Visual data file opened successfully..");
//
//					VisualData >> visualdata[0] >> visualdata[1] >> visualdata[2] >> visualdata[3] >> visualdata[4] >> visualdata[5] >> visualdata[6];
//
//					robot.visual.get(visualdata);
//			}
//		else{
//
//			Log.error("Unable to read file !");
//		};
//
////	2. Map and Robot objects check
//
//	cout << "map : " << map.pose.ID << " " << map.pose.x << " " << map.pose.y << " "  << map.pose.z << " " << map.pose.thx << " " << map.pose.thy << " " << map.pose.thz << endl << endl;
//
//	Log.info("Map object check successful !");
//
//	cout << "robot pose : " << robot.pose.ID << " " << robot.pose.x << " " << robot.pose.y << " "  << robot.pose.z << " " << robot.pose.thx << " " << robot.pose.thy << " " << robot.pose.thz << endl << endl;
//
//	Log.info("Robot object -pose instance check successful !");
//
//	cout << "odometry : " << robot.odometry.IDprevious << " " << robot.odometry.IDcurrent << " " << robot.odometry.x << " " << robot.odometry.y << " " << robot.odometry.z << " " << robot.odometry.thx << " " << robot.odometry.thy << " " << robot.odometry.thz << " " << endl << endl;
//
//	Log.info("Robot object -odometry instance check successful !");
//
//	cout  << "visual : " << robot.visual.ID << " " << robot.visual.x << " " << robot.visual.y << " " << robot.visual.z << " " << robot.visual.thx << " " << robot.visual.thy << " " << robot.visual.thz << endl << endl;
//
//	Log.info("Robot object -visual instance check successful !");
//
////	3. Log::toString function check
//
//	double a = 2;
//	double b = 3;
//
//	string text = std::to_string(a) + std::to_string(b);
//
//	if (text == "23") {
//
//		Log.info("toString() check successful !");
//	}
//	else{
//
//		Log.error("toString() check failed !");
//
//	};
//
////	4. Matrix::size() function check
//
//	if ((TestMat.rows() ==4)&&(TestMat.cols() ==3)) {
//
//		Log.info("size() check successful !");
//	}
//	else{
//
//		Log.error("size() check failed !");
//
//	};
//
//	Log.println("Size = " + std::to_string(TestMat.rows()) + " x " + std::to_string(TestMat.cols()));
//
////	5. Matrix::colSum function check
//
//	std::vector<double>sumR{6 , 15 , 24 , 33};
//
//	std::vector<int> RowSum;
//	TestMat.rowSum(RowSum);
//
//	if ((RowSum.at(0) == sumR[0]) && (RowSum.at(1) == sumR[1]) && (RowSum.at(2) == sumR[2]) && (RowSum.at(3) == sumR[3])){
//
//		Log.info("rowSum() check successful !");
//	}
//	else{
//
//		Log.error("rowSum() check failed !");
//
//	};
//
//	Log.print("Actual row sums = ");
//	Log::printVec(RowSum);
//	Log.println("");
//
//	Log.print("Caluculated row sums = ");
//	Log::printVec(RowSum);
//	Log.println("");
//
////	6. Matrix::colSum function check
//
//	std::vector<double>sumC{22 , 26 , 30};
//
//	std::vector<int> colSum;
//	TestMat.colSum(colSum);
//
//	if ((colSum.at(0) == sumC[0]) && (colSum.at(1) == sumC[1]) && (colSum.at(2) == sumC[2])){
//
//		Log.info("colSum() check successful !");
//	}
//	else{
//
//		Log.error("colSum() check failed !");
//
//	};
//
//	Log.print("Actual column sums = ");
//	Log::printVec(sumC);
//	Log.println("");
//
//	Log.print("Caluculated column sums = ");
//	Log::printVec(colSum);
//	Log.println("");
//
////	7. Matrix::sum function check
//
//
//	double sum = TestMat.sum();
//
//	if (sum == 78) {
//
//		Log.info("Matrix::sum() check successful !");
//	}
//	else{
//
//		Log.error("Matrix::sum() check failed !");
//		Log.println("sum = " + std::to_string(sum));
//
//	};
//
//	Log.println("Actual sum = 78");
//	Log.print("");
//
//	Log.print("Caluculated column sums = ");
//	Log.println(std::to_string(sum));
//
////	8. Matrix::max function check
//
//	std::vector<Qmath::Max<int>> maximums;
//
//	int no_of_maxs = 5;
//	TestMat2.max(no_of_maxs,maximums);
//
//	if (maximums.at(0).ID == 2){
//
//		Log.info("Matrix::max() check successful !");
//	}
//	else{
//
//		Log.error("Matrix::max() check failed !");
//
//	};
//
//	std::cout << "Value" << " | " << "ID" << endl;
//	std::cout << "--------------" << endl;
//	for (int i=0 ; i<=no_of_maxs-1 ; i++){
//		std::cout << maximums.at(i).value << "          |    " << maximums.at(i).ID << endl;
//	};
//
////	9. Matrix::mean function check
//
//	double mu=0;
//	double mean = 6.5;
//
//	mu = TestMat.mean();
//
//	if (mean == mu){
//
//		Log.info("Matrix::mean() check successful !");
//	}
//	else{
//
//		Log.error("Matrix::mean() check failed !");
//
//	};
//
//	Log.print("Actual mean = ");
//	Log.println(std::to_string(mean));
//
//	Log.print("Caluculated mean = ");
//	Log.println(std::to_string(mu));
//
////	10. Matrix::StdDev function check
//
//	double sigma = 0;
//	double std_dev = 3.4521;
//
//	sigma = TestMat.stdDev();
//
//	if (std::abs(sigma -std_dev)<0.001){
//
//		Log.info("Matrix::stdDev() check successful !");
//	}
//	else{
//
//		Log.error("Matrix::stdDev() check failed !");
//
//	};
//
//	Log.print("Actual stdDev = ");
//	Log.println(std::to_string(std_dev));
//
//	Log.print("Caluculated stdDev = ");
//	Log.println(std::to_string(sigma));
//
////	11. Vector::num function check
//
//		sum = TestVec.sum();
//
//		if (sum == 78) {
//
//			Log.info("Vector::sum() check successful !");
//		}
//		else{
//
//			Log.error("Vector::sum() check failed !");
//
//		};
//
//		Log.println("Actual sum = 78");
//		Log.print("");
//
//		Log.print("Caluculated column sums = ");
//		Log.println(std::to_string(sum));
//
////	12. Vector::max function check
//
//		no_of_maxs = 5;
//		TestVec.max(no_of_maxs,maximums);
//
//		if (maximums.at(0).ID == 2){
//
//			Log.info("Vector::max() check successful !");
//		}
//		else{
//
//			Log.error("Vector::max() check failed !");
//
//		};
//
//		std::cout << "Value" << " | " << "ID" << endl;
//		std::cout << "--------------" << endl;
//		for (int i=0 ; i<=no_of_maxs-1 ; i++){
//			std::cout << maximums.at(i).value << "          |    " << maximums.at(i).ID << endl;
//		};
//
////	13. Vector::mean function check
//
//		mean = 6.500;
//		mu = TestVec.mean();
//		if (mean == mu){
//
//			Log.info("Vector::mean() check successful !");
//		}
//		else{
//
//			Log.error("Vector::mean() check failed !");
//
//		};
//
//		Log.print("Actual mean = ");
//		Log.println(std::to_string(mean));
//
//		Log.print("Caluculated mean = ");
//		Log.println(std::to_string(mu));
//
////	14. Vector::stdDev function check
//
//		std_dev = 3.4521;
//
//		sigma = TestVec.stdDev();
//		if (std::abs(sigma -std_dev)<0.001){
//
//			Log.info("Vector::stdDev() check successful !");
//		}
//		else{
//
//			Log.error("Vector::stdDev() check failed !");
//
//		};
//
//		Log.print("Actual stdDev = ");
//		Log.println(std::to_string(std_dev));
//
//		Log.print("Caluculated stdDev = ");
//		Log.println(std::to_string(sigma));
//
//
////	15. isPresent function check
//
//		std::string state;
//		int node = 12;
//		std::vector<int> putatives{10 , 5 , 12 , 8 , 4 , 9 , 7 , 3 , 2 , 1 , 6 , 11};
//
//
//		state = munans::Evaluate::isPresent(node,putatives);
//
//		if (state == "Present")
//		{
//			Log.info("isPresent() check successful !");
//		}
//		else{
//
//			Log.error("isPrsesnt() check failed !");
//
//		};
//
//		Log.print("Node = ");
//		Log.print(std::to_string(node));
//		Log.println("");
//
//		Log.print("Putatives = ");
//		Log::printVec(putatives);
//		Log.println("");
//
//
//
////	16. confusionToPerformmance function check
//
//		Qmath::Matrix<int> confMat({{23,30,29,20,14,13},{11,19,12,15,35,25},{8,5,2,22,1,16},{9,21,28,4,6,18},{34,10,31,32,33,26},{17,3,36,7,24,27}});
//
//		munans::performanceMetrics performance;
//		performance = munans::Evaluate::confusionToPerformance(confMat, 2);
//
//		if (((performance.accuracy - 0.7177)<0.001)&&((performance.precision - 0.0145)<0.001)&&((performance.recall - 0.0370)<0.001)&&((performance.sensitivity - 0.0106)<0.001)&&((performance.specificity - 0.7778)<0.001)&&((performance.F1_Score - 0.0208)<0.001))
//		{
//			Log.info("confusionToPerformance() check successful !");
//		}
//		else{
//
//			Log.error("confusionToPerformance() check failed !");
//
//		};
//
//		Log.print("Accuracy = ");
//		Log.print(std::to_string(performance.accuracy));
//		Log.println("");
//		Log.print("Precision= ");
//		Log.print(std::to_string(performance.precision));
//		Log.println("");
//		Log.print("Recall = ");
//		Log.print(std::to_string(performance.recall));
//		Log.println("");
//		Log.print("Sensitivity = ");
//		Log.print(std::to_string(performance.sensitivity));
//		Log.println("");
//		Log.print("Specificity = ");
//		Log.print(std::to_string(performance.specificity));
//		Log.println("");
//		Log.print("F1-Score = ");
//		Log.print(std::to_string(performance.F1_Score));
//		Log.println("");
//
//
////	17. isGoodMatch function check
//
//		if ((munans::VisualVocabulary::isGoodMatch(5, 10,0.7)==1) && (munans::VisualVocabulary::isGoodMatch(8, 10,0.7)==0))
//		{
//			Log.info("isGoodMatch() check successful !");
//		}
//		else{
//
//			Log.error("isGoodMatch() check failed !");
//
//		};
//
////	18. tfIdf function check
//
//		Qmath::Matrix<double> scores({{0,0,1},{0,0,1},{1,1,0},{0,1,0},{1,0,1},{1,1,0}});
//		std::vector<double> similarity_scores;
//		munans::VisualVocabulary::tfIdf(scores,similarity_scores);
//
//		if (((similarity_scores.at(0) - 0) < 0.001) && ((similarity_scores.at(1) - 0.4055) < 0.001) && ((similarity_scores.at(2) - 0.8109) < 0.001))
//		{
//			Log.info("tfIdf() check successful !");
//		}
//		else{
//
//			Log.error("tfIdf() check failed !");
//
//		};
//		Log.print("Actual similarity scores = ");
//		std::vector<double> vect({0,0.4055,0.8109});
//		Log::printVec(vect);
//		Log.println("");
//
//		Log.print("Calculated similarity scores = ");
//		Log::printVec(similarity_scores);
//		Log.println("");
//
////	19. normalizeLikelihood function check
//
//		Qmath::Vector<double> norm_scores({1,2,3,4,5,6,7,8,9,10});
//		munans::VisualVocabulary::normalizeLikelihood(norm_scores);
//
//		if (((norm_scores.Get(8)-1.1141)<0.001)&&((norm_scores.Get(9)-1.2959)<0.001))
//		{
//			Log.info("normalizeLikelihood() check successful !");
//		}
//		else{
//
//			Log.error("normalizeLikelihood() check failed !");
//
//		};
//
//		Log.print("Actual normalized values = ");
//		std::vector<double> vect_norm({0,0,0,0,0,0,0,1.1141,1.2959});
//		Log::printVec(vect_norm);
//		Log.println("");
//
//		Log.print("Calculated normalized values = ");
//		norm_scores.print();
//		Log.println("");
//
////	20. getPutative function check
//
//		std::vector<int> selections;
//		Qmath::Vector<int> vec({10 , 5 , 12 , 8 , 4 , 9 , 7 , 3 , 2 , 1 , 6 , 11});
//
//		munans::PlaceRecognition::getPutative(vec,5,selections);
//
//		if ((selections.at(0)==2)&&(selections.at(1)==11)&&(selections.at(2)==0)&&(selections.at(3)==5)&&(selections.at(4)==3))
//		{
//			Log.info("getPutative() check successful !");
//		}
//		else{
//
//			Log.error("getPutative() check failed !");
//
//		};
//		Log.print("Actual putatives = ");
//		std::vector<int> vect2({2,11,0,5,3});
//		Log::printVec(vect2);
//		Log.println("");
//
//		Log.print("Calculated putatives = ");
//		Log::printVec(selections);
//		Log.println("");
//
////	21. showImage() function check
//
//		cv::Mat img = cv::imread("butterfly",cv::IMREAD_COLOR);
//
//		if( img.empty() ){
//			Log.error("Empty image !");
//		};
//
//		munans::Debug::showImage("Butterfly",img);
//		cv::waitKey(5000);
//		cv::destroyWindow( "Butterfly");
//
//		Log.info("showImage() check successful !");
//
////	22. getSIFTdes() function check
//
//		munans::features sift;
//
//		sift = munans::PlaceRecognition::getSIFTdes(img,100);
//
//		Log.info("getSIFTdes() check successful !");
//
////	23. getSURFdes() function check
//
//		munans::features surf;
//
//		surf = munans::PlaceRecognition::getSURFdes(img,100);
//
//		Log.info("getSURFdes() check successful !");
//
////	24. getBRIEFdes() function check
//
//		munans::features brief;
//
//		brief = munans::PlaceRecognition::getBRIEFdes(img,100);
//
//		Log.info("getBRIEFdes() check successful !");
//
////	25. countMatches() function check
//
//		munans::features box1_1 , box2_1 , box1_2 , box2_2 , box1_3 , box2_3;
//
//		cv::Mat img1 = cv::imread("box.png",cv::IMREAD_COLOR);
//		cv::Mat img2 = cv::imread("box_in_scene.png",cv::IMREAD_COLOR);
//
//		// I. For SIFT
//
//		Log.println("For SIFT \n-----------");
//				box1_1 = munans::PlaceRecognition::getSIFTdes(img1,100);
//				box2_1 = munans::PlaceRecognition::getSIFTdes(img2,100);
//
//				int no_of_matches_sift = munans::PlaceRecognition::countMatches(box1_1.des,box2_1.des);
//
//				Log.info("countMatches() check for SIFT successful !");
//
//				Log.print("No. of descriptors in box1 = ");
//				Log.println(std::to_string(box1_1.des.rows));
//				Log.print("No. of descriptors in box2 = ");
//				Log.println(std::to_string(box2_1.des.rows));
//				Log.print("No. of matches = ");
//				Log.println(std::to_string(no_of_matches_sift));
//				Log.println("");
//
//		// II. For SURF
//				Log.println("For SURF \n-----------");
//
//				box1_2 = munans::PlaceRecognition::getSURFdes(img1,100);
//				box2_2 = munans::PlaceRecognition::getSURFdes(img2,100);
//
//				int no_of_matches_surf = munans::PlaceRecognition::countMatches(box1_2.des,box2_2.des);
//
//				Log.info("countMatches() check for SURF successful !");
//
//				Log.print("No. of descriptors in box1 = ");
//				Log.println(std::to_string(box1_2.des.rows));
//				Log.print("No. of descriptors in box2 = ");
//				Log.println(std::to_string(box2_2.des.rows));
//				Log.print("No. of matches = ");
//				Log.println(std::to_string(no_of_matches_surf));
//				Log.println("");
//
//		// III. For BRIEF
//				Log.println("For BRIEF \n-----------");
//
//				box1_3 = munans::PlaceRecognition::getBRIEFdes(img1,100);
//				box2_3 = munans::PlaceRecognition::getBRIEFdes(img2,100);
//
//				int no_of_matches_brief = munans::PlaceRecognition::countMatches(box1_3.des,box2_3.des);
//
//				Log.info("countMatches() check for BRIEF successful !");
//
//				Log.print("No. of descriptors in box1 = ");
//				Log.println(std::to_string(box1_3.des.rows));
//				Log.print("No. of descriptors in box2 = ");
//				Log.println(std::to_string(box2_3.des.rows));
//				Log.print("No. of matches = ");
//				Log.println(std::to_string(no_of_matches_brief));
//				Log.println("");
//
////	26. plotColouredFeatures() function check
//
//				cv::Mat imgPlotCol;
//
//			// I. For SIFT
//
//				imgPlotCol = munans::Debug::plotColouredFeatures(img,sift.kp,'r');
//
//				munans::Debug::showImage("Butterfly_SIFT_keypoints",imgPlotCol);
//				cv::waitKey(5000);
//				cv::destroyWindow( "Butterfly_SIFT_keypoints");
//
//				Log.info("plotColouredFeatures() check for SIFT successful !");
//
//			// II. For SURF
//
//				imgPlotCol = munans::Debug::plotColouredFeatures(img,surf.kp,'g');
//
//				munans::Debug::showImage("Butterfly_SURF_keypoints",imgPlotCol);
//				cv::waitKey(5000);
//				cv::destroyWindow( "Butterfly_SURF_keypoints");
//
//				Log.info("plotColouredFeatures() check for SURF successful !");
//
//			// III. For BRIEF
//
//				imgPlotCol = munans::Debug::plotColouredFeatures(img,brief.kp,'b');
//
//				munans::Debug::showImage("Butterfly_BRIEF_keypoints",imgPlotCol);
//				cv::waitKey(5000);
//				cv::destroyWindow( "Butterfly_BRIEF_keypoints");
//
//				Log.info("plotColouredFeatures() check for BRIEF successful !");
//
////	27. debugImage() function check
//
//				cv::Mat imgDebug;
//
//			// I. For SIFT
//
//				imgDebug = munans::Debug::debugImage(img1,box1_1.kp, box1_1.des, img2, box2_1.kp, box2_1.des);
//
//				munans::Debug::showImage("box_SIFT_debug",imgDebug);
//				cv::waitKey(5000);
//				cv::destroyWindow( "box_SIFT_debug");
//
//				Log.info("debugImage() check for SIFT successful !");
//
//
//			// II. For SURF
//
//				imgDebug = munans::Debug::debugImage(img1,box1_2.kp, box1_2.des, img2, box2_2.kp, box2_2.des);
//
//				munans::Debug::showImage("box_SURF_debug",imgDebug);
//				cv::waitKey(5000);
//				cv::destroyWindow( "box_SURF_debug");
//
//				Log.info("debugImage() check for SURF successful !");
//
//
//
//			// III. For BRIEF
//
//				imgDebug = munans::Debug::debugImage(img1,box1_3.kp, box1_3.des, img2, box2_3.kp, box2_3.des);
//
//				munans::Debug::showImage("box_BRIEF_debug",imgDebug);
//				cv::waitKey(5000);
//				cv::destroyWindow( "box_BRIEF_debug");
//
//				Log.info("debugImage() check for BRIEF successful !");

//	28. verifyGeometric() function check

	munans::verificationParms verific_Params;

	cv::Mat img3 = cv::imread("image4.png", cv::IMREAD_COLOR);
	cv::Mat img4 = cv::imread("node_17.png", cv::IMREAD_COLOR);

	verific_Params = munans::PlaceRecognition::verifyGeometric(img3, img4);

	Log.info("verifyGeometric() check successful !");

	Log.print("No. of inliers = ");
	Log.println(std::to_string(verific_Params.no_of_inliers));
	Log.print("Pmatch = ");
	Log.println(std::to_string(verific_Params.P_match));
	Log.println("Fundamental Matrix = ");
	std::cout << verific_Params.FundamentalMat << std::endl;
	//				Log.println(verific_Params.FundamentalMat);
	Log.println("");

//	29. Detect() function check

	cv::Mat image_sub = munans::PlaceRecognition::captureImage(4);
//				munans::Debug::showImage("Debug",image_sub);
//				cv::waitKey(5000);
//				cv::destroyWindow( "Debug");
	int node = munans::PlaceRecognition::Detect(image_sub);

	Log.println("Current node = " + std::to_string(node));

	return 0;

}
;
