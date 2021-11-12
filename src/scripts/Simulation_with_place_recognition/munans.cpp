#include <iostream>
#include <cstdlib>
#include <cmath>
#include <string>
#include <array>
#include <vector>
#include <sstream>
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
#include "munans.h"

void munans::Pose::get(double *data) {

	ID = data[0];
	x = data[1];
	y = data[2];
	z = data[3];
	thx = data[4];
	thy = data[5];
	thz = data[6];
}
;

void munans::Odometry::get(double *data) {

	IDprevious = data[0];
	IDcurrent = data[1];
	x = data[2];
	y = data[3];
	z = data[4];
	thx = data[5];
	thy = data[6];
	thz = data[7];
}
;

void munans::Visual::get(double *data) {

	ID = data[0];
	x = data[1];
	y = data[2];
	z = data[3];
	thx = data[4];
	thy = data[5];
	thz = data[6];
}
;

double munans::Evaluate::accuracy(int TP, int TN, int FP, int FN) {

	double acc = double(TN + TP) / double(TN + FP + FN + TP);

	return acc;

}
;

double munans::Evaluate::precision(int TP, int FP) {

	double pre = double(TP) / double(FP + TP);

	return pre;

}
;

double munans::Evaluate::recall(int TP, int FN) {

	double recall = double(TP) / double(FN + TP);

	return recall;

}
;

double munans::Evaluate::sensitivity(int TP, int FP, int FN) {

	double sens = double(TP) / double(FP + FN);

	return sens;

}
;

double munans::Evaluate::specificity(int FP, int TN) {

	double spec = double(TN) / double(TN + FP);

	return spec;

}
;

double munans::Evaluate::F1Score(double precision, double recall) {

	double F1 = double(2 * precision * recall) / double(precision + recall);

	return F1;

}
;

munans::performanceMetrics munans::Evaluate::confusionToPerformance(
		Qmath::Matrix<int> confusion_matrix, int query_index) {

	int TP = 0;
	int FP = 0;
	int FN = 0;
	int TN = 0;

	std::vector<int> RowSum;
	std::vector<int> ColSum;

	int CM_sum = 0;

	munans::performanceMetrics metrics;

	confusion_matrix.rowSum(RowSum); // row sum
	confusion_matrix.colSum(ColSum); // column sum
	CM_sum = confusion_matrix.sum(); // sum of the confusion matrix

	TP = confusion_matrix.Get(query_index, query_index); // true positive
	FP = ColSum.at(query_index) - TP; // false positive
	FN = RowSum.at(query_index) - TP; // false negative
	TN = CM_sum - TP - FP - FN; // true negative

	metrics.accuracy = Evaluate::accuracy(TP, TN, FP, FN);
	metrics.precision = Evaluate::precision(TP, FP);
	metrics.recall = Evaluate::recall(TP, FN);
	metrics.sensitivity = Evaluate::sensitivity(TP, FP, FN);
	metrics.specificity = Evaluate::specificity(FP, TN);
	metrics.F1_Score = Evaluate::F1Score(metrics.precision, metrics.recall);

	return metrics;
}
;

std::string munans::Evaluate::isPresent(int node, std::vector<int> putatives) {

	std::string presence = "Absent";

	for (int i = 0; i <= putatives.size() - 1; i++) {

		if (node == putatives.at(i)) {

			presence = "Present";
			break;
		};
	};

	return presence;
}
;

//void munans::VisualVocabulary::makeDictionary(double* data, std::string& file_name, std::string& sheet_name) {
//
//};
//
//
//
//void munans::VisualVocabulary::getDictionary(std::string& file_name, std::string& sheet_name) {
//
//};

int munans::VisualVocabulary::isGoodMatch(double distance_closest,
		double distance_next_closest, double match_thresh) {

	int match = 0;
	double distance_ratio = distance_closest / distance_next_closest;

	if (distance_ratio <= match_thresh) {

		match = 1;
	} else {

		match = 0;
	};
	return match;
}
;

void munans::VisualVocabulary::tfIdf(Qmath::Matrix<double> scores,
		std::vector<double> &similarityScores) {

	Qmath::Matrix<double> tfidf(scores.rows(), scores.cols()); // tf-idf matrix
	std::vector<double> idf; // idf value array

	scores.rowSum(idf); // row sum
	//	Calculating tf-idf of each element row-wise
	for (int i = 0; i <= tfidf.rows() - 1; i++) {
		idf.at(i) = std::log(double(tfidf.cols()) / double(1 + idf.at(i))); // Computing idf for the row
		for (int j = 0; j <= tfidf.cols() - 1; j++) {
			tfidf.Set(i, j, (scores.Get(i, j) * idf.at(i))); // Computing tf-idf
		};
	};
	tfidf.colSum(similarityScores); // column sum
}
;

void munans::VisualVocabulary::normalizeLikelihood(Qmath::Vector<double> &z) {

	double mu = z.mean(); // mean;
	double sigma = z.stdDev(); // standard deviation

	// normalizing likelihood
	for (int i = 0; i <= z.size() - 1; i++) {

		if (z.Get(i) > (mu + sigma)) {
			z.Set(i, (z.Get(i) - sigma) / mu);
		} else {
			z.Set(i, 0);
		};
	};
}
;

cv::Mat munans::Debug::plotColouredFeatures(cv::Mat image,
		std::vector<cv::KeyPoint> &points, char colour) {

	cv::Scalar col_val;
	cv::Mat img_col_key;

	// setting color
	if (colour == 'r') {
		col_val = cv::Scalar(0, 0, 255);
	} else if (colour == 'g') {
		col_val = cv::Scalar(0, 255, 0);
	} else if (colour == 'b') {
		col_val = cv::Scalar(255, 0, 0);
	} else if (colour == 'c') {
		col_val = cv::Scalar(255, 255, 0);
	} else if (colour == 'm') {
		col_val = cv::Scalar(255, 0, 255);
	} else if (colour == 'y') {
		col_val = cv::Scalar(0, 255, 255);
	};

	cv::drawKeypoints(image, points, img_col_key, col_val, 0); // draw keypoints

	return img_col_key;
}
;

cv::Mat munans::Debug::debugImage(cv::Mat image1,
		std::vector<cv::KeyPoint> keypoints1, cv::Mat des1, cv::Mat image2,
		std::vector<cv::KeyPoint> keypoints2, cv::Mat des2) {
	cv::Mat output_image;
	std::vector<cv::DMatch> matches;
	matches.clear();

	//Brute-force match descriptors
	cv::BFMatcher desc_matcher(cv::NORM_L2, true);
	desc_matcher.match(des1, des2, matches, cv::Mat());

	// draw matches
	// cv::drawMatches(image1, keypoints1, image2, keypoints2, matches,
	// 		output_image, cv::Scalar(0, 255, 255), cv::Scalar(0, 0, 255),
	// 		std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);

		cv::drawMatches(image1, keypoints1, image2, keypoints2, matches,
			output_image, cv::Scalar::all(-1) , cv::Scalar(0, 0, 255),
			std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);

	return output_image;

}
;

void munans::Debug::showImage(std::string window_name, cv::Mat image) {

	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE); //cv::WINDOW_NORMAL|cv::WINDOW_KEEPRATIO);
	cv::imshow(window_name, image);
}
;

cv::Mat munans::PlaceRecognition::captureImage(int image_ID) {

//	double image_ID_f = (double) image_ID;
//
	cv::String image_name = "Test_images_Prince_Philip_cropped/node(17)/image"
			+ std::to_string(image_ID) + ".png";
	cv::Mat image = cv::imread(image_name, cv::IMREAD_GRAYSCALE);

	return image;

}
;

munans::features munans::PlaceRecognition::getSIFTdes(cv::Mat image,
		int no_of_features) {

	features result;

	std::vector<cv::KeyPoint> kp;
	cv::Mat des;

//	cv::Mat image_gray;

//	cv::cvtColor(image,image_gray,cv::COLOR_RGB2GRAY,0); // converting to grayscale

	//extracting features
	cv::Ptr<cv::Feature2D> sift = cv::xfeatures2d::SIFT::create(no_of_features);

	sift->detectAndCompute(image, cv::Mat(), kp, des);

	result.kp = kp;
	result.des = des;

	return result;

}
;

munans::features munans::PlaceRecognition::getSURFdes(cv::Mat image,
		int no_of_features) {

	features result;

	std::vector<cv::KeyPoint> kp;
	cv::Mat des;

//	cv::Mat image_gray;

//	cv::cvtColor(image,image_gray,cv::COLOR_RGB2GRAY,0);// converting to grayscale

	//extracting features
	cv::Ptr<cv::Feature2D> surf = cv::xfeatures2d::SURF::create(no_of_features);

	surf->detectAndCompute(image, cv::Mat(), kp, des);

	result.kp = kp;
	result.des = des;

	return result;
}
;

munans::features munans::PlaceRecognition::getBRIEFdes(cv::Mat image,
		int no_of_features) {

	features result;

	std::vector<cv::KeyPoint> kp;
	cv::Mat des;

//	cv::Mat image_gray;

//	cv::cvtColor(image,image_gray,cv::COLOR_RGB2GRAY,0);// converting to grayscale

	// extracting features
	cv::Ptr<cv::Feature2D> sift = cv::xfeatures2d::SIFT::create(no_of_features);
//	cv::Ptr<cv::FastFeatureDetector> create( int threshold=10,bool nonmaxSuppression=true,int type=cv::FastFeatureDetector::TYPE_9_16 );
//	cv::Ptr<cv::Feature2D> brief = cv::xfeatures2d::BriefDescriptorExtractor::create(64);

	cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> brief =
			cv::xfeatures2d::BriefDescriptorExtractor::create(64);

	sift->detect(image, kp, cv::Mat());
	brief->compute(image, kp, des);

	result.kp = kp;
	result.des = des;

	return result;
}
;

munans::features munans::PlaceRecognition::getBRIEFVV(std::string image_name,
		int tilt, int longitude) {

	features result;

	std::vector<cv::KeyPoint> kp;
	cv::Mat des;

	// reading the image
	cv::Mat image = cv::imread(image_name, cv::IMREAD_GRAYSCALE); // converting to grayscale

	// extracting features

	result.kp = kp;
	result.des = des;

	return result;
}
;

int munans::PlaceRecognition::countMatches(cv::Mat des1, cv::Mat des2) {

	int no_of_matches = 0;

	std::vector<cv::DMatch> matches;

	matches.clear();

	//Brute-force match descriptors
	cv::BFMatcher desc_matcher(cv::NORM_L2, true);
	desc_matcher.match(des1, des2, matches, cv::Mat());

	no_of_matches = matches.size();

	return no_of_matches;
}
;

munans::verificationParms munans::PlaceRecognition::verifyGeometric(
		cv::Mat query_image, cv::Mat putative_image) {

	munans::verificationParms result;

	int MIN_MATCH_COUNT = 10; // Set a value higher than 10

	const double ratio_thresh = 0.7;

	std::vector<cv::Point2f> src_pts;
	std::vector<cv::Point2f> dst_pts;

	munans::features query;
	munans::features putative;

	cv::Mat match_mask;

	std::vector<std::vector<cv::DMatch> > knn_matches; // k-nearest neighbour matches
	std::vector<std::vector<cv::DMatch>> good_matches; // features that passes the ratio test

	// extracting features

	query = munans::PlaceRecognition::getSIFTdes(query_image, 300);
	putative = munans::PlaceRecognition::getSIFTdes(putative_image, 300);

	// cv::Mat aa = munans::Debug::plotColouredFeatures(query_image, query.kp,
	// 		'g');
	// cv::Mat bb = munans::Debug::plotColouredFeatures(putative_image,
	// 		putative.kp, 'g');

	// munans::Debug::showImage("query", aa);
	// munans::Debug::showImage("putative", bb);
	// cv::waitKey(5000);
	// cv::destroyWindow("query");
	// cv::destroyWindow("putative");

	// cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(
	// 		cv::DescriptorMatcher::FLANNBASED);

	// //k-NN match descriptors
	// matcher->knnMatch(query.des, putative.des, knn_matches, 2);

		cv::FlannBasedMatcher matcher;
	//k-NN match descriptors
	matcher.knnMatch(query.des, putative.des, knn_matches, 2);

// cv::Mat debug_result = munans::Debug::debugImage(query_image,
// 					query.kp, query.des, putative_image, putative.kp,
// 					putative.des);

// 			munans::Debug::showImage("Debug", debug_result);
// 			cv::waitKey(5000);
// 			cv::destroyWindow("Debug");

//	std::vector<cv::DMatch> matches;
////
//	matches.clear();
//
//    cv::BFMatcher desc_matcher(cv::NORM_L2, true);
//    desc_matcher.match(query.des, putative.des, matches,cv::Mat());

	// Filter matches using the Lowe's ratio test
	std::cout << knn_matches.size() << std::endl;
	for (int i = 0; i <= knn_matches.size()-1; i++) {
		if (knn_matches.at(i).at(0).distance
				< ratio_thresh * knn_matches.at(i).at(1).distance) {
			good_matches.push_back(knn_matches.at(i));
			src_pts.push_back(query.kp.at(knn_matches.at(i).at(0).queryIdx).pt);
			dst_pts.push_back(putative.kp.at(knn_matches.at(i).at(0).trainIdx).pt);
		};
	};

	// geometric verification (computing the no. of inliers)
	std::cout << good_matches.size() << std::endl;
	if (good_matches.size() > MIN_MATCH_COUNT) {

		result.FundamentalMat = cv::findFundamentalMat(src_pts, dst_pts,
				cv::FM_RANSAC, 3, 0.99);
		result.no_of_inliers = match_mask.total();
	}

	else {
		result.no_of_inliers = 0;
	};

	result.P_match = 2 * result.no_of_inliers
			/ (sizeof(query.kp) / 2 + sizeof(putative.kp) / 2);

	return result;
}
;

void munans::PlaceRecognition::getPutative(Qmath::Vector<int> vec,
		int no_of_putatives, std::vector<int> &putatives) {

	std::vector<Qmath::Max<int>> maximums;

	vec.max(no_of_putatives, maximums);

	for (int i = 0; i <= no_of_putatives - 1; i++) {
		putatives.at(i) = (maximums.at(i).ID + 1);
	};
}
;

munans::EssentialMatrix munans::PlaceRecognition::computeRotationTranslation(
		cv::InputArray EssentialMat) {

	munans::EssentialMatrix Essential;

	cv::decomposeEssentialMat(EssentialMat, Essential.R1, Essential.R2,
			Essential.t);

	return Essential;
}
;

/*This function won't be necessary if the images coordinated are .Then the camera matrices becomes identity. Hence E = F*/
cv::Mat munans::PlaceRecognition::getEssentialMatrix(cv::InputArray FundMat,
		cv::InputArray K1, cv::InputArray K2) {

	cv::Mat EssentialMat;
//	cv::sfm::essentialFromFundamental(FundMat,K1,K2,EssentialMat);
	return EssentialMat;
}
;

int munans::PlaceRecognition::Detect(cv::Mat img_sub) {

	//Parameters
	//-----------

	int no_of_nodes = 30;
	int no_of_putatives = 5;
	int max_features = 50;
	double verification_thresh = 0.1000;
	int hes_thresh = 12000;
	int predicted_node = 0;

	std::string image_type = "png";
	std::string debug_mode = "off";
	std::string second_stage_verification = "on";
	std::string present_absent = "Absent";

	std::vector<int> presense_array(no_of_putatives, 0);
	std::vector<int> putatives(no_of_putatives, 0);
	std::vector<Qmath::Max<int>> max_node;
	std::vector<double> verif_array;
	std::vector<Qmath::Max<double>> node_closest;

	cv::Mat K; // Camera matrix
	Qmath::Vector<int> voting_array(no_of_nodes);
	munans::verificationParms verif_params;
	Log::log Log;

	// Implementation
	// --------------
	munans::features brief_sub = munans::PlaceRecognition::getSIFTdes(img_sub,
			max_features);

	//Log.info(std::string(brief_sub.kp.size()) + " features detected!\n");
	Log.info("Checking for matches...\n");

	// Checking for similarities

	for (int k = 0; k <= no_of_nodes - 1; k++) {

		std::string trial_image = "Panoramas/node_" + std::to_string(k + 1)
				+ ".png";
		cv::Mat img_trial = cv::imread(trial_image, cv::IMREAD_GRAYSCALE);
		munans::features brief_trial = munans::PlaceRecognition::getSIFTdes(
				img_trial, max_features);
		int no_of_matches = munans::PlaceRecognition::countMatches(
				brief_sub.des, brief_trial.des);

		voting_array.Set(k, no_of_matches);

		Log.info(
				"node " + std::to_string(k + 1) + " checked!..."
						+ std::to_string(no_of_matches) + " matches found!");

		// ---------------------------------------------------------------------------------------------
		// Debugging

		if (debug_mode == "off") {
			;
		} else {
			cv::Mat debug_result = munans::Debug::debugImage(img_sub,
					brief_sub.kp, brief_sub.des, img_trial, brief_trial.kp,
					brief_trial.des);
			munans::Debug::showImage("Debug", debug_result);
			cv::waitKey(5000);
			cv::destroyWindow("Debug");

		};
	};

	// ----------------------------------------------------------------------------------------------

	Log.print("\nVoting array : ");
	voting_array.print();

	if (voting_array.sum() != (voting_array.Get(0) * voting_array.size())) { // If all the elements of the array are not identical

		munans::PlaceRecognition::getPutative(voting_array, no_of_putatives,
				putatives);
		Qmath::Vector<int> votes(voting_array);

		votes.max(1, max_node);

		predicted_node = max_node.at(0).ID;

		Log.print("\nPutative nodes : ");
		Log::printVec(putatives);
		Log.info(
				"Best matching node from voting array : "
						+ std::to_string(predicted_node + 1));
		Log.print("");

		// Second stage verification
		// -------------------------

		if (second_stage_verification == "off") {
			return (predicted_node + 1);
		} else {
			Log.info("\nSecond stage verification...\n");
			verif_array.clear();

			for (int l = 0; l <= putatives.size() - 1; l++) {

				std::string putative_image = "Panoramas/node_"
						+ std::to_string(putatives.at(l)) + ".png";
				cv::Mat img_put = cv::imread(putative_image,
						cv::IMREAD_GRAYSCALE);
				Log.println("here");
				verif_params = munans::PlaceRecognition::verifyGeometric(
						img_sub, img_put);

				Log.info(
						"node " + std::to_string(putatives.at(l))
								+ " checked!... "
								+ std::to_string(verif_params.no_of_inliers)
								+ " inliers detected!... Similarity score = "
								+ std::to_string(verif_params.P_match));
				verif_array.push_back(verif_params.P_match);
			};

			Qmath::Vector<double> verif_array2(verif_array); //Converting the list into array in order to extract the first column

			if (verif_array2.size() != 0) {
				Log.info("Verification completed!\n");
				verif_array2.max(1, node_closest);

				Log.info(
						"Closest node : "
								+ std::to_string(node_closest.at(0).ID));
				Log.println(" ");
				return (node_closest.at(0).ID + 1);
			} else {
				Log.error("Verification failed !");
				return 0;
			};

			Log.println("Verification array\n-------------------");
			verif_array2.print();
		};
	} else {

		Log.error("Unable to predict a matching node and putative nodes!\n");
	};
	return 0;
};

