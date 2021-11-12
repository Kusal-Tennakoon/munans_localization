/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
 											HEADER FILES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//C++
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <ctime>
#include <time.h>
#include <boost/filesystem.hpp>
#include "VariadicTable.h"

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

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											CONTROL VARIABLES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

// PARAMETERS
// -----------

const int no_of_nodes = 30 ;
const int no_of_query_images = 26;
const int no_of_putatives = 5;
const int starting_image = 1; 
const int max_features = 100;
const double verification_thresh = 0.1000;
const int resize_factor = 1;
std::string image_type = "png";
const int hes_thresh = 12000;
std::string debug_mode = "off";
std::string second_stage_verification = "on";

std::vector<std::vector<double>> K ; // Camera matrix
std::vector<int> putative_nodes(no_of_putatives);
std::string present_absent;
munans::verificationParms verif_params;
munans::performanceMetrics perform_metrics;
time_t t_start,t_end;

// File paths
// ----------

std::string File_name = "Place_Recognition_test";
std::string Directory_trial = "./src/munans_localization/src/Panoramas/";
std::string Directory_query = "./src/munans_localization/src/Test_images_Prince_Philip_cropped/" ; // Change extension to .png
//std::string Directory_query = "Test_Prince_Philip_actual/" // Change extension to .jpg
std::string Directory_test = "./src/munans_localization/src/simulation_log_files/";

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											DATA STRUCTURES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

Log::log PR_log; 

munans::features test_img;
munans::features trial_img;

// Creating the empty confusion matrix
Qmath::Matrix<int> confusion_matrix(no_of_nodes+1,no_of_nodes+1);

// Creating an empty presence/absence array
Qmath::Vector<int> presence_array(no_of_nodes);

VariadicTable<int, double, double, double, double, double, double> vt({"Node", "Presence", "Accuracy", "Precision","Recall","Specificity","F1 Score"},10);

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

    time(&t_start);
    // Printing
    // --------

    // Printing the parameters

    PR_log.println("\nBrute force place recognition using Brute Force matcher");
    PR_log.println("-------------------------------------------------------");

    PR_log.println("No. of nodes = " + std::to_string(no_of_nodes));
    PR_log.println("No. of query images = " + std::to_string(no_of_query_images));
    PR_log.println("No. of putatives = " + std::to_string(no_of_putatives));
    PR_log.println("Max no. of features = " + std::to_string(max_features));
    PR_log.println("Verification threshold = " + std::to_string(verification_thresh));
    PR_log.println("Debug mode = " + debug_mode);
    PR_log.println("Second stage verification = " + second_stage_verification);
    PR_log.println("Image type = " + image_type);
    PR_log.println("Trial image set path - " + Directory_trial);
    PR_log.println("Query image set path - " + Directory_query);
    PR_log.println("Simulation log path - " + Directory_test);
    PR_log.println("\n-------------------------------------------------------\n");

    // --------------------------------------------------------------------

    // Creating a directory to save the log files of the current trial

    char buffer [80];
    time_t currentDT = time(0);
    tm * ltm = localtime(&currentDT);
    size_t date_time = strftime(buffer,80,"%Y-%m-%d_%H:%M:%S",ltm);
    std::string  Directory_test_result = File_name +"("+ asctime(ltm)+")";
    std::string test_dir_path = Directory_test + Directory_test_result +"/";

    std::string dir_log = Directory_test + Directory_test_result;
    bool stat = boost::filesystem::create_directories(dir_log);
    if (stat){
        PR_log.info("Directory created successfully! \n");
    }else{
        PR_log.error("Directory creation failed! \n");
    };

    // Creating a directory to save the debug images of the current trial

    if (debug_mode == "off"){
        
    }else{
        std::string dir_debug = test_dir_path + "Debug";
        bool stat = boost::filesystem::create_directories(dir_debug);

        if (stat){
            PR_log.info("Directory created successfully! \n");
        }else{
            PR_log.error("Directory creation failed! \n");
        };
    };

    // ---------------------------------------------------------------------


    // Implementation
    // --------------

    for (int i = 0; i <= no_of_nodes-1;i++){

        for (int j = starting_image; j<= starting_image + no_of_query_images;j++){

            PR_log.println("\n\nNode " + std::to_string(i+1) + " -> query image " + std::to_string(j));
            PR_log.println("------------------------\n");

            Qmath::Vector<int> voting_array(no_of_nodes);

            std::string sub_image = Directory_query + "node(" + std::to_string(i+1) + ")/image" + std::to_string(j) + "." + image_type;

            cv::Mat img_sub = cv::imread(sub_image);

            //cv::resize(img_sub, img_sub, cv::Size(), resize_factor, resize_factor); // To resize query images that are larger than the panorama.

            test_img = munans::PlaceRecognition::getBRIEFdes(img_sub,max_features);

            PR_log.info(std::to_string(test_img.kp.size()) + " features detected!\n");
            PR_log.info("Checking for matches...\n");		

            // Checking for similarities

            for (int k = 0 ; k <= no_of_nodes-1 ; k++){

                std::string trial_image = Directory_trial + "node_" + std::to_string(k+1) + ".png";

                cv::Mat img_trial = cv::imread(trial_image);

                trial_img = munans::PlaceRecognition::getBRIEFdes(img_trial,max_features);

                int no_of_matches = munans::PlaceRecognition::countMatches(test_img.des,trial_img.des);

                voting_array.Set(k,no_of_matches);

                PR_log.info("node " + std::to_string(k+1) + " checked!..." + std::to_string(no_of_matches) + " matches found!");

                // ---------------------------------------------------------------------------------------------
                // Debugging

                if(debug_mode == "off"){
                    continue;
                }else{
                    cv::Mat debug_result = munans::Debug::debugImage(img_sub,test_img.kp,test_img.des,img_trial,trial_img.kp,trial_img.des);
                    cv::imwrite(Directory_test + Directory_test_result + "/Debug_images"+ "/node"+ std::to_string(i+1) +"_image" + std::to_string(j) + "--> node" + std::to_string(k+1) + ".png",debug_result);			
                };
            };
            
            // ----------------------------------------------------------------------------------------------
            
            PR_log.info("Voting array : ");
            voting_array.print();

            if (voting_array.sum() != (voting_array.Get(0) * (voting_array.size()-1))){ // If all the elements of the array are not identical

                munans::PlaceRecognition::getPutative(voting_array,no_of_putatives,putative_nodes);

                std::vector<Qmath::Max<int>> maximums;

                voting_array.max(1,maximums);
                
                int predicted_node = maximums.at(0).ID + 1;

                for (int t : putative_nodes){

                    if ( (i+1) == t){
                        presence_array.Set(i,presence_array.Get(i) + 1);
                        present_absent = "Present";
                        break;
                    }else{
                        present_absent = "Absent";
                    };
                };

                PR_log.info("Putative nodes : ");
                Log::printVec<int>(putative_nodes);
                PR_log.info("Best matching node from voting array : " + std::to_string(predicted_node));
                PR_log.info("Presence of query node in putative nodes : " + present_absent);
                PR_log.print("");
                // print putative_nodes[-1]	

                // Second stage verification
                // -------------------------

                if (second_stage_verification == "off"){
                    confusion_matrix.Set(i,predicted_node,confusion_matrix.Get(i,predicted_node)+ 1);
                }else{
                    PR_log.info("Second stage verification...\n");
                    std::vector<std::vector<double>> verif_array;
                    std::vector<double> v_params;
                    std::vector<double> v_put_nodes;

                    for(int l : putative_nodes){

                        std::string putative_image = Directory_trial + "node_" + std::to_string(l) + "." + image_type;
                        cv::Mat img_put = cv::imread(putative_image);
                        verif_params = munans::PlaceRecognition::verifyGeometric(img_sub,img_put);

                        PR_log.info("node " + std::to_string(l) + " checked!... " + std::to_string(verif_params.no_of_inliers) + " inliers detected!... Similarity score = " + std::to_string(verif_params.P_match));

                        if (verif_params.P_match >= verification_thresh){
                            v_params.push_back(verif_params.P_match);
                            v_put_nodes.push_back(l);
                        }else{
                            continue;
                        };
                    };
                    
                    verif_array.push_back(v_params);
                    verif_array.push_back(v_put_nodes);

                    Qmath::Vector<double> verif_vec(v_params);
                    
                    if(verif_array.size() != 0){

                        PR_log.info("Verification completed!\n");
                        std::vector<Qmath::Max<double>> maximums;
                        verif_vec.max(1,maximums);
                        int node_closest = verif_array.at(maximums.at(0).ID).at(1);

                        PR_log.info("Closest node : " + std::to_string(node_closest));
                        PR_log.print(" ");

                        confusion_matrix.Set(i,node_closest,confusion_matrix.Get (i,node_closest) + 1.0);
                        // (node_closest -1) beacuse node_closest begins from 1 where as list index of confusion matrix begins from 0

                    }else{
                        PR_log.error("Verification failed !");
                        confusion_matrix.Set(i,no_of_nodes,confusion_matrix.Get(i,no_of_nodes) + 1);
                    };

                    PR_log.info("Verification array\n-------------------");
                    Log::printMat<double>(verif_array);

                };

                // Determining orientation
                // -------------------------

            }else{

                PR_log.error("Unable to predict a matching node and putative nodes!\n");
                confusion_matrix.Set(i,no_of_nodes,confusion_matrix.Get(i,no_of_nodes) + 1);
            };
            // time.sleep(1);

        };

        PR_log.info(" Node " + std::to_string(i+1) + " testing completed!\n");
    };

    // Preparing the performance criteria table

    for (int i = 0 ; i <= no_of_nodes-1 ; i++){
        
        perform_metrics = munans::Evaluate::confusionToPerformance(confusion_matrix,i);
        double presence = presence_array.Get(i)/no_of_query_images*100;

        vt.addRow(std::tuple<int, double, double, double, double, double, double>(i+1, presence, perform_metrics.accuracy*100, perform_metrics.precision, perform_metrics.recall, perform_metrics.specificity, perform_metrics.F1_Score));
    };
    // Printing 
    PR_log.info("Results\n-------\n");
    PR_log.info("Performance criteria\n--------------------");
    vt.print(std::cout);
    PR_log.println("\n");
    PR_log.info("Confusion Matrix\n----------------");
    confusion_matrix.print();

    time(&t_end);

    PR_log.print("\n");
    PR_log.info("Time elapsed = " + std::to_string((t_end - t_start)/60) + " min (" + std::to_string((t_end - t_start)/3600) + " h)");

    return 0;
};