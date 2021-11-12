import cv2 
import numpy as np 
import time
import datetime
import os

from prettytable import *
from Functions import get_BRIEF_des
from Functions import count_matches
from Functions import get_putative
from Functions import conf2perform
from Functions import debug_image
from Functions import is_present
from Functions import verify_geometric
tic = time.clock()

# PARAMETERS
#-----------

no_of_nodes = 49 
no_of_query_images = 30
starting_image = 1
no_of_putatives = 5
max_features = 100
verification_thresh = 0.1000
resize_factor = 1
image_type_test = 'png'
image_type_trial = 'jpg'
hes_thresh = 12000
debug_mode = 'off'
second_stage_verification = 'on'

K = [] # Camera matrix

# File paths
# ----------

File_name = 'Brute_force_loop_all'
Directory_trial = 'ISLAB/'
Directory_query = 'Test_images_ISLAB_cropped/' # Change extension to .png
# Directory_query = 'Test_Prince_Philip_actual/' # Change extension to .jpg
Directory_test = 'simulation_log_files/'

# Creating the output table
table = PrettyTable()
table.field_names = ["Node", "Presence(%)", "Accuracy(%)", "Precision", "Recall" , "Specificity" , "F1 Score"]
table.align = "c"
table.align["Node"] = "c"

# Creating the empty confusion matrix
confusion_matrix = np.zeros((no_of_nodes+1,no_of_nodes+1))

# Creating an empty presence/absence array
presence_array = np.zeros(no_of_nodes)

# Printing
# --------

# Printing the parameters

print("\nBrute force place recognition using Brute Force matcher\n")
print("-------------------------------------------------------")

print("No. of nodes = " + str(no_of_nodes))
print("No. of query images = " + str(no_of_query_images))
print("No. of putatives = " + str(no_of_putatives))
print("Max no. of features = " + str(max_features))
print("Verification threshold = " + str(verification_thresh))
print("Debug mode = " + debug_mode)
print("Second stage verification = " + second_stage_verification)
print("Image type of test images= " + image_type_test)
print("Image type of trial images= " + image_type_trial)
print("Trial image set path - " + Directory_trial)
print("Query image set path - " + Directory_query)
print("Simulation log path - " + Directory_test)
print("\n-------------------------------------------------------\n")

# --------------------------------------------------------------------

# Creating a directory to save the log files of the current trial

currentDT = datetime.datetime.now()
date_time = currentDT.strftime("%Y-%m-%d_%H:%M:%S")
Directory_test_result = File_name +'('+ date_time +')/'
test_dir_path = Directory_test + Directory_test_result

try:  
    os.mkdir(Directory_test + Directory_test_result) #Directory for simulation log
except OSError:  
    print ("Directory creation failed! \n")
else:  
    print ("Directory created successfully! \n")

# Creating a directory to save the debug images of the current trial

if debug_mode == 'off':
	pass
else:
	try:  
	    os.mkdir(test_dir_path + 'Debug_images') #Sub directory for debug images
	except OSError:  
	    print ("Directory creation failed! \n")
	else:  
	    print ("Directory created successfully! \n")

# ---------------------------------------------------------------------


# Implementation
# --------------

for i in range(no_of_nodes):

	for j in range(starting_image, starting_image + no_of_query_images):

		print('\n\nNode ' + str(i+1) + ' -> query image ' + str(j))
		print('------------------------\n')

		voting_array = np.zeros(no_of_nodes,dtype=int)

		sub_image = Directory_query + 'node(' + str(i+1) + ')/image' + str(j) + '.' + image_type_test

		img_sub = cv2.imread(sub_image)

		if (img_sub is None):
			pass
		else:

			img_sub = cv2.resize(img_sub,(0,0),fx=resize_factor,fy=resize_factor) # To resize query images that are larger than the panorama.

			kp_sub,des_sub = get_BRIEF_des(img_sub,max_features)

			print(str(len(kp_sub)) + ' features detected!\n')
			print('Checking for matches...\n')		

			# Checking for similarities

			for k in range(no_of_nodes):

				trial_image = Directory_trial + 'node(' + str(k+1) + ').jpg'

				img_trial = cv2.imread(trial_image)

				kp_trial,des_trial = get_BRIEF_des(img_trial,max_features)

				no_of_matches = count_matches(des_sub,des_trial)

				voting_array[k] = no_of_matches

				print('node ' + str(k+1) + ' checked!...' + str(no_of_matches) + ' matches found!')

				# ---------------------------------------------------------------------------------------------
				# Debugging

				if debug_mode == 'off':
					continue
				else:
					debug_result = debug_image(img_sub,kp_sub,des_sub,img_trial,kp_trial,des_trial)
					cv2.imwrite(Directory_test + Directory_test_result +"/Debug_images"+ "/node"+ str(i+1) +"_image" + str(j) + "--> node" + str(k+1) + ".png",debug_result)			

				# ----------------------------------------------------------------------------------------------

			print('\nVoting array : ' + str(voting_array))

			if (np.count_nonzero(voting_array == voting_array[0]) != len(voting_array)): # If all the elements of the array are not identical

				putative_nodes = get_putative(voting_array,no_of_putatives)

				predicted_node = np.argmax(voting_array)

				if (i+1 in putative_nodes):
					presence_array[i] = presence_array[i] + 1
					present_absent = 'Present'
				else:
					present_absent = 'Absent'

				print('\nPutative nodes : '+ str(putative_nodes))
				print('\nBest matching node from voting array : ' + str(predicted_node + 1))
				print('\nPresence of query node in putative nodes : ' + str(present_absent))
				print("")
				# print putative_nodes[-1]	

				# Second stage verification
				# -------------------------

				if (second_stage_verification == 'off'):
					confusion_matrix[i][predicted_node] = confusion_matrix[i][predicted_node] + 1
				else:
					print("\nSecond stage verification...\n")
					verif_array =[]

					for l in putative_nodes:

						putative_image = Directory_trial + 'node(' + str(l) + ').' + image_type_trial
						img_put = cv2.imread(putative_image)
						verif_out,no_of_inliers = verify_geometric(img_sub,img_put)

						print('node ' + str(l) + ' checked!... '+ str(no_of_inliers)+' inliers detected!... Similarity score = ' + str(np.round(verif_out,4)))

						if (verif_out >= verification_thresh):

							verif_array.append([verif_out,l])
						else:
							pass
					
					verif_array = np.array(verif_array) #Converting the list into array in order to extract the first column

					if (np.size(verif_array) != 0):
						print("\nVerification completed!\n")
						node_closest = int(verif_array[np.argmax(verif_array[:,0])][1])

						print('\nClosest node : ' + str(node_closest))
						print(" ")

						confusion_matrix[i][node_closest-1] = confusion_matrix[i][node_closest-1] + 1.0 
						# (node_closest -1) beacuse node_closest begins from 1 where as list index of confusion matrix begins from 0

					else:
						print ("\nVerification failed !")
						confusion_matrix[i][-1] = confusion_matrix[i][-1] + 1.0

					print("\nVerification array\n-------------------")
					print verif_array



				# Determining orientation

				# -------------------------

			else:

				print('\nUnable to predict a matching node and putative nodes!\n')		

				confusion_matrix[i][-1] = confusion_matrix[i][-1] + 1

			# time.sleep(1)

		print('\n\n Node ' + str(i+1) + ' testing completed!\n')



# Preparing the performance criteria table
for i in range(no_of_nodes):
	
	accuracy,precison,recall,specificity,F1_score = conf2perform(confusion_matrix,i)
	F1_score = 2 * recall * precison/(recall+precison)
	presence = presence_array[i]/no_of_query_images*100

	table.add_row([i+1, np.round(presence,2), np.round(accuracy*100,1), np.round(precison,3), np.round(recall,3), np.round(specificity,3), np.round(F1_score,3)])

# Printing 
print("Results\n-------\n")
print("Performance criteria\n--------------------")
print(table)
print("\n")
np.set_printoptions(threshold=np.inf)
print("Confusion Matrix\n----------------")
print (confusion_matrix.astype(int))

toc = time.clock()

print("\n")
print("Time elapsed = " + str(round((toc - tic)/60,1)) + " min (" + str(round((toc - tic)/3600,1)) + " h)")

# The following line can be used to display the resulrts and save them to a file at the same time. Type it in the command line.
# python Brute_force_loop.py | tee "simulation_log_files/Brute_force_loop_all("`date +%F_%T`")".txt

# The following command "nohup" runs the program in the background
# nohup python Brute_force_loop.py | tee "simulation_log_files/Brute_force_loop_all("`date +%F_%T`")".txt

