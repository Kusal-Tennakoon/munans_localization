import os
#import rospy
import cv2
import numpy as np 
import math
import pandas as pd # To handle .xlsx files
#import Equirec2Perspec as E2P 
from matplotlib import pyplot as plt 

#from std_msgs.msg import String
#from place_localization.msg import Node


# Function to save data to sheets of an .xlsx. Cannot save multiple sheets of data
def make_dictionary(data,file_name,sheet_name):

	data = pd.DataFrame(data)
	vocabulary = pd.ExcelWriter(file_name)
	data.to_excel(vocabulary,sheet_name = sheet_name,index = False , header = False)

	vocabulary.save()

# Fucntion to import the descriptors corresponding to a particular node from an excel file 
def get_dictionary(file_name,sheet_name):

	file = pd.ExcelFile(file_name)
	# If there is no header and index col's (If there are no labels for the col's or the rows as in a tabel)
	# set headera and index_col parameters to None
	node = file.parse(sheet_name,header = None, indel_col = None)

	vocabulary = np.array(node)

	return vocabulary

# Function to print given set of points on an image
def plot_coloured_points(image,points,hor_displacement,ver_displacement,colour):
	
	if (colour == "r"):
		col_val = (0,0,255)

	elif (colour == "g"):
		col_val = (0,255,0)

	elif (colour == "b"):
		col_val = (255,0,0)

	for i in range(np.size(points,0)):
		cv2.circle(image,(int(points[i].pt[0])+hor_displacement,int(points[i].pt[1])+ver_displacement),3,col_val,2)

	return image

# Function to pring given set of feature points on an image
def plot_coloured_features(image,points,colour):
	
	if (colour == "r"):
		col_val = (0,0,255)

	elif (colour == "g"):
		col_val = (0,255,0)

	elif (colour == "b"):
		col_val = (255,0,0)

	img_col_key = cv2.drawKeypoints(image,points,None,col_val)

	return img_col_key

# Function to extract the feature points that belong to a specific node
def get_residual(kp_query,kp_trial):

	kp_residual = [i for i in kp_query if i not in kp_trial]

	return kp_residual

# Function to obtain the SIFT desciptors of an image
def get_SIFT_des(image,no_of_features):

	gray= cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

	# retval  = cv.xfeatures2d.SIFT_create( [, nfeatures[, nOctaveLayers[, contrastThreshold[, edgeThreshold[, sigma]]]]] )

	sift = cv2.xfeatures2d.SIFT_create(no_of_features)

	# kp = sift.detect(gray,None) # To compute keypoints only
	# kp,des = sift.compute(gray,kp) # To compute the discriptors once the keypoints are known

	kp,des = sift.detectAndCompute(gray,None)

	return kp,des

# Function obtain the SURF descriptors of an image
def get_SURF_des(image,hes_thresh):

	# retval	=	cv.xfeatures2d.SURF_create(	[, hessianThreshold[, nOctaves[, nOctaveLayers[, extended[, upright]]]]]	)
	surf = cv2.xfeatures2d.SURF_create(hes_thresh)
	surf.setUpright(True) # Creates upright features by eliminating orientation
	surf.setExtended(True) # Generates 128 dimensional descriptors

	# img = cv2.imread(image)
	img_gry = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	
	kp,des = surf.detectAndCompute(img_gry,None)

	return kp, des

# Fucntion to obatin the BRIEF descriptors of an image
def get_BRIEF_des(image,no_of_features):

	# Initiate FAST detector
	# star = cv2.xfeatures2d.StarDetector_create()

	sift = cv2.xfeatures2d.SIFT_create(no_of_features)

	# Initiate BRIEF extractor
	# retval	=	cv.xfeatures2d.BriefDescriptorExtractor_create(	[, bytes[, use_orientation]])
	brief = cv2.xfeatures2d.BriefDescriptorExtractor_create()

	# Process the image
	# img = cv2.imread(image)
	img_gry = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

	# Find keypoints with STAR
	# kp = star.detect(img_gry,None)
	kp = sift.detect(img_gry,None) 

	# Compute the descriptors with BRIEF
	kp,des = brief.compute(img_gry,kp)

	return kp, des

# Function to get Brief descriptors with virtual views
def get_BRIEF_VV(image_name,tilt,longitude):

	descriptors = np.array([32*[0]])
	keypoints = []

	for i in range(len(tilt)):
		for j in range(len(longitude)):

			equ = E2P.Equirectangular(image_name)    # Load equirectangular image
			img_VV = equ.GetPerspective(90, longitude[j], tilt[i], 720, 1080) # Specify parameters(FOV, theta, phi, height, width)

			kp,des = get_BRIEF_des(img_VV)

			if (np.size(des) >1):
				descriptors = np.concatenate((descriptors,des))
				keypoints.append(kp)

	np.delete(descriptors,(0),axis=0)

	return keypoints,descriptors

# Function to compute the no. of matching descriptors
def count_matches(des1,des2):
	bf = cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)
	matches = bf.match(des1,des2)
	no_of_matches = np.size(matches,0)
	# print matches
	return no_of_matches

# Function to connect matching feature points between two given images 
def draw_matches(img1, keypoints1,des1, img2, keypoints2,des2):
    rows1, cols1 = img1.shape[:2]
    rows2, cols2 = img2.shape[:2]

    # # Create a new output image that concatenates the two images together
    # output_img = np.zeros((max([rows1,rows2]), cols1+cols2, 3), dtype='uint8')
    # output_img[:rows1, :cols1, :] = np.dstack([img1, img1, img1])
    # output_img[:rows2, cols1:cols1+cols2, :] = np.dstack([img2, img2, img2])

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1,des2)

    firsttime = True

    if firsttime==True:
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]
        nWidth = w1+w2
        nHeight = max(h1, h2)
        hdif = abs(h1-h2)/2
        firsttime=False

    result = np.zeros((nHeight, nWidth,3), np.uint8)
    print result
    result[hdif:hdif+h1, :w1] = img1
    result[:h2, w1:w1+w2] = img2
    result = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
    result = cv2.cvtColor(result,cv2.COLOR_GRAY2BGR)

    result = plot_coloured_points(result,keypoints1,0,0,"g")
    result = plot_coloured_points(result,keypoints2,w1,0,"r")
    print len(keypoints1)
    print len(keypoints2)
    print len(matches)
    for i in range(len(matches)):
    	print i
        pt_a=(int(keypoints1[matches[i].trainIdx].pt[0])+hdif, int(keypoints1[matches[i].trainIdx].pt[1]))
        pt_b=(int(keypoints2[matches[i].queryIdx].pt[0])+w1, int(keypoints2[matches[i].queryIdx].pt[1]))
        cv2.line(result, pt_a, pt_b, (255, 0, 0))

    cv2.imshow('Feature matches', result)

# Function to debug errors in feature matching
def debug_image(img1, keypoints1,des1, img2, keypoints2,des2):
    rows1, cols1 = img1.shape[:2]
    rows2, cols2 = img2.shape[:2]

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1,des2)

    firsttime = True

    if firsttime==True:
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]
        nWidth = w1+w2
        nHeight = max(h1, h2)
        hdif = abs(h1-h2)/2
        firsttime=False

    result = np.zeros((nHeight, nWidth,3), np.uint8)
    result[hdif:hdif+h1, :w1] = img1
    result[:h2, w1:w1+w2] = img2
    # result = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
    # result = cv2.cvtColor(result,cv2.COLOR_GRAY2BGR)
    cv2.drawMatches(img1,keypoints1,img2,keypoints2,matches[:],result,(0,255,0),(0,0,255))

    return result

 # Function to determine the correct node using geometric verification + RANSAC
def verify_geometric(query_image,putative_image):

	MIN_MATCH_COUNT = 10 # Set a value higher than 10

	img1 = query_image
	img2 = putative_image

	kp1,des1 = get_SIFT_des(img1,300)
	kp2,des2 = get_SIFT_des(img2,300)

	FLANN_INDEX_KDTREE = 0
	index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
	search_params = dict(checks = 50)

	flann = cv2.FlannBasedMatcher(index_params,search_params)

	matches = flann.knnMatch(des1,des2,k=2)

	good = []

	for m,n in matches:
		if m.distance < 0.7*n.distance:
			good.append(m)

	if len(good) > MIN_MATCH_COUNT:
		src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
		dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1,1,2)

		M,mask = cv2.findFundamentalMat(src_pts,dst_pts, cv2.RANSAC,3.0,0.99)
		no_inliers = np.sum(mask)

	else:

		no_inliers = 0

	P_match = 2 * no_inliers/(len(kp1)+len(kp2))

	return P_match , no_inliers

def show_img(window_name,image,size):

	img = cv2.imread(image)
	# cv2.startWindowThread()
	img_gry = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	cv2.namedWindow(window_name)
	
	cv2.imshow(window_name,cv2.resize(img_gry,size))

	return None


# Function to determine whether the obtained nearest neighbour is a good match to the query descriptor
def is_good_match(distance_closest,distance_next_closest,match_thresh):

	distance_ratio = distance_closest/distance_next_closest
	# distance_ratio = float(distance_closest)/float(distance_next_closest)

	if distance_ratio <= match_thresh:
		match = 1
	else:
		match = 0

	return match

# Function to compute the tf-idf similarity scores for the nodes
def tf_idf(no_of_nodes,scores):

	tfidf = np.zeros_like(scores)

	for i in range(np.size(scores,0)):

		idf = math.log((no_of_nodes/(1+np.sum(scores[i,:]))))
		tfidf[i,:] = scores[i,:] * idf
	similarityScores = np.sum(tfidf,axis=0)


	return similarityScores

# Function to normalize the likelihood 
def normalize_likelihood(z):

	L = np.zeros(z.shape)
	mu = np.mean(z)
	sigma = np.std(z)

	for i in range(len(z)):
	 	if z[i] > mu + sigma :
	 		L[i] = (z[i] - sigma) /mu
	 	else:
	 		L[i] = 1.000

	return L

#Function to get putative matches
def get_putative(vec,no_of_putatives):

	putative =np.argpartition(vec,-no_of_putatives)[-no_of_putatives:] + 1

	return putative

#Function to compute the accuracy of retrieval
def accuracy(TP,FP,TN,FN):

	acc = (TN+TP)/(TN+FP+FN+TP)

	return acc

#Function to compute the accuracy of retrieval
def precision(TP,FP):

	pre = (TP)/(FP+TP)
	
	return pre

#Function to compute the recall of retrieval
def recall(TP,FN):

	recall = (TP)/(FN+TP)
	
	return recall	

#Function to compute the sensitivity of retrieval
def sensitivity(TP,FP,FN):

	sens = (TP)/(FP+FN)
	
	return sens

#Function to compute the specificity of retrieval
def specificity(FP,TN):

	spec = (TN)/(TN+FP)
	
	return spec

#Function to compute the F1 score of retrieval
def F1_score(precision,recall):

	F1 = 2*precision*recall/(precision+recall)
	
	return F1	

# Function to compute the performace measures (Accuracy, precision,sensitivity,specificity)
def performance(TP,FP,TN,FN):

	acc = accuracy(TP,FP,TN,FN)
	pre = precision(TP,FP)
	rec = recall(TP,FN)
	sens = sensitivity(TP,FP,FN)
	spec = specificity(FP,TN)
	F1 = F1_score(pre,rec)

	return acc,pre,rec,sens,spec,F1

#  Fucntion to derive the performance measures from the confusion matrix
def conf2perform(confusion_matrix,query_index):

	CM = confusion_matrix
	idx = query_index

	TP = CM[idx][idx]
	FP = np.sum(CM,axis=0)[idx] - TP
	FN = np.sum(CM,axis=1)[idx] - TP
	TN = np.sum(CM) - TP - FP - FN

	acc,pre,recall,sens,spec,F1 = performance(TP,FP,TN,FN)

	return acc,pre,recall,sens,spec

# Function to determine the presence of the query node in the putative nodes
def is_present(node,putatives):
	if (node in putatives):
		presence_absence = 'Present'
	else:
		presence_absence = 'Absent'

	return presence_absence

#Publisher to publish the nodes
# def nodes(node):
# 	publisher = rospy.Publisher('nodes', Node, queue_size = 10)
# 	rospy.init_node('nodes',anonymous)
# 	rate = rospy.Rate(10)

# 	while not rospy.is_shutdown():

# 		publisher.publish(str(node))
# 		rate.sleep()