import cv2 
import numpy as np 
import time

from prettytable import *
from Functions import get_BRIEF_des
from Functions import count_matches
from Functions import get_putative

# PARAMETERS
#-----------

no_of_nodes = 30 
hes_thresh = 12000
no_of_putatives = 5
max_features = 100
Directory = 'Panoramas/'
sub_image = '1-sub_images/node_4.jpg'

voting_array = np.zeros(no_of_nodes,dtype=int)

img_sub = cv2.imread(sub_image)

kp_sub,des_sub = get_BRIEF_des(img_sub,max_features)
print len(kp_sub)

for i in range(no_of_nodes):

	trial_image = Directory + 'node_' + str(i+1) + '.png'

	img_trial = cv2.imread(trial_image)

	kp_trial,des_trial = get_BRIEF_des(img_trial,max_features)

	no_of_matches = count_matches(des_sub,des_trial)
	print no_of_matches

	voting_array[i] = no_of_matches

	print('node ' + str(i+1) + ' checked!\n')

putative_nodes = get_putative(voting_array,no_of_putatives)

print voting_array
print putative_nodes
print np.argmax(voting_array)+1
print max(putative_nodes)




