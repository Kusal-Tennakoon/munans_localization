import cv2
import numpy as np
import random
import os
import datetime

# Parameters
# ----------
img_width = 5472
img_height = 2736
no_of_images = 49
no_of_crops = 150
lower_bound = 0 # As a a percentage of the height
upper_bound = 1.0 # As a a percentage of the height
N=6 # No. of segments
aspect_ratio = 0.5625
# snippet_width = [48,72,96,144,240,192,360,480,512,720,800,1024,1080]
snippet_width = [360,480,512,720,800,1024,1080]

File_name = 'Snippets'
Directory_source = '/home/umbra/catkin_ws/src/munans_localization/src/Image_Data/ISLAB/'
Directory_result = 'Test_images_ISLAB_cropped/'

# Creating a directory to save the snippets

currentDT = datetime.datetime.now()
date_time = currentDT.strftime("%Y-%m-%d_%H:%M:%S")
Directory_snippet = File_name +'('+ date_time +')/'
result_dir_path = Directory_result + Directory_snippet

try:  
    os.mkdir(result_dir_path) #Directory for snippets of the current run
except OSError:  
    print ("Directory creation failed! \n")
else:  
    print ("Directory created successfully! \n")

for t in range(1,no_of_images+1):		   

	i=0
	j=0
	k=1

	Destination_dir = result_dir_path + "/node(" + str(t) + ")"

	try:  
		os.mkdir(Destination_dir) #Directory for Snippets of node t
	except OSError:  
		print ("Sub directory creation failed! \n")
	else:  
		print ("Sub directory " + "/node(" + str(t) + ")" + " created successfully! \n")


	image = cv2.imread(Directory_source + "node(" + str(t) + ").jpg")

	# Grapic of the parameters involved
	# ---------------------------------
	print("")

	print("       ------------ crop(x+w,y+l)")
	print("      |            |")
	print("    w |            |")
	print("      |            |")
	print("p(x,y) ------------ ")
	print("            l       ")

	print("")

	print ("image  |  j  |  section  |  p(x,y)  |  s(l,w)  |  crop (x+w,y+l)  |")

	print("")

	# Code
	# ----

	while (i < (no_of_crops)):
		for n in range(1,N):
			x = random.randint(int(img_height*lower_bound),int(img_height*upper_bound))
			y = random.randint((int(img_width*(n-1)/N)+1),int(img_width*n/N))

			p = [x,y]

			l = snippet_width[j]
			w = int(aspect_ratio * l)
			s = [l,w]

			crop = [x+l,y+w]

			if ((x+w<img_height) and (y+l<img_width)):
				cv2.imwrite(Destination_dir + '/imagee'+ str(k) + '.png',image[x:x+w,y:y+l])
				k = k + 1
				print ("crop " + str(i+1) + " --> " + "j = " + str(j) + " --> " +"Section " + str(n) + " --> " + str(p) + " --> " + str(s) + " --> " + str(crop))

			i = i + 1

		if (j<=(len(snippet_width)-2)):
			j = j + 1
		else:
			j = 0

