import cv2
import numpy as np
import random

# Parameters
# ----------
img_width = 3328
img_height = 1536
no_of_crops = 50
N=6 # No. of segments
aspect_ratio = 0.5625
# snippet_width = [48,72,96,144,240,192,360,480,512,720,800,1024,1080]
snippet_width = [360,480,512,720,800,1024,1080]
i=0
j=0
k=1

Destination_dir = "Test_images_Prince_Philip_cropped/node(30)"

image = cv2.imread('Panoramas/node_30.png')

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

while (i < (no_of_crops-1)):
	for n in range(1,N):
		x = random.randint(int(img_height*0.3),int(img_height*0.5))
		y = random.randint((int(img_width*(n-1)/N)+1),int(img_width*n/N))

		p = [x,y]

		l = snippet_width[j]
		w = int(aspect_ratio * l)
		s = [l,w]

		crop = [x+l,y+w]

		if ((x+w<img_height) and (y+l<img_width)):
			cv2.imwrite(Destination_dir + '/image'+ str(k) + '.png',image[x:x+w,y:y+l])
			k = k + 1
			print ("crop " + str(i+1) + " --> " + "j = " + str(j) + " --> " +"Section " + str(n) + " --> " + str(p) + " --> " + str(s) + " --> " + str(crop))

		i = i + 1

	if (j<=(len(snippet_width)-2)):
		j = j + 1
	else:
		j = 0

