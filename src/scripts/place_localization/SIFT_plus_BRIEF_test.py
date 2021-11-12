import cv2
import numpy as np

img = cv2.imread('butterfly')
gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

# retval  = cv.xfeatures2d.SIFT_create( [, nfeatures[, nOctaveLayers[, contrastThreshold[, edgeThreshold[, sigma]]]]] )

sift = cv2.xfeatures2d.SIFT_create(100)

# kp = sift.detect(gray,None) # To compute keypoints only
# kp,des = sift.compute(gray,kp) # To compute the discriptors once the keypoints are known

# kp,des = sift.detectAndCompute(gray,None)

brief = cv2.xfeatures2d.BriefDescriptorExtractor_create()

# Process the image
# img = cv2.imread(image)
# img_gry = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

# Find keypoints with STAR
# kp = star.detect(img_gry,None)
kp = sift.detect(gray,None) 

# Compute the descriptors with BRIEF
kp,des = brief.compute(gray,kp)

img=cv2.drawKeypoints(gray,kp,None,(0,0,255))

cv2.imwrite('sift_keypoints.jpg',img)