import cv2
import numpy as np

img = cv2.imread('butterfly')
gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

# retval  = cv.xfeatures2d.SIFT_create( [, nfeatures[, nOctaveLayers[, contrastThreshold[, edgeThreshold[, sigma]]]]] )

sift = cv2.xfeatures2d.SIFT_create(100)

kp,des = sift.detectAndCompute(gray,None)

img=cv2.drawKeypoints(gray,kp,None,(0,0,255))

cv2.imwrite('sift_keypoints.jpg',img)