from lib.Robot import Robot
import time
import numpy as np
import math
import lib.plot as plot
from lib.utils import simubot
import picamera
from picamera.array import PiRGBArray

import cv2
import numpy as np;


def get_blob2 (frame, hsv, hsv_min, hsv_max):
    # Font to write text overlay
    font = cv2.FONT_HERSHEY_SIMPLEX
    imgBGR = frame
    # Create lists that holds the thresholds
    hsvMin = hsv_min
    hsvMax = hsv_max

    # Apply HSV thresholds 
    mask = cv2.inRange(hsv, hsvMin, hsvMax)

    # Erode and dilate
    mask = cv2.erode(mask, None, iterations=3)
    mask = cv2.dilate(mask, None, iterations=3)
    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    
    x = 0
    y = 0
    radius = 0

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 5:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(imgBGR, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(imgBGR, center, 5, (0, 0, 255), -1)
            
    output = cv2.bitwise_and(imgBGR, imgBGR, mask=mask)
    cv2.imshow("images", np.hstack([imgBGR, output]))




def get_blob (frame, hsv_min, hsv_max):
    # Font to write text overlay
    font = cv2.FONT_HERSHEY_SIMPLEX
    b = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(b, cv2.COLOR_BGR2HSV)

    # Create lists that holds the thresholds
    hsvMin = hsv_min
    hsvMax = hsv_max

    # Apply HSV thresholds 
    mask = cv2.inRange(hsv, hsvMin, hsvMax)

    # Erode and dilate
    mask = cv2.erode(mask, None, iterations=3)
    mask = cv2.dilate(mask, None, iterations=3)

    # Adjust detection parameters
    params = cv2.SimpleBlobDetector_Params()
     
    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 100;
     
    # Filter by Area.
    params.filterByArea = False
    params.minArea = 50
    params.maxArea = 50000
     
    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.01
     
    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.5
     
    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.5

    # Detect blobs
    detector = cv2.SimpleBlobDetector_create(params)

    # Invert the mask
    reversemask = 255-mask

    # Run blob detection
    keypoints = detector.detect(reversemask)

    # Get the number of blobs found
    blobCount = len(keypoints)

    # Write the number of blobs found
    text = "Count=" + str(blobCount) 
    cv2.putText(frame, text, (5,25), font, 1, (0, 255, 0), 2)
    
    kx, ky, ksize = -1, -1, -1
    for k in keypoints:
        if ksize < k.size:
            kx = k.pt[0]
            ky = k.pt[1]
            ksize = k.size
    
    text2 = "X=" + "{:.2f}".format(kx )
    cv2.putText(frame, text2, (5,50), font, 1, (0, 255, 0), 2)
    text3 = "Y=" + "{:.2f}".format(ky)
    cv2.putText(frame, text3, (5,75), font, 1, (0, 255, 0), 2)
    text4 = "S=" + "{:.2f}".format(ksize)
    cv2.putText(frame, text4, (5,100), font, 1, (0, 255, 0), 2)
    
    cv2.circle(frame, (int(kx),int(ky)), int(ksize / 2), (0, 255, 0), 2)
    
    return mask





# Just dummy function for callbacks from trackbar
def nothing(x):
    pass

# Create a trackbar window to adjust the HSV values
# They are preconfigured for a yellow object 
cv2.namedWindow("Tracking")
cv2.createTrackbar("LH", "Tracking", 0,255, nothing)
cv2.createTrackbar("LS", "Tracking", 105, 255, nothing)
cv2.createTrackbar("LV", "Tracking", 85, 255, nothing)
cv2.createTrackbar("UH", "Tracking", 255, 255, nothing)
cv2.createTrackbar("US", "Tracking", 255, 255, nothing)
cv2.createTrackbar("UV", "Tracking", 255, 255, nothing)

cam = picamera.PiCamera()

cam.resolution = (320, 240)
cam.framerate = 32
rawCapture = PiRGBArray(cam, size=(320, 240))

for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    frame = img.array

    # Convert to HSV colour space

    # Read the trackbar values
    lh = cv2.getTrackbarPos("LH", "Tracking")
    ls = cv2.getTrackbarPos("LS", "Tracking")
    lv = cv2.getTrackbarPos("LV", "Tracking")
    uh = cv2.getTrackbarPos("UH", "Tracking")
    us = cv2.getTrackbarPos("US", "Tracking")
    uv = cv2.getTrackbarPos("UV", "Tracking")

    # Create arrays to hold the minimum and maximum HSV values
    hsvMin = np.array([lh, ls, lv])
    hsvMax = np.array([uh, us, uv])
    
    mask = get_blob(frame,hsvMin, hsvMax)
    
    # Apply HSV thresholds 
    #mask = cv2.inRange(hsv, hsvMin, hsvMax)
   
    # Uncomment the lines below to see the effect of erode and dilate
    #mask = cv2.erode(mask, None, iterations=3)
    #mask = cv2.dilate(mask, None, iterations=3)

    # The output of the inRange() function is black and white
    # so we use it as a mask which we AND with the orignal image
    res = cv2.bitwise_and(frame, frame, mask=mask)

    # Show the result
    cv2.imshow("Camera", frame)
    cv2.imshow("Result view", res)

    # Wait for the escape key to be pressed
    rawCapture.truncate(0)
    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()

