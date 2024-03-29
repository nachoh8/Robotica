
"""
Example adapted from:
http://www.learnopencv.com/image-alignment-feature-based-using-opencv-c-python/
"""
from __future__ import print_function

import argparse
import cv2
import numpy as np
import sys
import os
import time

PI = True
if PI:
    import picamera
    from picamera.array import PiRGBArray

# ASCI codes to interact with windows when debug > 0
ESC = 27
letter_s = 115

# max number of features to extract per image
MAX_FEATURES = 500
# REQUIRED number of correspondences (matches) found:
MIN_MATCH_COUNT=20          # initially
MIN_MATCH_OBJECTFOUND=15    # after robust check, to consider object-found

class RecLogo:
    def __init__(self, img_file, img2_file = None, debug = 0):
        self.cam = picamera.PiCamera()

        self.cam.resolution = (640, 480)
        self.cam.framerate = 10 # less frame rate, more light BUT needs to go slowly (or stop)
        self.rawCapture = PiRGBArray(self.cam)
        
        self.imReference = cv2.imread(img_file, cv2.IMREAD_COLOR)
        self.imReference2 = cv2.imread(img2_file, cv2.IMREAD_COLOR) if img2_file is not None else None
        
        self.DEBUG = debug
        
        # allow the self.camera to warmup
        time.sleep(0.2)

    def drawMatches2(self, img1, kp1, img2, kp2, matches, color=None, thickness = 2, mask=None): 
        """
        Similar to drawMatches in newer versions of open CV
        Draws lines between matching keypoints (kp1, kp2) of the two input images
        color and thickness: line plot properties
        matches: n x Match_objects
        mask: n x bool. List of booleans to indicate which matches should be displayed 
        """
        # We're drawing them side by side.  Get dimensions accordingly.
        # Handle both color and grayscale images.
        if len(img1.shape) == 3:
            new_shape = (max(img1.shape[0], img2.shape[0]), img1.shape[1]+img2.shape[1], img1.shape[2])
        elif len(img1.shape) == 2:
            new_shape = (max(img1.shape[0], img2.shape[0]), img1.shape[1]+img2.shape[1])
        new_img = np.zeros(new_shape, type(img1.flat[0]))  
        # Place images onto the new image.
        new_img[0:img1.shape[0],0:img1.shape[1]] = img1
        new_img[0:img2.shape[0],img1.shape[1]:img1.shape[1]+img2.shape[1]] = img2
        
        # Draw lines between matches.
        if color:
            c = color
        for i, m in enumerate(matches):
            if mask is None or (mask is not None and mask[i]):            
                # Generate random color for RGB/BGR and grayscale images as needed.
                if not color: 
                    c = np.random.randint(0,256,3) if len(img1.shape) == 3 else np.random.randint(0,256)
                p1 = tuple(np.round(kp1[m.queryIdx].pt).astype(int))
                p2 = tuple(np.round(kp2[m.trainIdx].pt).astype(int) + np.array([img1.shape[1], 0]))
                cv2.line(new_img, p1, p2, c, thickness)
        return new_img
 
 
    def match_images(self, img1_bgr, img2_bgr):
     
        # Feature extractor uses grayscale images
        img1 = cv2.cvtColor(img1_bgr, cv2.COLOR_BGR2GRAY)
        img2 = cv2.cvtColor(img2_bgr, cv2.COLOR_BGR2GRAY)
        
        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3: # CURRENT RASPBERRY opencv version is 2.4.9
            # Initiate ORB detector --> you could use any other detector, but this is the best performing one in this version
            binary_features = True

            detector = cv2.ORB()
        else: 
            # Initiate BRISK detector --> you could use any other detector, including NON binary features (SIFT, SURF)
            # but this is the best performing one in this version
            binary_features=True
            detector = cv2.BRISK_create()
            

        # find the keypoints and corresponding descriptors
        kp1, des1 = detector.detectAndCompute(img1,None)
        kp2, des2 = detector.detectAndCompute(img2,None)

        if des1 is None or des2 is None:
            print("WARNING: empty detection?")
            return False
        if len(des1) < MIN_MATCH_COUNT or len(des2) < MIN_MATCH_COUNT:
            print("WARNING: not enough FEATURES (im1: %d, im2: %d)" %(len(des1), len(des2)) )
            return False
        print(" FEATURES extracted (im1: %d, im2: %d)" %(len(des1), len(des2)) )
            

        if binary_features:
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des1,des2)
            matches = sorted(matches, key = lambda x:x.distance)
            good = matches
        else:
            FLANN_INDEX_KDTREE = 0
            index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
            search_params = dict(checks = 50)
            flann = cv2.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch(des1,des2,k=2)
            # store all the good matches as per Lowe's ratio test.
            good = []
            for m,n in matches:
                if m.distance < 0.7*n.distance:
                    good.append(m)

        print(" Initial matches found: %d" %(len(good)))
        if self.DEBUG > 1:
            ver = (cv2.__version__).split('.')
            if int(ver[0]) < 3: # CURRENT RASPBERRY opencv version is 2.4.9
                img_tmp = self.drawMatches2(img1,kp1,img2,kp2,good)
            else:
                img_tmp = cv2.drawMatches(img1,kp1,img2,kp2,good,None)    
            cv2.imshow("All matches", img_tmp)
            cv2.waitKey(0)

        if len(good)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            H_21, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)
            matchesMask = mask.ravel().tolist()
            num_robust_matches = np.sum(matchesMask)
            if num_robust_matches < MIN_MATCH_OBJECTFOUND:
                found = False
                print("NOT enough ROBUST matches found - %d (required %d)" % 
                    (num_robust_matches, MIN_MATCH_OBJECTFOUND))
                return found
            h,w = img1.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,H_21)
            img2_res = cv2.polylines(img2_bgr, [np.int32(dst)], True, 
                                     color=(255,255,255), thickness=3)
            found = True
            print("ROBUST matches found - %d (out of %d) --> OBJECT FOUND" % (np.sum(matchesMask), len(good)))
        else:
            print("Not enough initial matches are found - %d (required %d)" % (len(good), MIN_MATCH_COUNT))
            matchesMask = None
            found = False

        if self.DEBUG:
            if int(ver[0]) < 3: # CURRENT RASPBERRY opencv version is 2.4.9
                img3 = self.drawMatches2(img1_bgr,kp1,img2_bgr,kp2, good, color=(0, 255, 0),
                    mask = matchesMask)
            else:
                draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                               singlePointColor = None,
                               matchesMask = matchesMask, # draw only inliers
                               flags = 2)
                img3 = cv2.drawMatches(img1_bgr,kp1,img2_bgr,kp2,good,None,**draw_params)
            cv2.imshow("INLIERS", img3)
            #cv2.waitKey(0) # WAIT is run outside

        return found
     
       
    def find_template(self, mirror=False, img=None, refFilename = "R2-D2s.png"):
     
        print("Looking for reference image : ", refFilename)
        imReference = cv2.imread(refFilename, cv2.IMREAD_COLOR)

        if PI and img is None:
            print("**** processing PI-CAM image file ****")
            self.cam = piself.camera.PiCamera()

            #self.cam.resolution = (320, 240)
            self.cam.resolution = (640, 480)
            self.cam.framerate = 10 # less frame rate, more light BUT needs to go slowly (or stop)
            self.rawCapture = PiRGBArray(self.cam)
            
            # allow the self.camera to warmup
            time.sleep(0.2)

            while True:
                t1 = time.time()
                rectFound = False
                self.cam.capture(self.rawCapture, format="bgr")
                frame = self.rawCapture.array  
                
                frame = cv2.flip(frame, -1) # to rotate 180
                if self.DEBUG > 2:
                    cv2.imshow("Current view", frame)
                    cv2.imshow("Current target", imReference)
                    cv2.waitKey(0)
                
                t2 = time.time()
                found = match_images(imReference, frame)
                t3 = time.time()
                print("time to match %.2f" %(t3-t2))
                
                self.rawCapture.truncate(0)
                         
                if self.DEBUG:
                    if found:
                        cv2.waitKey(0)    
                    k = cv2.waitKey(1) & 0xff
                    if k == letter_s:
                        cv2.imwrite(str(time.time())+"_image.jpg", frame)
                    if k == ESC:
                        self.cam.close()
                        break

        else:
            if img is not None:
                print("**** processing input image file ****")
                found = match_images(imReference, img)
                cv2.waitKey(0)

            else:
                print("**** processing from regular webself.cam if connected ****")
                self.cam = cv2.VideoCapture(0)
                while True:
                    ret_val, img = self.cam.read()
                    found = match_images(imReference, img)

                    if found:
                        cv2.waitKey(0)
                    k = cv2.waitKey(1) & 0xff
                    if k == letter_s: # s to save current image
                        cv2.imwrite(str(time.time())+"_image.jpg", img)
                    if k == ESC:
                        self.cam.close()
                        break # esc to quit    
        
        cv2.destroyAllWindows()
        
    def find_logo2(self):
     
        # print("Looking for reference image : ", refFilename)
        

        if PI:
            print("**** processing PI-CAM image file ****")
            
            t1 = time.time()
            rectFound = False
            self.cam.capture(self.rawCapture, format="bgr")
            frame = self.rawCapture.array  
            
            # frame = cv2.flip(frame, -1) # to rotate 180
            if self.DEBUG > 2:
                cv2.imshow("Current view", frame)
                cv2.imshow("Current target", self.imReference)
                cv2.waitKey(0)
            
            t2 = time.time()
            found = self.match_images(self.imReference, frame)
            t3 = time.time()
            print("time to match %.2f" %(t3-t2))
            
            self.rawCapture.truncate(0)
                     
            if self.DEBUG:
                if found:
                    cv2.waitKey(0)    
                k = cv2.waitKey(1) & 0xff
                if k == letter_s:
                    cv2.imwrite(str(time.time())+"_image.jpg", frame)
                if k == ESC:
                    self.cam.close()
                

        else:
            pass
        
        cv2.destroyAllWindows()
        return found
        
    def find_logos(self):
     
        # print("Looking for reference image : ", refFilename)
        
        if PI:
            print("**** processing PI-CAM image file ****")
            
            t1 = time.time()
            rectFound = False
            self.cam.capture(self.rawCapture, format="bgr")
            frame = self.rawCapture.array  
            
            # frame = cv2.flip(frame, -1) # to rotate 180
            if self.DEBUG > 2:
                cv2.imshow("Current view", frame)
                cv2.imshow("target", self.imReference)
                if self.imReference2 is not None: cv2.imshow("target2", self.imReference2)
                cv2.waitKey(0)
            
            t2 = time.time()
            found1 = self.match_images(self.imReference, frame)
            t3 = time.time()
            print("time to match1 %.2f" %(t3-t2))
            
            if self.imReference2 is not None:
                t2 = time.time()
                found2 = self.match_images(self.imReference2, frame)
                t3 = time.time()
                print("time to match2 %.2f" %(t3-t2))
            else:
                found2 = False
            
            self.rawCapture.truncate(0)
                     
            if self.DEBUG:
                if found1 or found2:
                    cv2.waitKey(0)    
                k = cv2.waitKey(1) & 0xff
                if k == letter_s:
                    cv2.imwrite(str(time.time())+"_image.jpg", frame)
                if k == ESC:
                    self.cam.close()
        else:
            pass
        
        cv2.destroyAllWindows()
        if found1 and found2:
            return 3
        elif found1:
            return 1
        elif found2:
            return 2
        else:
            return 0
    
    def find_logo(self, idx = 1):
        rec_res = 0
        t = 0
        while rec_res != 1 and t < 3:
            rec_res = self.find_logos()
            t += 1
            time.sleep(0.1)
        
        return rec_res == idx or rec_res == 3


"""
def main():
    img1 = "/home/pi/Robotica/P5/R2-D2_s.png"
    img2 = "/home/pi/Robotica/P5/BB8_s.png"
    if not os.path.isfile(img1):
        print("image " + img1 + " mal");
        return
    if not os.path.isfile(img2):
        print("image " + img2 + " mal");
        return
    rec = RecLogo(img1,debug=3)
        
    print(rec.find_logo())
if __name__ == '__main__':
    
    #Match input image or current life video feed with the selected template
    
    main()
"""
