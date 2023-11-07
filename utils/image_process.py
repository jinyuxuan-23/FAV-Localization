import cv2
import numpy as np
import sys
from math import *
from scipy.spatial.transform import Rotation as R
sys.path.append("..")
from parameters.param import *

def video_init(video_file_path):
    cap = cv2.VideoCapture(video_file_path)
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('video_out.avi', fourcc, fps, (w, h))
    return cap, out, n_frames

def cv_show(name,img):
    cv2.imshow(name,img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Gaussian filtering
def good_thresh_img(img):
    gs_frame = cv2.GaussianBlur(img, (5, 5), 0)                     
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)                
    erode_hsv = cv2.erode(hsv, None, iterations=2)
    return erode_hsv

def findColor(img_hsv, myColors):
    lower = np.array(myColors[0:3])
    upper = np.array(myColors[3:6])
    mask = cv2.inRange(img_hsv,lower,upper)
    mask_process_img = cv2.bitwise_and(img_hsv, img_hsv, mask = mask)
    return mask_process_img

def find_target(mask_process_img, target_list, img):
    inRange_gray = cv2.cvtColor(mask_process_img,cv2.COLOR_BGR2GRAY)
    contours,hierarchy = cv2.findContours(inRange_gray,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    # Size selection
    for c in contours:
        if cv2.contourArea(c) < 10 or cv2.contourArea(c) > 900:   
            continue
        else:
            target_list.append(c)               
    for i in target_list:                   
        rect = cv2.minAreaRect(i)
        box = cv2.boxPoints(rect)
        cv2.drawContours(img, [np.int0(box)], -1, (0, 255, 255), 2)
    return img
