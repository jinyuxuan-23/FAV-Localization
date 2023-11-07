import cv2
import os
from math import *
from utils.calculate_attitude import atti_calculate
from utils.get_GNSS_Position import Cal_GNSS_Position
from utils.image_process import cv_show, good_thresh_img, findColor, find_target, video_init
from parameters.param import *

def draw_center(target_list, img, atti_result):
    latitude, longitude, altitude, phi_radi, theta_radi, psi_radi = atti_result
    for  c in target_list:
        M = cv2.moments(c)                   
        center_x = int(M['m10']/M['m00'])
        center_y = int(M['m01']/M['m00'])
        input_info = [latitude, longitude, altitude, center_x, center_y, phi_radi, theta_radi, psi_radi]
        Pg, GNSS_pos = Cal_GNSS_Position(input_info)
        cv2.circle(img,(center_x,center_y),7,128,-1)
        str2 ='(' + str(GNSS_pos[0])+ ',' + str(GNSS_pos[1]) + ')'
        cv2.putText(img,str2,(center_x-50, center_y+40),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
    str_plane_GNSS_pos ='FAV latitude=' + str(latitude) + ' longitude=' + str(longitude)  + ' flying_height=' + str(altitude - ground_altitude) + 'm'
    cv2.putText(img,str_plane_GNSS_pos,(50, 90),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2,cv2.LINE_AA)
    return img

if __name__ == "__main__":
    if os.path.exists(result_save_dir) == False:
        os.mkdir(result_save_dir)
    if runmode == 0:
        #photo
        img_filenames = os.listdir(img_file_dir)
        for img_filename in img_filenames:
            target_list = []
            atti_dict = {}
            time = os.path.splitext(img_filename)[0]
            img_file_path = os.path.join(img_file_dir, img_filename)
            img =cv2.imread(img_file_path)
            imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            img_Hsv = good_thresh_img(img)
            mask_process_img = findColor(img_Hsv, myColors)
            img_with_target = find_target(mask_process_img, target_list, img)
            if len(target_list)>0:
                atti_filename = time + ".txt"
                attitude_file_path = os.path.join(attitude_file_dir, atti_filename)
                with open(attitude_file_path, encoding='utf-8') as file:
                    content=file.read()
                    exec("atti_dict = " + content, globals())
                    atti_result = atti_calculate(atti_dict, float(time))
                final_img = draw_center(target_list, img_with_target, atti_result)
                save_path = os.path.join(result_save_dir, img_filename)
                cv2.imwrite(save_path, final_img)
            else:
                final_img = img
    else:    
        #video
        cap, out, nframe = video_init(video_file_path)
        time = 0
        for i in range(nframe):
            target_list = []
            atti_dict = {}
            success , frame = cap.read()
            imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            img_Hsv = good_thresh_img(frame)
            mask_process_img = findColor(img_Hsv, myColors)
            img_with_target = find_target(mask_process_img, target_list, frame)
            if len(target_list)>0:
                atti_filename = time + ".txt"
                attitude_file_path = os.path.join(attitude_file_dir, atti_filename)
                with open(attitude_file_path, encoding='utf-8') as file:
                    content=file.read()
                    exec("atti_dict = " + content, globals())
                    atti_result = atti_calculate(atti_dict, float(time))
                final_img = draw_center(target_list, img_with_target, atti_result)
            else:
                final_img = frame
            cv2.imshow("video",final_img)
            cv2.waitKey(10)
            out.write(final_img)