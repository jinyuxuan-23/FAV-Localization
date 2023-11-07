import numpy as np
from math import *
import sys
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.interpolate import interp1d
sys.path.append("..")
from parameters.param import *

Threshold = float(0.5) - float(0.0009765625)   

def atti_interp(sorted_key, sorted_quaternion, time):
    if time >= sorted_key[-1]:
        return sorted_quaternion[-1]
    elif time <= sorted_key[0]:
        return sorted_quaternion[0]
    else:
        r = R.from_quat(sorted_quaternion)
        slerp = Slerp(sorted_key, r)
        r_interp = slerp(time)
        return list(r_interp.as_quat())

def cubic_interp(sorted_key, sorted_value, time):
    if time >= sorted_key[-1]:
        return sorted_value[-1]
    elif time <= sorted_key[0]:
        return sorted_value[0]
    else:
        sorted_key = np.array(sorted_key)
        sorted_value = np.array(sorted_value)
        f = interp1d(sorted_key, sorted_value, kind='cubic')
        return float(f(time))

def atti_calculate(atti_dict, time):
    quaternion_total =[]
    key_total = []
    altitude_total = []
    latitude_total = []
    longitude_total =[]
    
    for key, value in atti_dict.items():
        key_total.append(key)
        value = value.decode('utf-8').split("\t")
        quaternion = [float(i) for i in value[:4]]
        quaternion_total.append(quaternion)
        latitude = float(value[4])     
        latitude_total.append(latitude)
        longitude = float(value[5])    
        longitude_total.append(longitude)
        altitude = float(value[6].strip(b'\x00'.decode()))  
        altitude_total.append(altitude)
    
    zip_key_quaternion = zip(key_total, quaternion_total, altitude_total, latitude_total, longitude_total)    
    sorted_zip = sorted(zip_key_quaternion, key=lambda x:x[0])
    sorted_key, sorted_quaternion, sorted_altitude, sorted_latitude, sorted_longitude = zip(*sorted_zip)
    #  interpolation
    quaternion = atti_interp(sorted_key, sorted_quaternion, time)
    altitude = cubic_interp(sorted_key, sorted_altitude, time)
    latitude = cubic_interp(sorted_key, sorted_latitude, time)
    longitude = cubic_interp(sorted_key, sorted_longitude, time)

    test_value = quaternion[0]*quaternion[2] - quaternion[1]*quaternion[3]
    if test_value < -Threshold or test_value > Threshold:   #飞机奇异状态下的姿态角（theta=+-90°）
        if test_value > 0:
            test_value = 1
        elif test_value < 0:
            test_value = -1
        psi_radi = -2*test_value*atan2(quaternion[1], quaternion[0])
        theta_radi = test_value * (pi / 2.0)
        phi_radi = 0
    else: 
        C33 = quaternion[0]**2 - quaternion[1]**2 - quaternion[2]**2 + quaternion[3]**2
        C32 = 2*(quaternion[2]*quaternion[3] + quaternion[0]*quaternion[1])
        C13 = 2*(quaternion[1]*quaternion[3] - quaternion[0]*quaternion[2])
        C21 = 2*(quaternion[1]*quaternion[2] + quaternion[0]*quaternion[3])
        C11 = quaternion[0]**2 + quaternion[1]**2 - quaternion[2]**2 - quaternion[3]**2
        psi_radi = atan2(C21, C11)
        theta_radi = -asin(C13)
        phi_radi = atan2(C32,C33)
    
    # c_ground_x = flying_height*tan(camera_theta_radi)    
    # c_ground_y = (flying_height/cos(camera_theta_radi)) * tan(camera_phi_radi) * (-1)  
    
    return latitude, longitude, altitude, phi_radi, theta_radi, psi_radi