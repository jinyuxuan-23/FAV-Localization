import numpy as np

# camera info
fx = 809.0155 
fy = 807.7006 
cx = 948.0682 
cy = 523.6103 
k1 = -0.1820
k2 = 0.0343
k3 = -0.0065
p1 = 0
p2 = 0

# Intrinsic matrix of the camera
mtx = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
# distortion
dists = np.array([[k1, k2, p1, p2, k3]])

# ground altitude
ground_altitude = 9.82

# camera_install angle
camera_install_angle_psi_deg = 0
camera_install_angle_theta_deg = -106
camera_install_angle_phi_deg = 0

# color selection
myColors = [149,157,149,255,255,255]

# file parameters
img_file_dir = "fpv_localization/data/imgs"
attitude_file_dir = "/data/jyxuan/fpv_localization/data/att"
video_file_path = ""
result_save_dir = "fpv_localization/result/"

# run mode
runmode = 0    # 0 for image, 1 for video