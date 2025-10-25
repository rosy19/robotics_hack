import cv2
import numpy as np

# From RC (example coordinates in cm)
robot_pts = np.float32([ [-0.22, 0.05], [0.22, 0.05], [-0.22, 0.25] ]) # BL, BR, TL

# From Calibration Session (converted to pixel coordinates)
frame_width, frame_height = 1920, 1080 # Example resolution
cam_norm_bl = (0.0, 1.0)
cam_norm_br = (1.0, 1.0)
cam_norm_tl = (0.0, 0.0)
camera_pts = np.float32([
    [cam_norm_bl[0] * frame_width, cam_norm_bl[1] * frame_height],
    [cam_norm_br[0] * frame_width, cam_norm_br[1] * frame_height],
    [cam_norm_tl[0] * frame_width, cam_norm_tl[1] * frame_height]
])

# THE CORE CALCULATION:
affine_matrix = cv2.getAffineTransform(camera_pts, robot_pts)
print("Calculated Affine Matrix:")
print(affine_matrix)

# Save this matrix for the SI to use!
np.save("affine_matrix.npy", affine_matrix)

def load_transform():
    return np.load("affine_matrix.npy")

def transform_cam_to_robot(cam_x_norm, cam_y_norm, matrix, frame_w, frame_h):
    cam_px = cam_x_norm * frame_w
    cam_py = cam_y_norm * frame_h
    cam_point = np.array([[[cam_px, cam_py]]], dtype=np.float32) # Needs shape (1, 1, 2) for transform

    robot_point = cv2.transform(cam_point, matrix)

    return robot_point[0][0][0], robot_point[0][0][1] # Return rx, ry