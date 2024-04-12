import cv2
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def capture_image(save_img:bool, cam_id:int = 0, file_name=''):
    # Initialize the camera
    cap = cv2.VideoCapture(cam_id)  # 0 is the default camera

    # Check if the camera is opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Capture a frame
    ret, frame = cap.read()

    # Check if the frame was captured successfully
    if not ret:
        print("Error: Could not capture frame.")
        return

    if save_img:
        # Save the captured frame as a PNG image
        cv2.imwrite(file_name, frame)
        print("Image captured and saved as", file_name)

    # Release the camera
    cap.release()

    return frame

def generate_pcd(frame):
    """
    Given a frame of the camera, generate a point cloud
    """

    # TODO figure out if we should do this from opencv or realsense
    pcd = None
    return pcd

def process(down_sample:bool, voxel_size=0.001, table_dist_threshold=0.5, nb_neighbors=20, std_ratio=2):
    img = capture_image(False, 0)

    pcd = generate_pcd(img)

    # if down sampling id desired, use voxels to down sample
    if down_sample:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    # Get points with z-distance below table_dist_threshold
    point_arr = np.asarray(pcd.points)
    pcd = pcd.select_by_index(np.where(np.abs(point_arr[:, 2])) < table_dist_threshold)[0]

    # remove outliers
    pcd, idx = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)