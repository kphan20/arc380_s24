import cv2
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from perception_helpers import convert_to_task_frame, get_aligned_frames

def capture_image(save_img:bool, cam_id:int = 0, file_name=''):
    """
    Use opencv to take an image with the specified camera id, saving the image if necessary
    """

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

def generate_pcd(color_img, depth_img):
    """
    Given a frame of the camera, generate a point cloud
    """

    # TODO figure out if we should do this from opencv or realsense
    # TODO maybe use https://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.create_from_depth_image
    # TODO or use https://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.create_from_depth_image
    pcd = None
    return pcd

def get_pose(block_pcd):
    """
    Given a point cloud of a block, get the principle axes
    """
    points = np.asarray(block_pcd.points)
    mean = np.mean(points, axis=0)
    centered_points = points - mean
    cov = np.cov(centered_points.T)
    u, sigma, v_transpose = np.linalg.svd(cov)
    v = v_transpose.T
    cluster_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=mean)
    cluster_axes.rotate(v, center=mean)
    return cluster_axes

def process(debug=False, down_sample:bool=True, voxel_size=0.001, table_dist_threshold=0.5, nb_neighbors=20, std_ratio=2, eps=.005, min_points=10):
    """
    Extract the blocks and acrylic pieces from sensors
    """

    # TODO see if we can use built in pyrealsense functions to process depth images
    # img, depth_img = get_aligned_frames()
    img = capture_image(False, 0)
    depth_img = capture_image(False, 1)

    # get image warped to task from using aruco markers
    task_img, task_depth_img = convert_to_task_frame(img, depth_img, debug)

    pcd = generate_pcd(task_img, task_depth_img)

    # if down sampling id desired, use voxels to down sample
    if down_sample:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    # Get points with z-distance below table_dist_threshold
    point_arr = np.asarray(pcd.points)
    pcd = pcd.select_by_index(np.where(np.abs(point_arr[:, 2])) < table_dist_threshold)[0]

    # remove outliers
    pcd, idx = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    # Ransac plane fitting + DBSCAN
    # TODO see if you can tune distance threshold to not have to do additional processing to find acrylic pieces
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.001, ransac_n=3, num_iterations=1000)
    [a, b, c, d] = plane_model # most likely gives table plane, might not distinguish acrylic pieces
    
    # get the normal vector for all objects (assume that all have the same normal)
    norm_vector = np.array([a, b, c])
    norm_vector /= np.linalg.norm(norm_vector)

    # get non-table points
    outlier_pcd = pcd.select_by_index(inliers, invert=True) # get non table points
    
    # run DBSCAN on outliers/blocks
    block_labels = outlier_pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=debug)
    block_centers = list()
    blocks = list()
    block_poses = list()
    for label in np.unique(block_labels):
        cluster = outlier_pcd.select_by_index(np.where(block_labels == label)[0])
        centroid = np.asarray(cluster.get_center())
        block_centers.append(centroid)
        blocks.append(cluster)
        block_poses.append(get_pose(cluster))
    
    return block_centers, block_poses, blocks

    