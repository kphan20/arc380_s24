# from ARC380_Assignment5_helper import capture_img
import cv2
from cv2 import aruco
import numpy as np
import enum
import matplotlib.pyplot as plt
import compas.geometry as cg
import pyrealsense2 as rs

# Dimensions from inner corner to inner corner
width = 18 + 15 / 16  # 22.8125  # 10
height = 11.75  # 15.75  # 7.5
ppi = 32

def get_aligned_frames():
    """
    Code borrowed from realsense examples to align depth frame to rgb
    """
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Get frameset of color and depth
    frames = pipeline.wait_for_frames()
    # frames.get_depth_frame() is a 640x360 depth image

    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()

    # Validate that both frames are valid
    if not aligned_depth_frame or not color_frame:
        return None

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    return color_image, depth_image

def handle_transform(img, corners, ids):
    """
    Takes image and aruco info to transform image into task frame
    """
    global width, height, ppi

    # Sort corners based on id
    ids = ids.flatten()

    # Sort the corners based on the ids
    corners = np.array([corners[i] for i in np.argsort(ids)])

    # Remove dimensions of size 1
    corners = np.squeeze(corners)

    # Sort the ids
    ids = np.sort(ids)

    # Extract source points corresponding to the exterior bounding box corners of the 4 markers
    src_pts = np.array(
        [corners[0][1], corners[1][2], corners[2][3], corners[3][0]], dtype="float32"
    )

    # Define destination points as the corners of the output image
    dst_pts = np.array(
        [[0, 0], [0, height * ppi], [width * ppi, height * ppi], [width * ppi, 0]],
        dtype="float32",
    )

    # Compute the perspective transformation matrix
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    # Apply the perspective transformation to the input image
    corrected_img = cv2.warpPerspective(img, M, (1920, 1080))

    # Crop the output image to the specified dimensions
    corrected_img = corrected_img[: int(height * ppi), : int(width * ppi)]

    # debug to see image output
    # cv2.imwrite("warped.png", corrected_img)

    return corrected_img


class Shape(enum.Enum):
    CIRCLE = 0
    SQUARE = 1


def get_world_pos(contour):
    """
    Given a contour, calculate the world position based on the trained points
    """
    global ppi
    moments = cv2.moments(contour)
    u_c = int(moments["m10"] / moments["m00"])
    v_c = int(moments["m01"] / moments["m00"])

    # Convert pixels to inches
    x_in = u_c / ppi
    y_in = v_c / ppi

    # Inches to mm
    x_mm = x_in * 25.4
    y_mm = y_in * 25.4

    point1 = cg.Point(-236.99, 498.73, 21.31)  # 24.5
    point2 = cg.Point(241.78, 485.88, 21.1)  # 25.5
    point3 = cg.Point(235.89, 193.29, 21.34)  # 24.5
    point4 = cg.Point(-230.38, 166.52, 200.5)  # tr
    task_frame = cg.Frame.from_points(point2, point1, point3)

    ee_frame_t = cg.Frame(cg.Point(x_mm, y_mm), [1, 0, 0], [0, 1, 0])
    ee_frame_w = task_frame.to_world_coordinates(ee_frame_t)
    ee_frame_w.point.x += 5

    return ee_frame_w

def convert_to_task_frame(img, depth_img=None, debug=False):
    """
    Function that processes image, annotates it with object features, and returns the features
    """
    # convert to cv2 image
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # detect aruco markers within image
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    aruco_params = aruco.DetectorParameters()
    aruco_detector = aruco.ArucoDetector(aruco_dict, aruco_params)
    corners, ids, _ = aruco_detector.detectMarkers(img)

    if debug:
        markers_img = img.copy()
        aruco.drawDetectedMarkers(markers_img, corners, ids)

        plt.figure(figsize=(16, 9))
        plt.imshow(markers_img)
        plt.title("Detected ArUco markers")
        plt.show()

    # return corrected image(s)
    # assumes that depth image is aligned
    if depth_img is None:
        return handle_transform(img, corners, ids)
    else:
        return handle_transform(img, corners, ids), handle_transform(depth_img, corners, ids)

