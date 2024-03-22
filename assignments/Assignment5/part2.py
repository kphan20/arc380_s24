from ARC380_Assignment5_helper import capture_img
import cv2
from cv2 import aruco
import numpy as np 
    
def save_aruco_image(img, corners, ids):
    """
    Debug function to save image with aruco markers
    """
    
    markers_img = img.copy()
    aruco.drawDetectedMarkers(markers_img, corners, ids)
    cv2.imwrite("aruco_markers.png", markers_img)

def handle_transform(img, corners, ids):
    """
    Takes image and aruco info to transform image into task frame
    """
    
    # Define the dimensions of the output image
    width = 10      # inches
    height = 7.5    # inches
    ppi = 96        # pixels per inch (standard resolution for most screens - can be any arbitrary value that still preserves information)
    
    # Sort corners based on id
    ids = ids.flatten()

    # Sort the corners based on the ids
    corners = np.array([corners[i] for i in np.argsort(ids)])

    # Remove dimensions of size 1
    corners = np.squeeze(corners)

    # Sort the ids
    ids = np.sort(ids)

    # Extract source points corresponding to the exterior bounding box corners of the 4 markers
    src_pts = np.array([corners[0][0], corners[1][1], corners[2][2], corners[3][3]], dtype='float32')

    # Define destination points as the corners of the output image
    dst_pts = np.array([[0, 0], [0, height*ppi], [width*ppi, height*ppi], [width*ppi, 0]], dtype='float32')
    
    # Compute the perspective transformation matrix
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    # TODO see if BGR image is necessary
    # Apply the perspective transformation to the input image
    corrected_img = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))

    # Crop the output image to the specified dimensions
    corrected_img = corrected_img[:int(height*ppi), :int(width*ppi)]
    
    # debug to see image output
    #cv2.imwrite("warped.png", corrected_img)
    
    return corrected_img
    
def process_image():
    """
    Function that processes image and annotates with object features
    """
    
    # get image and convert to cv2 image
    img = cv2.cvtColor(capture_img(), cv2.COLOR_BGR2RGB)
    
    # detect aruco markers within image
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    aruco_params = aruco.DetectorParameters()
    aruco_detector = aruco.ArucoDetector(aruco_dict, aruco_params)
    corners, ids, _ = aruco_detector.detectMarkers(img)
    
    # debug function to visualize Aruco markers
    #save_aruco_image(img, corners, ids)
    
    # get corrected image
    img = handle_transform(img, corners, ids)
    

if __name__ == "__main__":
    process_image()