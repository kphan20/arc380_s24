from ARC380_Assignment5_helper import capture_img
import cv2
from cv2 import aruco
import numpy as np
import enum
import matplotlib.pyplot as plt


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
    width = 10  # inches
    height = 7.5  # inches
    ppi = 96  # pixels per inch (standard resolution for most screens - can be any arbitrary value that still preserves information)

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
        [corners[0][0], corners[1][1], corners[2][2], corners[3][3]], dtype="float32"
    )

    # Define destination points as the corners of the output image
    dst_pts = np.array(
        [[width * ppi, 0], [width * ppi, height * ppi], [0, height * ppi], [0, 0]],
        dtype="float32",
    )

    # Compute the perspective transformation matrix
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    # TODO see if BGR image is necessary
    # Apply the perspective transformation to the input image
    corrected_img = cv2.warpPerspective(img, M, (1920, 1080))

    # Crop the output image to the specified dimensions
    corrected_img = corrected_img[: int(height * ppi), : int(width * ppi)]

    # debug to see image output
    # cv2.imwrite("warped.png", corrected_img)

    return corrected_img


# Defines broad BGR color ranges for each color
bgr_color_ranges = {
    "blue": ((90, 0, 0), (255, 120, 50)),  # BGR range for blue
    "yellow": ((0, 180, 180), (80, 255, 255)),  # BGR range for yellow
    "red": ((0, 0, 90), (80, 80, 255)),  # BGR range for red
}


def match_color(bgr_color):
    # Convert bgr_color to a numpy array
    bgr_color_np = np.array(bgr_color)
    print(bgr_color_np)

    # Iterate through the predefined color ranges
    for color_name, (lower, upper) in bgr_color_ranges.items():
        lower_np = np.array(lower)
        upper_np = np.array(upper)

        # Check if the detected color is within the current range
        if np.all(lower_np <= bgr_color_np) and np.all(bgr_color_np <= upper_np):
            return color_name
    return "unknown"


def detect_color(corrected_img, contour):
    # Create mask where the white is what we want to keep
    mask = np.zeros(corrected_img.shape[:2], dtype="uint8")
    cv2.drawContours(mask, [contour], -1, 255, -1)

    # Compute the mean color of pixels within the area of the mask
    mean_val = cv2.mean(corrected_img, mask=mask)

    # Returns the RGB values
    return mean_val[:3]


class Shape(enum.Enum):
    CIRCLE = 0
    SQUARE = 1


def detect_shape(contour, circle_threshold=0.8) -> Shape:
    p = cv2.arcLength(contour, closed=True)
    a = cv2.contourArea(contour)
    return Shape.CIRCLE if 4 * np.pi * a / p**2 > circle_threshold else Shape.SQUARE


def get_world_pos(contour, ppi=96):
    moments = cv2.moments(contour)
    u_c = int(moments["m10"] / moments["m00"])
    v_c = int(moments["m01"] / moments["m00"])

    # Convert pixels to inches
    x_in = u_c / ppi
    y_in = v_c / ppi

    # Inches to mm
    x_mm = x_in * 25.4
    y_mm = y_in * 25.4

    return x_mm, y_mm


def process_image():
    """
    Function that processes image, annotates it with object features, and returns the features
    """

    features = {}

    # get image and convert to cv2 image
    img = cv2.cvtColor(
        cv2.imread("tiles.png"), cv2.COLOR_BGR2RGB
    )  # cv2.cvtColor(capture_img(), cv2.COLOR_BGR2RGB)

    # detect aruco markers within image
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    aruco_params = aruco.DetectorParameters()
    aruco_detector = aruco.ArucoDetector(aruco_dict, aruco_params)
    corners, ids, _ = aruco_detector.detectMarkers(img)

    # markers_img = img.copy()
    # aruco.drawDetectedMarkers(markers_img, corners, ids)

    # plt.figure(figsize=(16,9))
    # plt.imshow(markers_img)
    # plt.title('Detected ArUco markers')
    # plt.show()

    # debug function to visualize Aruco markers
    # save_aruco_image(img, corners, ids)

    # get corrected image
    img = handle_transform(img, corners, ids)

    # Run k-means clustering on the image

    # Reshape our image data to a flattened list of RGB values
    img_data = img.reshape((-1, 3))

    img_data = np.float32(img_data)

    # Define the number of clusters
    k = 6

    # Define the criteria for the k-means algorithm
    # This is a tuple with three elements: (type of termination criteria, maximum number of iterations, epsilon/required accuracy)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

    # Run the k-means algorithm
    # Parameters: data, number of clusters, best labels, criteria, number of attempts, initial centers
    _, labels, centers = cv2.kmeans(
        img_data, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS
    )

    centers = np.uint8(centers)

    # Rebuild the image using the labels and centers
    kmeans_data = centers[labels.flatten()]
    kmeans_img = kmeans_data.reshape(img.shape)
    labels = labels.reshape(img.shape[:2])

    filtered = []
    for label in range(k):
        mask_img = np.zeros(kmeans_img.shape[:2], dtype="uint8")
        mask_img[labels == label] = 255

        contours, _ = cv2.findContours(
            mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        areas = [cv2.contourArea(contour) for contour in contours]
        contours = [
            contours[i]
            for i in range(len(contours))
            if areas[i] > 4000 and areas[i] < 20000
        ]
        areas = [cv2.contourArea(contour) for contour in contours]
        print(f"Area of each region: {areas}")

        if len(contours) < 3 or len(contours) > 4:
            contours = []
        else:
            filtered.append(contours)

        contour_img = img.copy()
        cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 3)

        plt.imshow(contour_img)
        plt.title(f"Contour image for cluster {label}")
        plt.gca().invert_yaxis()
        plt.show()
        # plt.imshow(mask_img, cmap="gray")
        # plt.title(f"Mask image for cluster {label} corresponding to dark green")
        # plt.gca().invert_yaxis()
        # plt.show()

    for contour_color in filtered:
        color = match_color(
            detect_color(img, contour_color[0])
        )  # TODO could add checks here to make sure all of the shapes fit the right color
        color_list = list()
        for idx, contour in enumerate(contours):
            shape_features = dict()
            shape_features["id"] = idx
            shape_features["shape"] = detect_shape(contour)  # TODO tune threshold
            shape_features["pos"] = get_world_pos(contour)
            shape_features["color"] = color
            color_list.append(shape_features)
        features[color] = color_list

    return features


if __name__ == "__main__":
    process_image()
