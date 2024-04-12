# from ARC380_Assignment5_helper import capture_img
import cv2
from cv2 import aruco
import numpy as np
import enum
import matplotlib.pyplot as plt
import compas.geometry as cg

# Dimensions from inner corner to inner corner
width = 18 + 15 / 16  # 22.8125  # 10
height = 11.75  # 15.75  # 7.5
ppi = 32


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


def detect_shape(contour, circle_threshold=0.8) -> Shape:
    smoothed = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
    p = cv2.arcLength(smoothed, closed=True)
    a = cv2.contourArea(smoothed)
    return (
        ("Circle", Shape.CIRCLE)
        if 4 * np.pi * a / p**2 > circle_threshold
        else ("Square", Shape.SQUARE)
    )


def get_world_pos(contour):
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


def get_center(contour):
    moments = cv2.moments(contour)
    u_c = int(moments["m10"] / moments["m00"])
    v_c = int(moments["m01"] / moments["m00"])
    return (u_c, v_c)


def get_orientation(contour, shape):
    # flip contour about y axis
    # contour = cv2.flip(contour, 1)
    if shape[1] == Shape.CIRCLE:
        return ""
    else:
        return round(90 - cv2.minAreaRect(contour)[-1], 2)


def capture_and_save_image(file_name):
    # Initialize the camera
    cap = cv2.VideoCapture(0)  # 0 is the default camera

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

    # Save the captured frame as a PNG image
    cv2.imwrite(file_name, frame)

    # Release the camera
    cap.release()

    print("Image captured and saved as", file_name)


def process_image(capture_image=True, debug=False):
    """
    Function that processes image, annotates it with object features, and returns the features
    """

    features = []

    if capture_image:
        # snap image from camera and save
        capture_and_save_image("raw_img.png")

    # get image and convert to cv2 image
    img = cv2.cvtColor(cv2.imread("raw_img.png"), cv2.COLOR_BGR2RGB)

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

    # get corrected image
    img = handle_transform(img, corners, ids)

    # Run k-means clustering on the image

    # Reshape our image data to a flattened list of RGB values
    img_data = img.reshape((-1, 3))

    img_data = np.float32(img_data)

    # Define the number of clusters
    k = 5

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
            if areas[i] > 2000 and areas[i] < 20000
        ]
        areas = [cv2.contourArea(contour) for contour in contours]
        print(f"Area of each region: {areas}")

        if debug:
            contour_img = img.copy()
            if len(contours) != 0:
                print(f"Number of filtered regions: {len(contours)}")
                # print color of region
                mean_val = cv2.mean(kmeans_img, mask=mask_img)
                print(f"Mean color of region: {mean_val[:3]}")
                cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 3)

                plt.imshow(contour_img)
                plt.title(f"Contour image for cluster {label}")
                plt.gca().invert_yaxis()
                plt.show()

        if len(contours) < 3 or len(contours) > 4:
            contours = []
        else:
            filtered.append(contours)
            # find average color of the region
            # print(f"Mean color of region: {mean_val[:3]}")

    all_contours_img = img.copy()

    for i in range(len(filtered)):
        mask = np.zeros(kmeans_img.shape[:2], dtype="uint8")
        cv2.drawContours(mask, filtered[i], -1, 255, -1)
        mean_val = cv2.mean(kmeans_img, mask=mask)
        # print(f"Mean color of region: {mean_val[:3]}")
        if mean_val[0] > 110:
            color = "yellow"
        elif mean_val[0] > 50 and mean_val[1] < 50:
            color = "red"
        else:
            color = "blue"
        for j in range(len(filtered[i])):
            cv2.drawContours(all_contours_img, filtered[i], j, (0, 255, 0), 3)
            cv2.circle(
                all_contours_img, get_center(filtered[i][j]), 5, (255, 255, 0), -1
            )
            features.append(
                {
                    "shape": detect_shape(filtered[i][j])[0],
                    "color": color,
                    "size": cv2.contourArea(filtered[i][j]),
                    "pos": get_world_pos(filtered[i][j]),
                    "center": get_center(filtered[i][j]),
                    "orientation": get_orientation(
                        filtered[i][j], detect_shape(filtered[i][j])
                    ),
                }
            )
            # label each contour below center
            cv2.putText(
                all_contours_img,
                f"Shape: {features[-1]['shape']} Size: {features[-1]['size']}",
                (
                    get_center(filtered[i][j])[0] - 60,
                    get_center(filtered[i][j])[1] + 10,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                all_contours_img,
                f"WorldPos: ({round(features[-1]['pos'].point.x,2)}, {round(features[-1]['pos'].point.y,2)})",
                (
                    get_center(filtered[i][j])[0] - 60,
                    get_center(filtered[i][j])[1] + 20,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                all_contours_img,
                f"Center: {features[-1]['center']}",
                (
                    get_center(filtered[i][j])[0] - 60,
                    get_center(filtered[i][j])[1] + 30,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                all_contours_img,
                f"Color: {features[-1]['color']} Angle: {features[-1]['orientation']}",
                (
                    get_center(filtered[i][j])[0] - 60,
                    get_center(filtered[i][j])[1] + 40,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    if debug:
        plt.imshow(all_contours_img)
        plt.title("All contours in image")
        plt.show()
        cv2.imwrite("annotated.png", cv2.cvtColor(all_contours_img, cv2.COLOR_RGB2BGR))

    return features


if __name__ == "__main__":
    feats = process_image(capture_image=False, debug=True)
    print(feats)
