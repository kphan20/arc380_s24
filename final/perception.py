import cv2
import numpy as np
import matplotlib.pyplot as plt
from perception_helpers import Color, Shape, convert_to_task_frame, detect_shape, get_aligned_frames, display_image, display_pcd, get_center, get_orientation, get_ply, get_world_pos, task_to_world_frame
import math

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

def filter_kmeans(contour, is_block):
        area = cv2.contourArea(contour)
        if area < 900 or area > 10000:
            return False
        rect = cv2.minAreaRect(contour)
        h, w = rect[1]
        is_square = 0.9 < h/w < 1.1
        return not is_square if is_block else is_square

def process2d(debug=False, take_image=True, cam_id = 1):

    if take_image:
        # snap image from camera and save
        capture_image(True, cam_id, "raw_img.png")

    # get image and convert to cv2 image
    img = cv2.imread("raw_img.png")

    # get corrected image
    img = convert_to_task_frame(img, debug=debug)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # finds blocks through thresholding
    ret,thresh = cv2.threshold(gray,100,255,0)
    blocks,hierarchy = cv2.findContours(thresh, 1, 2)
    # print("Number of contours detected:", len(contours))
    block_contours = [block for block in blocks if cv2.contourArea(block) > 1000]
    if debug:
        bruh = cv2.drawContours(img.copy(), block_contours, -1, (0, 255, 0), 3)
        plt.imshow(bruh)
        plt.show()
    
    block_info = list()
    for c in block_contours:
        block_info.append(
                {
                    "shape": "block",
                    "color": "",
                    "size": cv2.contourArea(c),
                    "pos": get_world_pos(c),
                    "orientation": get_orientation(
                        c, ("Square", Shape.SQUARE)
                    ),
                }
            )

    # Run k-means clustering on the image to find the acrylic pieces
    acrylic_squares, large_disks, small_disks = list(), list(), list()

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

        contours = [contour for contour in contours if filter_kmeans(contour, False)]
        areas = [cv2.contourArea(contour) for contour in contours]

        if debug:
            print(f"Area of each region: {areas}")
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
            contour_img = img.copy()

        if len(contours) > 0:
            filtered.append(contours)

    all_contours_img = img.copy()

    for i in range(len(filtered)):
        mask = np.zeros(kmeans_img.shape[:2], dtype="uint8")
        cv2.drawContours(mask, filtered[i], -1, 255, -1)
        mean_val = cv2.mean(kmeans_img, mask=mask)
        # print(f"Mean color of region: {mean_val[:3]}")

        if mean_val[0] > 120:
            color = Color.YELLOW
        elif mean_val[0] > 50 and mean_val[1] < 50:
            color = Color.RED
        else:
            color = Color.BLUE
        
        if debug:
            print(mean_val)
            plt.imshow(mask)
            plt.show()
        
        for j in range(len(filtered[i])):
            orientation = get_orientation(
                        filtered[i][j], detect_shape(filtered[i][j])
                    )
            info = {
                    "shape": detect_shape(filtered[i][j])[0],
                    "color": color,
                    "size": cv2.contourArea(filtered[i][j]),
                    "pos": get_world_pos(filtered[i][j], orientation* math.pi / 180),
                    "orientation": orientation,
            }
            if info["shape"] == "Square":
                acrylic_squares.append(info)
            elif info["size"] > 6000:
                large_disks.append(info)
            else:
                small_disks.append(info)

    # blocks, large disks, squares, small disk
    # TODO see if dividing by color would help
    return block_info, large_disks, acrylic_squares, small_disks


if __name__ == "__main__":
    #bruh = process(True, False)[1]
    _, a, b, c = process2d(True, False)