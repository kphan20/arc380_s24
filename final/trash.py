def generate_pcd(color_img, depth_img, filename='output.ply'):
    """
    Given a frame of the camera, generate a point cloud
    """

    # TODO figure out if we should do this from opencv or realsense
    # TODO maybe use https://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.create_from_depth_image
    # TODO or use https://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.create_from_depth_image
    task_frame = get_ply()
    pcd = o3d.io.read_point_cloud(filename)
    return pcd, task_frame

def get_pose(block_pcd):
    """
    Given a point cloud of a block, get the representative rotation matrix
    """
    points = np.asarray(block_pcd.points)
    mean = np.mean(points, axis=0)
    centered_points = points - mean
    cov = np.cov(centered_points.T)
    u, sigma, v_transpose = np.linalg.svd(cov)
    v = v_transpose.T
    # cluster_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=mean)
    # cluster_axes.rotate(v, center=mean)
    return v

def process(debug=False, down_sample:bool=True, voxel_size=0.001, table_dist_threshold=1, nb_neighbors=20, std_ratio=2, eps=.005, min_points=10):
    """
    Extract the blocks and acrylic pieces from sensors
    """

    # TODO see if we can use built in pyrealsense functions to process depth images
    # img, depth_img = get_aligned_frames()
    #img = capture_image(False, 0)
    #depth_img = capture_image(False, 1)

    # get image warped to task from using aruco markers
    #task_img, task_depth_img = convert_to_task_frame(img, depth_img, debug)

    #if debug:
    #    display_image(task_img)
    #    display_image(task_depth_img) # TODO test

    pcd, task_frame = generate_pcd(None, None)#task_img, task_depth_img)

    # if down sampling id desired, use voxels to down sample
    if down_sample:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    # Get points with z-distance below table_dist_threshold
    point_arr = np.asarray(pcd.points)
    pcd = pcd.select_by_index(np.where(np.abs(point_arr[:, 2]) < table_dist_threshold)[0])

    # remove outliers
    pcd, idx = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    if debug:
        display_pcd([pcd])

    # Ransac plane fitting + DBSCAN
    # TODO see if you can tune distance threshold to not have to do additional processing to find acrylic pieces
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.0002, ransac_n=3, num_iterations=1000)
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
        orientation = get_pose(cluster)
        mat = np.eye(4)
        mat[:3,:3] = orientation
        mat[:3, 3] = centroid
        # add pose in task frame
        block_poses.append(task_to_world_frame(task_frame.to_local_coordinates(cg.Frame.from_matrix(mat.tolist()))))
    if debug:
        #display_pcd([pcd.select_by_index(inliers)])
        display_pcd([outlier_pcd])# + block_poses)
    
    # TODO distinguish between blocks and acrylic pieces
    return block_centers, block_poses, blocks


def kmeans(img, k, debug, mask=None):
    # Run k-means clustering on the image

    if mask is not None:
        print(mask.shape)
        valid_points = np.argwhere(mask==255)
        print(valid_points.shape)
        mask = mask.reshape((-1, 1)).squeeze()
        print(mask.shape)
    # Reshape our image data to a flattened list of RGB values
    # rgb_planes = cv2.split(img)

    # result_planes = []
    # result_norm_planes = []
    # for plane in rgb_planes:
    #     dilated_img = cv2.dilate(plane, np.ones((7,7), np.uint8))
    #     bg_img = cv2.medianBlur(dilated_img, 21)
    #     diff_img = 255 - cv2.absdiff(plane, bg_img)
    #     norm_img = cv2.normalize(diff_img,None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    #     result_planes.append(diff_img)
    #     result_norm_planes.append(norm_img)
        
    # result = cv2.merge(result_planes)
    #result_norm = result#cv2.merge(result_norm_planes)
    # img = cv2.convertScaleAbs(img, alpha=.5, beta=0)
    # img = cv2.equalizeHist(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
    # print(img.shape)
    #test = cv2.medianBlur(img, 5)#cv2.GaussianBlur(img, (9, 9), 0)
    img_data = img.reshape((-1, 3))
    print(img_data.shape)

    img_data = img_data[mask == 255]
    print(img_data.shape)

    img_data = np.float32(img_data)

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
    rebuild = np.zeros(img.shape)
    labeled_img = np.full(img.shape[:2], -1)
    print(labeled_img.shape)
    print(labeled_img[valid_points[:, 0], valid_points[:, 1]].shape)
    labeled_img[valid_points[:, 0], valid_points[:, 1]] = labels.flatten()
    #kmeans_data = centers[labels.flatten()]
    #kmeans_img = kmeans_data.reshape(img.shape)
    #labels = labels.reshape(img.shape[:2])
    for label in range(k):
        mask_img = np.zeros(img.shape[:2], dtype="uint8")
        mask_img[labeled_img == label] = 255

        contours, _ = cv2.findContours(
            mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        areas = [cv2.contourArea(contour) for contour in contours]
        contours = [
            contours[i]
            for i in range(len(contours))
            if areas[i] > 100 and areas[i] < 20000
        ]
        areas = [cv2.contourArea(contour) for contour in contours]
        print(f"Area of each region: {areas}")

        if debug:
            contour_img = result_norm.copy()
            if len(contours) != 0:
                print(f"Number of filtered regions: {len(contours)}")
                # print color of region
                # mean_val = cv2.mean(kmeans_img, mask=mask_img)
                # print(f"Mean color of region: {mean_val[:3]}")
                for cnt in contours:
                    rect = cv2.minAreaRect(cnt)
                    w, h = rect[1]
                    print(w / h)
                    if .9 < w / h < 1.1:
                        continue
                    box = cv2.boxPoints(rect) # cv2.cv.BoxPoints(rect) for OpenCV <3.x
                    box = np.int0(box)
                    cv2.drawContours(contour_img,[box],0,(0,0,255),2)
                    cv2.drawContours(contour_img, [cnt], -1, (0, 255, 0), 3)

                plt.imshow(contour_img)
                plt.title(f"Contour image for cluster {label}")
                plt.gca().invert_yaxis()
                plt.show()

        if len(contours) < 3:
            contours = []
        elif len(contours) > 4:
            print("blocks?")
            contours = []


def process2d(debug=False, take_image=True, cam_id = 1):

    # do stuff
    features = []

    if take_image:
        # snap image from camera and save
        capture_image(True, cam_id, "raw_img.png")

    # get image and convert to cv2 image
    img = cv2.imread("raw_img.png")

    # get corrected image
    img = convert_to_task_frame(img, debug=debug)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret,thresh = cv2.threshold(gray,100,255,0)
    blocks,hierarchy = cv2.findContours(thresh, 1, 2)
    # print("Number of contours detected:", len(contours))
    if debug:
        bruh = cv2.drawContours(img.copy(), [block for block in blocks if cv2.contourArea(block) > 1000], -1, (0, 255, 0), 3)
        plt.imshow(bruh)
        plt.show()
        # canny = cv2.Canny(gray, 0, 255, apertureSize=3)
        # plt.imshow(canny)
        # plt.show()
        # cnts = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        # img_copy = img.copy()
        # for c in cnts:

        #     cv2.drawContours(img_copy,[c], 0, (0,255,0), 3)
        # plt.imshow(img_copy)
        # plt.show()
        # plt.imshow(cv2.GaussianBlur(img, (5,5), 0))
        # plt.show()
        # kernel = np.array([[0,-1, 0],[-1, 5, -1],[0, -1, 0]])
        # sharpened = cv2.filter2D(gray, -1, kernel)
        # sharpened = cv2.filter2D(sharpened, -1, kernel)
        # canny = cv2.Canny(sharpened, 100, 255, 1, apertureSize=3)
        # cnts = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        # img_copy = img.copy()
        # for c in cnts:
        #     if cv2.contourArea(c) > 100:
        #         cv2.drawContours(img_copy,[c], 0, (0,255,0), 3)
        # plt.imshow(img_copy)
        # plt.show()
    

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

    def filter_kmeans(contour, is_block):
        area = cv2.contourArea(contour)
        if area < 900 or area > 10000:
            return False
        rect = cv2.minAreaRect(contour)
        h, w = rect[1]
        print(h/w)
        is_square = 0.9 < h/w < 1.1
        return not is_square if is_block else is_square

    filtered = []
    all_acrylic_masked = np.full(kmeans_img.shape[:2], 255, dtype="uint8")
    for label in range(k):
        mask_img = np.zeros(kmeans_img.shape[:2], dtype="uint8")
        mask_img[labels == label] = 255

        contours, _ = cv2.findContours(
            mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        #areas = [cv2.contourArea(contour) for contour in contours]
        contours = [contour for contour in contours if filter_kmeans(contour, False)]
        #contours = [contour for contour in contours if filter_kmeans(contour, True)]
        # contours = [
        #     contours[i]
        #     for i in range(len(contours))
        #     if areas[i] > 1000 and areas[i] < 10000
        # ]
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
            contour_img = img.copy()
            # print(len(block_contours))
            # if len(block_contours) != 0:
            #     print(f"Number of filtered regions: {len(block_contours)}")
            #     # print color of region
            #     mean_val = cv2.mean(kmeans_img, mask=mask_img)
            #     print(f"Mean color of region: {mean_val[:3]}")
            #     cv2.drawContours(contour_img, block_contours, -1, (0, 255, 0), 3)

            #     plt.imshow(contour_img)
            #     plt.title(f"Contour image for cluster {label}")
            #     plt.gca().invert_yaxis()
            #     plt.show()

        if len(contours) < 3:
            contours = []
        elif len(contours) > 4:
            print("blocks?")
            contours = []
        else:
            filtered.append(contours)
            all_acrylic_masked[labels == label] = 0
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
                    "orientation": get_orientation(
                        filtered[i][j], detect_shape(filtered[i][j])
                    ),
                }
            )
    blocks = [{}]
    acrylic = [{}]
    #masked = cv2.bitwise_and(img, img, mask=all_acrylic_masked)
    # kernel = np.array([[0,-1, 0],[-1, 5, -1],[0, -1, 0]])
    # sharpened = cv2.filter2D(cv2.cvtColor(masked, cv2.COLOR_RGB2GRAY), -1, kernel)
    # sharpened = cv2.filter2D(sharpened, -1, kernel)
    # canny = cv2.Canny(sharpened, 100, 255, 1, apertureSize=3)
    # cnts = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    #kmeans(img, 2, True, all_acrylic_masked)
    return features, blocks, acrylic