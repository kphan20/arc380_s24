import cv2 as cv


def record(cam_id:int = 0, fps:float=20.0, frame_size:tuple=(640, 480)):

    try:
        cap = cv.VideoCapture(cam_id)
        
        # Define the codec and create VideoWriter object
        fourcc = cv.VideoWriter_fourcc(*'XVID')
        out = cv.VideoWriter('output.avi', fourcc, fps, frame_size)
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            # TODO see if flipping is necessary
            frame = cv.flip(frame, 0)
            
            # write the flipped frame
            out.write(frame)
    finally:
        # Release everything if job is finished
        cap.release()
        out.release()