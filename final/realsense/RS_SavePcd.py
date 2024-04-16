import time

import pyrealsense2 as rs
import numpy as np
import open3d as o3d

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

# Start the pipeline
pipeline.start(config)

# Get the device and color sensor
profile = pipeline.get_active_profile()
device = profile.get_device()
color_sensor = device.first_color_sensor()

color_sensor.set_option(rs.option.enable_auto_exposure, 1)
color_sensor.set_option(rs.option.enable_auto_white_balance, 1)

# Wait for the auto exposure and white balance to stabilize
time.sleep(2)

try:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        raise RuntimeError("Could not acquire depth or color frames.")

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Create alignment primitive with color as its target stream
    align = rs.align(rs.stream.color)
    frames = align.process(frames)

    # Update frames after alignment
    aligned_depth_frame = frames.get_depth_frame()

    # Validate that both frames are valid
    if not aligned_depth_frame:
        raise RuntimeError("Could not align depth frame to color frame.")

    # Create pointcloud object and map to color
    pc = rs.pointcloud()
    pc.map_to(color_frame)
    pointcloud = pc.calculate(aligned_depth_frame)

    # Export the point cloud to a PLY file
    pointcloud.export_to_ply("output.ply", color_frame)
    print("Point cloud saved to 'output.ply'.")

finally:
    # Stop pipeline
    pipeline.stop()
