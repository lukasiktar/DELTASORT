import cv2
from pypylon import pylon
import numpy as np
import pyrealsense2 as rs
import time

def connect_to_basler_cameras(num_cameras):
    """Connect to multiple Basler cameras."""
    cameras = []
    devices = pylon.TlFactory.GetInstance().EnumerateDevices()

    if len(devices) < num_cameras:
        raise ValueError(f"Only {len(devices)} Basler cameras found, but {num_cameras} requested.")

    for i in range(num_cameras):
        camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[i]))
        camera.Open()
        camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        cameras.append(camera)
    return cameras

def connect_to_realsense():
    """Connect to RealSense camera."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)
    return pipeline, align

def main():
    num_basler_cameras = 2  # Example for 2 Basler cameras

    try:
        # Connect to cameras
        basler_cameras = connect_to_basler_cameras(num_basler_cameras)
        rs_pipeline, rs_align = connect_to_realsense()

        # Create OpenCV windows
        for i in range(num_basler_cameras):
            cv2.namedWindow(f'Basler Camera {i + 1}', cv2.WINDOW_NORMAL)
        cv2.namedWindow("RealSense", cv2.WINDOW_NORMAL)

        while True:
            basler_frames = []

            # Grab frames from Basler cameras first
            for i, camera in enumerate(basler_cameras):
                grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grab_result.GrabSucceeded():
                    image = grab_result.GetArray()
                    basler_frames.append(image)
                grab_result.Release()

            # Grab frame from RealSense camera
            rs_frames = rs_pipeline.wait_for_frames()
            aligned_frames = rs_align.process(rs_frames)
            color_frame = aligned_frames.get_color_frame()
            rs_image = np.asanyarray(color_frame.get_data())

            # Display Basler frames
            for i, frame in enumerate(basler_frames):
                cv2.imshow(f'Basler Camera {i + 1}', frame)

            # Display RealSense frame
            cv2.imshow("RealSense", rs_image)

            # Exit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Stop Basler cameras
        for camera in basler_cameras:
            camera.StopGrabbing()
            camera.Close()
        # Stop RealSense pipeline
        rs_pipeline.stop()
        # Close OpenCV windows
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
