import cv2
import numpy as np
import pyrealsense2 as rs
from pypylon import pylon
from datetime import datetime
import os
import time

def connect_basler(num_cameras):
    cameras = []
    devices = pylon.TlFactory.GetInstance().EnumerateDevices()
    if len(devices) < num_cameras:
        raise ValueError(f"Not enough Basler cameras detected ({len(devices)} found, {num_cameras} required)")
    
    for i in range(num_cameras):
        cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[i]))
        cam.Open()
        cam.Width.SetValue(cam.Width.GetMax())
        cam.Height.SetValue(cam.Height.GetMax())
        cam.OffsetX.SetValue(0)
        cam.OffsetY.SetValue(0)
        cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        cameras.append(cam)
        print(f"Connected to Basler Camera {i+1}: {cam.Width.GetValue()}x{cam.Height.GetValue()} px")
    return cameras

def connect_realsense():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    print(" Connected to RealSense camera (640x480 @ 30fps)")
    return pipeline, align

def main():
    duration = 20  # seconds
    basler_cams = connect_basler(2)
    rs_pipeline, rs_align = connect_realsense()

    os.makedirs("images_basler_1", exist_ok=True)
    os.makedirs("images_basler_2", exist_ok=True)
    os.makedirs("images_realsense", exist_ok=True)

    print(f"\n Capturing as fast as possible for {duration} seconds...\n")

    start_time = time.time()
    capture_count = 0

    try:
        while (time.time() - start_time) < duration:
            loop_start = time.time()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

            # --- Capture Basler Frames ---
            basler_images = []
            for i, cam in enumerate(basler_cams):
                grab_result = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grab_result.GrabSucceeded():
                    img = grab_result.GetArray()
                    if len(img.shape) == 2:
                        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                    basler_images.append(img)
                grab_result.Release()

            # --- Capture RealSense Frame ---
            frames = rs_pipeline.wait_for_frames()
            aligned = rs_align.process(frames)
            color_frame = aligned.get_color_frame()
            rs_img = np.asanyarray(color_frame.get_data())

            # --- Save Images (faster if JPEG) ---
            if len(basler_images) > 0:
                cv2.imwrite(f"images_basler_1/{timestamp}.jpg", basler_images[0], [cv2.IMWRITE_JPEG_QUALITY, 95])
            if len(basler_images) > 1:
                cv2.imwrite(f"images_basler_2/{timestamp}.jpg", basler_images[1], [cv2.IMWRITE_JPEG_QUALITY, 95])
            cv2.imwrite(f"images_realsense/{timestamp}.jpg", rs_img, [cv2.IMWRITE_JPEG_QUALITY, 95])

            capture_count += 1
            loop_end = time.time()
            elapsed = (loop_end - loop_start) * 1000
            print(f" Capture {capture_count} done in {elapsed:.1f} ms")

    except Exception as e:
        print(f" Error: {e}")

    finally:
        for cam in basler_cams:
            cam.StopGrabbing()
            cam.Close()
        rs_pipeline.stop()
        cv2.destroyAllWindows()
        print(f"\n Capture finished. Total captures: {capture_count}")

if __name__ == "__main__":
    main()
