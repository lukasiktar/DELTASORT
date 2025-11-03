import cv2
import numpy as np
import pyrealsense2 as rs
from pypylon import pylon
from datetime import datetime
import os
import time

# Path to SSD 
SSD_PATH = "/media/deltasort1/ADATA SD620/camera_data"
os.makedirs(SSD_PATH, exist_ok=True)

# Test SSD write permission 
test_file = os.path.join(SSD_PATH, "test.txt")
try:
    with open(test_file, "w") as f:
        f.write("test")
    os.remove(test_file)
except Exception as e:
    print(f"Cannot write to SSD folder: {e}")
    exit(1)

# Connect Basler cameras 
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

# Connect RealSense camera 
def connect_realsense():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    print("Connected to RealSense camera (640x480 @ 30fps)")
    return pipeline, align

# Main capture loop
def main():
    duration = 20  # seconds
    basler_cams = connect_basler(2)
    rs_pipeline, rs_align = connect_realsense()

    #  Create subfolders for each camera 
    save_dirs = {
        "basler_1": os.path.join(SSD_PATH, "images_basler_1"),
        "basler_2": os.path.join(SSD_PATH, "images_basler_2"),
        "realsense": os.path.join(SSD_PATH, "images_realsense")
    }
    for d in save_dirs.values():
        os.makedirs(d, exist_ok=True)

    print(f"\n Saving images to SSD at: {SSD_PATH}")
    print(f"Duration: {duration} seconds\n")

    start_time = time.time()
    capture_count = 0

    try:
        while (time.time() - start_time) < duration:
            loop_start = time.time()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

            # Capture Basler frames 
            basler_images = []
            for i, cam in enumerate(basler_cams):
                grab_result = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grab_result.GrabSucceeded():
                    img = grab_result.GetArray()
                    if len(img.shape) == 2:
                        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                    # Optional resize to speed up writing
                    #img = cv2.resize(img, (640, 480))
                    basler_images.append(img)
                    print(f"Basler camera {i+1} grabbed frame {capture_count}, shape: {img.shape}")
                else:
                    print(f"Basler camera {i+1} failed to grab frame {capture_count}")
                grab_result.Release()

            # Capture RealSense frame ---
            frames = rs_pipeline.wait_for_frames()
            aligned = rs_align.process(frames)
            color_frame = aligned.get_color_frame()
            if not color_frame:
                print(f"RealSense failed to grab frame {capture_count}")
                continue
            rs_img = np.asanyarray(color_frame.get_data())
            rs_img = cv2.resize(rs_img, (640, 480))

            # --- Save images ---
            if len(basler_images) > 0:
                success = cv2.imwrite(f"{save_dirs['basler_1']}/{timestamp}.jpg", basler_images[0], [cv2.IMWRITE_JPEG_QUALITY, 90])
                if success: print(f"Saved Basler 1 image")
            if len(basler_images) > 1:
                success = cv2.imwrite(f"{save_dirs['basler_2']}/{timestamp}.jpg", basler_images[1], [cv2.IMWRITE_JPEG_QUALITY, 90])
                if success: print(f"Saved Basler 2 image")
            success = cv2.imwrite(f"{save_dirs['realsense']}/{timestamp}.jpg", rs_img, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if success: print(f"Saved RealSense image")

            capture_count += 1
            loop_end = time.time()
            elapsed = (loop_end - loop_start) * 1000
            print(f"Capture {capture_count} done in {elapsed:.1f} ms\n")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        for cam in basler_cams:
            cam.StopGrabbing()
            cam.Close()
        rs_pipeline.stop()
        cv2.destroyAllWindows()
        print(f"\nCapture finished. Total captures: {capture_count}")

if __name__ == "__main__":
    main()
