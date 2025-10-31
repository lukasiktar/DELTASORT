import cv2
import numpy as np
import pyrealsense2 as rs
from pypylon import pylon
import time
import os

# Path to SSD 
SSD_PATH = "/media/deltasort1/ADATA SD620/camera_data_videos"
os.makedirs(SSD_PATH, exist_ok=True)

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

# Main video recording loop 
def main():
    duration = 20  # seconds
    frame_rate = 30
    width, height = 640, 480

    basler_cams = connect_basler(2)
    rs_pipeline, rs_align = connect_realsense()

    # Prepare VideoWriter objects on SSD 
    out_basler_0 = cv2.VideoWriter(os.path.join(SSD_PATH, 'basler0.avi'),cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (width, height))
    out_basler_1 = cv2.VideoWriter(os.path.join(SSD_PATH, 'basler1.avi'),cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (width, height))
    out_rs = cv2.VideoWriter(os.path.join(SSD_PATH, 'realsense.avi'),cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (width, height))

    frames_to_record = int(frame_rate * duration)
    print(f"\nRecording {frames_to_record} frames (~{duration}s) to SSD at: {SSD_PATH}\n")

    for frame_idx in range(frames_to_record):
        loop_start = time.time()

        # Grab Basler frames 
        basler_frames = []
        for i, cam in enumerate(basler_cams):
            grab_result = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grab_result.GrabSucceeded():
                img = grab_result.GetArray()
                if len(img.shape) == 2:
                    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                img = cv2.resize(img, (width, height))
                basler_frames.append(img)
            else:
                pass
            grab_result.Release()

        # Grab RealSense frame 
        frames = rs_pipeline.wait_for_frames()
        aligned = rs_align.process(frames)
        color_frame = aligned.get_color_frame()
        rs_img = np.asanyarray(color_frame.get_data())
        rs_img = cv2.resize(rs_img, (width, height))

        # Write frames to video files 
        if len(basler_frames) > 0:
            out_basler_0.write(basler_frames[0])
        if len(basler_frames) > 1:
            out_basler_1.write(basler_frames[1])
        out_rs.write(rs_img)

        loop_end = time.time()
        elapsed = (loop_end - loop_start) * 1000
        print(f"Frame {frame_idx+1} written in {elapsed:.1f} ms\n")

    #  Release everything 
    for cam in basler_cams:
        cam.StopGrabbing()
        cam.Close()
    rs_pipeline.stop()
    out_basler_0.release()
    out_basler_1.release()
    out_rs.release()
    cv2.destroyAllWindows()

    print(f"\nVideo recording finished. Files saved to: {SSD_PATH}")

if __name__ == "__main__":
    main()
