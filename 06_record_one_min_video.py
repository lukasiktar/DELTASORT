import cv2
import numpy as np
import pyrealsense2 as rs
from pypylon import pylon
import time

def connect_basler(num_cameras):
    cameras = []
    devices = pylon.TlFactory.GetInstance().EnumerateDevices()
    if len(devices) < num_cameras:
        raise ValueError("Not enough Basler cameras detected")
    for i in range(num_cameras):
        cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[i]))
        cam.Open()
        cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        cameras.append(cam)
    return cameras

def connect_realsense():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    return pipeline, align

def main():
    duration = 20  # seconds
    frame_rate = 30
    width, height = 640, 480

    basler_cams = connect_basler(2)
    rs_pipeline, rs_align = connect_realsense()

    out_basler_0 = cv2.VideoWriter('basler0.avi', cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (width, height))
    out_basler_1 = cv2.VideoWriter('basler1.avi', cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (width, height))
    out_rs = cv2.VideoWriter('realsense.avi', cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (width, height))

    frames_to_record = int(frame_rate * duration)
    for _ in range(frames_to_record):
        # Grab Basler frames
        basler_frames = []
        for cam in basler_cams:
            grab_result = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grab_result.GrabSucceeded():
                img = grab_result.GetArray()
                if len(img.shape) == 2:
                    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                img = cv2.resize(img, (width, height))
                basler_frames.append(img)
            grab_result.Release()

        # Grab RealSense frame
        frames = rs_pipeline.wait_for_frames()
        aligned = rs_align.process(frames)
        color_frame = aligned.get_color_frame()
        rs_img = np.asanyarray(color_frame.get_data())
        rs_img = cv2.resize(rs_img, (width, height))

        # Write all frames synchronously
        out_basler_0.write(basler_frames[0])
        out_basler_1.write(basler_frames[1])
        out_rs.write(rs_img)

    # Release everything
    for cam in basler_cams:
        cam.StopGrabbing()
        cam.Close()
    rs_pipeline.stop()
    out_basler_0.release()
    out_basler_1.release()
    out_rs.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


