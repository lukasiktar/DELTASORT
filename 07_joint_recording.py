import cv2
import numpy as np
import pyrealsense2 as rs
from pypylon import pylon
import time


def connect_basler(num_cameras):
    """Connect to multiple Basler cameras."""
    cameras = []
    devices = pylon.TlFactory.GetInstance().EnumerateDevices()

    if len(devices) < num_cameras:
        raise ValueError(f"Only {len(devices)} Basler cameras found, but {num_cameras} requested.")

    for i in range(num_cameras):
        cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[i]))
        cam.Open()
        cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        cameras.append(cam)
    return cameras


def connect_realsense():
    """Connect and start the RealSense camera."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    return pipeline, align


def get_basler_frames(cameras, width, height):
    """Grab one frame from each Basler camera."""
    frames = []
    for cam in cameras:
        grab_result = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grab_result.GrabSucceeded():
            img = grab_result.GetArray()
            if len(img.shape) == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            img = cv2.resize(img, (width, height))
            frames.append(img)
        grab_result.Release()
    return frames


def get_realsense_frame(pipeline, align, width, height):
    """Grab one frame from the RealSense color stream."""
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()
    if not color_frame:
        return None
    color_image = np.asanyarray(color_frame.get_data())
    color_image = cv2.resize(color_image, (width, height))
    return color_image


def main():
    num_basler_cameras = 2
    duration = 20          # seconds
    frame_rate = 30
    width, height = 640, 480

    print("Connecting to cameras...")
    basler_cams = connect_basler(num_basler_cameras)
    rs_pipeline, rs_align = connect_realsense()
    print("All cameras connected!")

    # Output video size: horizontally combine all 3 frames
    combined_width = width * (num_basler_cameras + 1)
    combined_height = height

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('combined_video.avi', fourcc, frame_rate, (combined_width, combined_height))

    frames_to_record = int(frame_rate * duration)
    print("Recording started...")

    for i in range(frames_to_record):
        # Get frames from all cameras
        basler_frames = get_basler_frames(basler_cams, width, height)
        realsense_frame = get_realsense_frame(rs_pipeline, rs_align, width, height)

        if len(basler_frames) != num_basler_cameras or realsense_frame is None:
            print("Frame skipped due to missing data.")
            continue

        # Combine horizontally: [Basler1 | Basler2 | RealSense]
        combined_frame = np.hstack(basler_frames + [realsense_frame])

        # Write to video
        out.write(combined_frame)

        # Optional live preview (press q to stop early)
        cv2.imshow("Combined View", combined_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("Recording finished.")

    # Cleanup
    for cam in basler_cams:
        cam.StopGrabbing()
        cam.Close()
    rs_pipeline.stop()
    out.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
