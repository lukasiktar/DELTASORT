import cv2
from pypylon import pylon
import numpy as np
import pyrealsense2 as rs
import time
from src.config import MOTION_THRESHOLD, MIN_MOTION_TIME, NO_MOTION_TIME
from src.motion_detection import detect_motion
from src.camera_setup import CameraSetup

class MotionCameraStream(CameraSetup):

    def __init__(self):
        super().__init__()

    def get_realsense_frame(self):
            """Return the latest RGB frame as a numpy array."""
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            if not color_frame:
                return None
            return np.asanyarray(color_frame.get_data())
    

    def run(self):
        print("Starting motion monitoring...")

        frame1 = self.get_realsense_frame()
        frame2 = self.get_realsense_frame()

        last_motion_time = 0
        first_motion_time = 0

        while True:
            if frame1 is None or frame2 is None:
                frame2 = self.get_realsense_frame()
                continue

            current_time = time.time()

            # Check for motion
            motion_detected = detect_motion(frame1, frame2)

            if motion_detected:
                last_motion_time = current_time
                if not self.motion:
                    first_motion_time = current_time
                    self.motion = True
                    print("Motion detected!")

            # If motion continues for threshold → start stream
            if self.motion and not self.stream_active:
                if current_time - first_motion_time >= MIN_MOTION_TIME:
                    print("Starting live stream...")
                    self.stream_active = True

            # If no motion for threshold → stop stream AND exit
            if self.stream_active and (current_time - last_motion_time >= NO_MOTION_TIME):
                print("No motion detected, stopping stream and exiting...")
                self.cleanup()
                break
        
            # Show stream only when active
            if self.stream_active:
                basler_frames = []

                # Grab frames from Basler cameras first
                for i, camera in enumerate(self.basler_cameras):
                    grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                    if grab_result.GrabSucceeded():
                        image = grab_result.GetArray()
                        basler_frames.append(image)
                    grab_result.Release()

                for i, frame in enumerate(basler_frames):
                    cv2.imshow(f'Basler Camera {i + 1}', frame)


                cv2.imshow("RealSense Stream", frame2)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("User exited.")
                    for camera in self.basler_cameras:
                        camera.StopGrabbing()
                        camera.Close()
                    self.pipeline.stop()
                    cv2.destroyAllWindows()

                    break
            # Update frames
            frame1, frame2 = frame2, self.get_realsense_frame()


if __name__ == "__main__":
    app = MotionCameraStream()
    app.run()