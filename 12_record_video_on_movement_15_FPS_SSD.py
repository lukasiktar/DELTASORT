import cv2
from pypylon import pylon
import numpy as np
import pyrealsense2 as rs
import time
import os
from src.config import MOTION_THRESHOLD, MIN_MOTION_TIME, NO_MOTION_TIME
from src.motion_detection import detect_motion
from src.camera_setup import CameraSetup

SSD_PATH = "/media/deltasort1/ADATA SD620/camera_data_videos"
os.makedirs(SSD_PATH, exist_ok=True)

# Timeout for program exit
TIMEOUT_SECONDS = 3600  # 1 hour

class MotionCameraStream(CameraSetup):

    def __init__(self):
        super().__init__()
        self.fps = 15  # Desired recording FPS
        self.frame_interval = 1.0 / self.fps
        self.recording = False
        self.motion = False
        self.stream_active = False
        self.running = True
        self.timestamp = None

    # --------------------------- Camera Frames ---------------------------

    def get_realsense_frame(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except Exception:
            return None
        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())

    # --------------------------- Recording Setup ---------------------------

    def start_recording(self):
        self.timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.out_basler_0 = cv2.VideoWriter(
            os.path.join(SSD_PATH, f'basler0_{self.timestamp}.avi'),
            cv2.VideoWriter_fourcc(*'XVID'),
            self.fps,
            (self.width, self.height)
        )
        self.out_basler_1 = cv2.VideoWriter(
            os.path.join(SSD_PATH, f'basler1_{self.timestamp}.avi'),
            cv2.VideoWriter_fourcc(*'XVID'),
            self.fps,
            (self.width, self.height)
        )
        self.out_rs = cv2.VideoWriter(
            os.path.join(SSD_PATH, f'realsense_{self.timestamp}.avi'),
            cv2.VideoWriter_fourcc(*'XVID'),
            self.fps,
            (self.width, self.height)
        )
        self.recording = True
        print(f"📹 Recording started: {self.timestamp}")

    def stop_recording(self):
        if self.recording:
            print(f"Stopping recording {self.timestamp}")
            self.out_basler_0.release()
            self.out_basler_1.release()
            self.out_rs.release()
            self.recording = False

    # --------------------------- Cleanup ---------------------------

    def cleanup(self):
        print("\nCleaning up resources...")
        self.stop_recording()
        try:
            for cam in self.basler_cameras:
                if cam.IsGrabbing():
                    cam.StopGrabbing()
                if cam.IsOpen():
                    cam.Close()
        except Exception as e:
            print(f"Error closing Basler cameras: {e}")

        try:
            self.pipeline.stop()
        except Exception as e:
            print(f"Error stopping RealSense pipeline: {e}")

        cv2.destroyAllWindows()
        print("Cleanup complete, exiting program.")

    # --------------------------- Main Loop ---------------------------

    def run(self):
        print("Starting motion monitoring...")
        frame1 = self.get_realsense_frame()
        frame2 = self.get_realsense_frame()
        last_motion_time = 0
        first_motion_time = 0
        start_time = time.time()
        last_frame_time = 0

        try:
            while self.running:
                # Timeout check
                if time.time() - start_time >= TIMEOUT_SECONDS:
                    print(f"Timeout reached ({TIMEOUT_SECONDS} seconds). Exiting...")
                    break

                if frame1 is None or frame2 is None:
                    frame2 = self.get_realsense_frame()
                    continue

                current_time = time.time()
                motion_detected = detect_motion(frame1, frame2)

                if motion_detected:
                    last_motion_time = current_time
                    if not self.motion:
                        first_motion_time = current_time
                        self.motion = True
                        print(" Motion detected!")

                # Start recording if motion sustained
                if self.motion and not self.stream_active and (current_time - first_motion_time >= MIN_MOTION_TIME):
                    print("Starting live stream...")
                    self.stream_active = True
                    self.start_recording()

                # Stop recording after no motion
                if self.stream_active and (current_time - last_motion_time >= NO_MOTION_TIME):
                    print("No motion detected, stopping stream...")
                    self.stop_recording()
                    self.stream_active = False
                    self.motion = False

                # Limit recording to desired FPS
                if self.stream_active and (current_time - last_frame_time >= self.frame_interval):
                    last_frame_time = current_time

                    # Capture Basler frames
                    basler_frames = []
                    for cam in self.basler_cameras:
                        grab_result = cam.RetrieveResult(1000, pylon.TimeoutHandling_Return)
                        if grab_result and grab_result.GrabSucceeded():
                            img = grab_result.GetArray()
                            if len(img.shape) == 2:
                                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                            img = cv2.resize(img, (self.width, self.height))
                            basler_frames.append(img)
                        if grab_result:
                            grab_result.Release()

                    # Write frames to files
                    if self.recording:
                        if len(basler_frames) > 0:
                            self.out_basler_0.write(basler_frames[0])
                        if len(basler_frames) > 1:
                            self.out_basler_1.write(basler_frames[1])
                        self.out_rs.write(frame2)

                    # Show live streams
                    for i, frame in enumerate(basler_frames):
                        cv2.imshow(f'Basler Camera {i+1}', frame)
                    cv2.imshow("RealSense Stream", frame2)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("✋ User exited.")
                        self.running = False
                        break

                # Prepare for next iteration
                frame1, frame2 = frame2, self.get_realsense_frame()

        except KeyboardInterrupt:
            print("\nKeyboard interrupt detected. Exiting...")

        finally:
            self.cleanup()


if __name__ == "__main__":
    app = MotionCameraStream()
    app.run()
