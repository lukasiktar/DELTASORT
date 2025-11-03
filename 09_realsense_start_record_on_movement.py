import pyrealsense2 as rs
import numpy as np
import cv2
import time
from src.config import MOTION_THRESHOLD, MIN_MOTION_TIME, NO_MOTION_TIME
from src.motion_detection import detect_motion


class RealSenseMotionStream:
    def __init__(self):
        print("Initializing RealSense camera...")

        # Setup RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        #self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)

        # Align depth to color 
        self.align = rs.align(rs.stream.color)

        # Control flags
        self.motion = False
        self.stream_active = False

        print("Camera initialized and ready.")

    def get_frame(self):
        """Return the latest RGB frame as a numpy array."""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())

    def cleanup(self):
        print("Stopping pipeline and closing windows...")
        try:
            self.pipeline.stop()
        except Exception as e:
            print(f"Pipeline already stopped: {e}")
        cv2.destroyAllWindows()

    def run(self):
        print("Starting motion monitoring...")

        frame1 = self.get_frame()
        frame2 = self.get_frame()

        last_motion_time = 0
        first_motion_time = 0

        while True:
            if frame1 is None or frame2 is None:
                frame2 = self.get_frame()
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
                cv2.imshow("RealSense Stream", frame2)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("User exited.")
                    self.cleanup()
                    break

            # Update frames
            frame1, frame2 = frame2, self.get_frame()


if __name__ == "__main__":
    app = RealSenseMotionStream()
    app.run()
