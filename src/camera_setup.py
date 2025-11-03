from pypylon import pylon
import pyrealsense2 as rs

class CameraSetup:

    def __init__(self):
        print("Initializing Basler cameras...")
        self.basler_cameras = self.connect_to_basler_cameras(num_cameras=2)

        print("Initializing RealSense camera...")
        self.connect_to_realsense()

        self.width, self.height, self.fps = 640, 480, 30
        self.recording = False
        self.motion = False
        self.stream_active = False
        self.running = True  # master flag



    def connect_to_basler_cameras(self, num_cameras):
        cameras = []
        devices = pylon.TlFactory.GetInstance().EnumerateDevices()
        if len(devices) < num_cameras:
            raise ValueError(f"Only {len(devices)} Basler cameras found, but {num_cameras} requested.")
        for i in range(num_cameras):
            cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[i]))
            cam.Open()
            cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            cameras.append(cam)
            print(f"Connected to Basler Camera {i + 1}")
        return cameras

    def connect_to_realsense(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)