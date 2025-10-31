import cv2
from pypylon import pylon
import numpy as np
from pypylon import genicam

def connect_to_cameras(num_cameras):
    """Connect to multiple Basler cameras and print GenICam parameters."""
    cameras = []
    devices = pylon.TlFactory.GetInstance().EnumerateDevices()

    if len(devices) < num_cameras:
        raise ValueError(f"Only {len(devices)} cameras found, but {num_cameras} requested.")

    print(f"\nFound {len(devices)} Basler cameras. Connecting to first {num_cameras}...\n")

    for i in range(num_cameras):
        cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[i]))
        cam.Open()

        # Access the node map (GenICam feature set)
        nodemap = cam.GetNodeMap()
        model = cam.GetDeviceInfo().GetModelName()
        serial = cam.GetDeviceInfo().GetSerialNumber()

        print(f"Camera {i+1} ")
        print(f"Model: {model}")
        print(f"Serial: {serial}")

        # Helper to safely get a node's value
        def safe_get(node_name):
            try:
                node = nodemap.GetNode(node_name)
                if genicam.IsReadable(node):
                    return node.GetValue()
                else:
                    return None
            except Exception:
                return None

        # Try reading relevant parameters
        exposure_auto = safe_get("ExposureAuto")
        exposure_time = safe_get("ExposureTimeAbs") or safe_get("ExposureTime")
        gain_auto = safe_get("GainAuto")
        gain = safe_get("Gain")
        frame_rate = safe_get("AcquisitionFrameRate")

        print(f"ExposureAuto: {exposure_auto}")
        print(f"ExposureTime: {exposure_time} µs")
        print(f"GainAuto: {gain_auto}")
        print(f"Gain: {gain}")
        print(f"FrameRate: {frame_rate} fps\n")

        cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        cameras.append(cam)

    return cameras


def main():
    num_cameras = 2

    try:
        cameras = connect_to_cameras(num_cameras)

        for i in range(num_cameras):
            cv2.namedWindow(f'Basler Camera {i + 1}', cv2.WINDOW_NORMAL)

        print("Streaming, press 'q' to quit.\n")

        while True:
            frames = []
            for i, cam in enumerate(cameras):
                grab = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grab.GrabSucceeded():
                    img = grab.GetArray()
                    frames.append(img)
                grab.Release()

            for i, frame in enumerate(frames):
                if frame.dtype != np.uint8:
                    frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
                    frame = np.uint8(frame)
                cv2.imshow(f'Basler Camera {i + 1}', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        for cam in cameras:
            cam.StopGrabbing()
            cam.Close()
        cv2.destroyAllWindows()
        print("\nAll cameras closed successfully.\n")


if __name__ == "__main__":
    main()
