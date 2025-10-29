import cv2
from pypylon import pylon
import numpy as np

def connect_to_cameras(num_cameras):
    """Connect to multiple cameras."""
    cameras = []
    
    # Enumerate all available cameras
    devices = pylon.TlFactory.GetInstance().EnumerateDevices()

    # Check if the number of cameras exceeds available devices
    if len(devices) < num_cameras:
        raise ValueError(f"Only {len(devices)} cameras found, but {num_cameras} requested.")

    # Connect to each camera
    for i in range(num_cameras):
        camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[i]))
        camera.Open()
        camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        cameras.append(camera)

    return cameras

def main():
    # Set the number of cameras you want to connect to
    num_cameras = 2  # Example for 2 cameras

    try:
        # Connect to the cameras
        cameras = connect_to_cameras(num_cameras)

        # Create OpenCV windows for each camera
        for i in range(num_cameras):
            cv2.namedWindow(f'Basler Camera {i + 1}', cv2.WINDOW_NORMAL)

        # Start grabbing images from each camera
        while True:
            frames = []

            # Grab a frame from each camera
            for i, camera in enumerate(cameras):
                grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                
                if grab_result.GrabSucceeded():
                    image = grab_result.GetArray()
                    frames.append(image)
                grab_result.Release()

            # Display each frame in its respective OpenCV window
            for i, frame in enumerate(frames):
                cv2.imshow(f'Basler Camera {i + 1}', frame)

            # Break if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Stop grabbing and close cameras
        for camera in cameras:
            camera.StopGrabbing()
            camera.Close()

        # Close all OpenCV windows
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
