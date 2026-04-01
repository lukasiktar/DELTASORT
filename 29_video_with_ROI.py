import cv2
from pypylon import pylon
import numpy as np


H=np.array([[ 8.83389491e-01,  1.45497747e-01,  5.10480420e+01],
 [-1.43517841e-02,  9.74895292e-01,  2.16090969e+02],
 [-2.32915049e-05,  2.11809073e-04,  1.00000000e+00]])

#NIR camera dimensions
height = 1026
width = 1282

def connect_to_cameras(num_cameras):
    """Connect to multiple cameras."""
    cameras = []
    
    # All available cameras
    devices = pylon.TlFactory.GetInstance().EnumerateDevices()
    for i, device in enumerate(devices):
        print(f"Camera {i}: Serial={device.GetSerialNumber()}, Model={device.GetModelName()}")

    # Check if the number of cameras exceeds available devices
    if len(devices) < num_cameras:
        raise ValueError(f"Only {len(devices)} cameras found, but {num_cameras} requested.")

    # Connect to each camera
    for i in range(num_cameras):
        camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[i]))
        camera.Open()
        # Configure Trigger
        camera.TriggerSelector.SetValue("FrameStart")
        camera.TriggerMode.SetValue("On")
        camera.TriggerSource.SetValue("Line1") # Or "Software"
        camera.TriggerActivation.SetValue("RisingEdge")
        camera.PixelFormat.SetValue("Mono8")
        
        camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        cameras.append(camera)

    return cameras

def main():
    # The number of cameras you want to connect 
    num_cameras = 2 

    try:
        cameras = connect_to_cameras(num_cameras)

        # Create OpenCV windows for each camera
        for i in range(num_cameras):
            cv2.namedWindow(f'Basler Camera {i + 1}', cv2.WINDOW_NORMAL)

        while True:
            frames = []

            # Grab a frame from each camera
            for i, camera in enumerate(cameras):
                grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                
                if grab_result.GrabSucceeded():
                    image = grab_result.GetArray()
                    if i == 0:
                        image=cv2.rotate(image, cv2.ROTATE_180)
                        # Warp cam1 into cam2's view
                        image = cv2.warpPerspective(image, H, (height, width))
                        #ROI
                        image = image[200:height-100,400:width-450]
                        frames.append(image)
                    else:
                        #ROI
                        image = image[200:height-100,400:width-450]
                        frames.append(image)
                grab_result.Release()

            # Display each frame in its respective OpenCV window
            for i, frame in enumerate(frames):
                cv2.imshow(f'Basler Camera {i + 1}', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        for camera in cameras:
            camera.StopGrabbing()
            camera.Close()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
