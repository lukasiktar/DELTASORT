import cv2
from pypylon import pylon

def connect_to_camera():
    # Create an instance of the camera
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    
    # Open the camera
    camera.Open()

    # Set the camera to continuous image acquisition mode
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    
    return camera

def main():
    # Connect to the Basler camera
    camera = connect_to_camera()

    # Create an OpenCV window
    cv2.namedWindow('Basler Camera Stream', cv2.WINDOW_NORMAL)

    try:
        while True:
            # Grab the frame from the camera
            grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            if grab_result.GrabSucceeded():
                # Convert the image to a numpy array for OpenCV processing
                image = grab_result.GetArray()

                # Display the image using OpenCV
                cv2.imshow('Basler Camera Stream', image)

            # Release the grab result object
            grab_result.Release()

            # Break the loop if the 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # Release the camera when done
        camera.StopGrabbing()
        camera.Close()

        # Close the OpenCV window
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
