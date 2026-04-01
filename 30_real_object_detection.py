import cv2
from pypylon import pylon
import numpy as np
import helper_functions
from config.config import H,NIR_WIDTH, NIR_HEIGHT
import csv
import os
import joblib
from scipy.ndimage import binary_fill_holes
ct = helper_functions.CentroidTracker(max_disappeared=4)

def main():
    # The number of cameras you want to connect 
    num_cameras = 2 

    try:
        cameras = helper_functions.connect_to_cameras(num_cameras)

        # Create OpenCV windows for each camera
        for i in range(num_cameras):
            cv2.namedWindow(f'Basler Camera {i + 1}', cv2.WINDOW_NORMAL)

        pkg = joblib.load('plastic_classifier02.pkl')
        frame_counter=0     #The counter conunts each 4 images - for 4 channels
        while True:
            frames = []
            bounding_boxes=[] #The number of bounding boxes detected on RGB image represents the number of objects that needs to be examined using NIR
            

            thresh_rgb_image = None     #Store the thresholded rgb image for intensity analysis
            # Grab a frame from each camera
            for i, camera in enumerate(cameras):
                grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grab_result.GrabSucceeded():
                    image = grab_result.GetArray()

                    #1
                    if i == 0:  #RGB camera that needs to be rotated and aligned to the NIR
                        image=cv2.rotate(image, cv2.ROTATE_180)
                        # Warp cam1 into cam2's view
                        image = cv2.warpPerspective(image, H, (NIR_HEIGHT, NIR_WIDTH))
                        #ROI
                        image = image[200:NIR_HEIGHT-200,420:NIR_WIDTH-470].astype(np.uint8)

                        #Detecting and segmenting objects
                        # CLAHE for local contrast boost 
                        clahe = cv2.createCLAHE(clipLimit=1.5, tileGridSize=(12, 12))
                        enhanced = clahe.apply(image)

                        # Bilateral filter — keeps sharp edges, kills noise 
                        denoised = cv2.bilateralFilter(enhanced, d=2, sigmaColor=90, sigmaSpace=90)

                        #  Adaptive threshold 
                        thresh = cv2.adaptiveThreshold(denoised, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                                        cv2.THRESH_BINARY, 91, -15)

                        #  Morphological cleanup 
                        k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                        k_open  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                        closed  = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, k_close, iterations=7)
                        cleaned = cv2.morphologyEx(closed,  cv2.MORPH_OPEN,  k_open,  iterations=4)

                        cleaned = binary_fill_holes(cleaned).astype(np.uint8) * 255

                        #  Connected component segmentation
                        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(cleaned, connectivity=8)

                        # Minimum area to ignore noise 
                        min_area = 2000


                        for i in range(1, num_labels):  # skip background (0)
                            area = stats[i, cv2.CC_STAT_AREA]

                            if area > min_area:
                                x = stats[i, cv2.CC_STAT_LEFT]
                                y = stats[i, cv2.CC_STAT_TOP]
                                w = stats[i, cv2.CC_STAT_WIDTH]
                                h = stats[i, cv2.CC_STAT_HEIGHT]
                                bbox = [x,y,w,h]
                                
                                bounding_boxes.append(bbox)
                                
                                # draw bounding box for visualization
                                #cv2.rectangle(cleaned, (x,y), (x+w,y+h), 128, 2)

                        frames.append(cleaned)

                    #2
                    else:
                        #ROI
                        image = image[200:NIR_HEIGHT-200,420:NIR_WIDTH-470]

                        #Try to segment objects if the annnotations are ready
                        try:
                            for bbox in bounding_boxes:
                                x,y,w,h= bbox

                                # draw bounding box for visualization
                                cv2.rectangle(image, (x,y), (x+w,y+h), (128,128,255), 2)

                        except:
                            print("No annotations")



                        frames.append(image)
                grab_result.Release()
            
            objects = ct.update(bounding_boxes)
            #Object is a instance with an obj_id and numpy array [centroid_x, centroid_y, width, hieght]
            
            for obj_id in objects[0].keys():
                #Calculate the properties for each 4 channels when they arrive
                if frame_counter == 4:
                    #Mean intensity
                    mean_intensity = np.mean(objects[1][obj_id])    
                    #Decide which object is present using the intensity
                    if mean_intensity > 120.0:
                        i_label = "HDPE"                       
                    elif mean_intensity > 30.0:
                        i_label = "PET"
                    else:
                        i_label = "Unknown"
                        
                    
                    

                    try:
                        if len(objects[2][obj_id])==4:
                                label, feature_stack = helper_functions.classify_object(objects[2][obj_id], obj_id, pkg)



                        objects[3][obj_id] = f"{label[0]}"
                    except:
                        print(f"Not possible to detect object type for {obj_id}")

                    # #TURN ON for storing dataset **************
                    # key = cv2.waitKey(1) & 0xFF
                    # file_exists = os.path.isfile("model_train_data02.csv")
                    # # Write header only once
                    # with open("model_train_data02.csv", "a", newline="") as f:
                    #         writer = csv.writer(f)
                    #         if not file_exists:
                    #             header = ["label", "S0", "S1", "S2", "S3",
                    #                     "diff_01", "diff_02", "diff_03",
                    #                     "diff_12", "diff_13", "diff_23"]
                    #             writer.writerow(header)


                    # if key == ord('p'):          # P → save as PET
                    #     with open("model_train_data02.csv", "a", newline="") as f:
                    #         writer = csv.writer(f)
                    #         label="PET"
                    #         # Write data row: label + 10 feature values
                    #         row = [label] + feature_stack.tolist()
                    #         writer.writerow(row)

                    # elif key == ord('h'):        # H → save as HDPE
                    #     with open("model_train_data02.csv", "a", newline="") as f:
                    #         writer = csv.writer(f)
                            
                    #         label="HDPE"
                    #         # Write data row: label + 10 feature values
                    #         row = [label] + feature_stack.tolist()
                    #         writer.writerow(row)
                    # #TURN ON for storing dataset *************

                    
                    #Reset all the states
                    objects[1][obj_id]=[]
                    objects[2][obj_id] = []
                    
                
                #Extract the region of interest for each object
                x,y,w,h = objects[0][obj_id]
                extracted_object = image[y-h//2: y+h//2, x-w//2: x+w//2] 
                #print(obj_id)
                extracted_segmentation = frames[0][y-h//2: y+h//2, x-w//2: x+w//2]
            
                # Only pixels where segmentation mask is non-zero
                segmentation_mask = extracted_segmentation > 0
                segmentation_pixels = extracted_object[segmentation_mask]
                #print(segmentation_pixels.shape)
                objects[2][obj_id].append(segmentation_pixels)
                #Average intensity of the object image
                seg_avg_intensity = np.mean(segmentation_pixels) if segmentation_pixels.size > 0 else 0.0
                objects[1][obj_id].append([seg_avg_intensity])
                try:
                    cv2.putText(image, f"{objects[3][obj_id]}", (x-w//2, y-h//2-10), 1, 1.0, (255,255,255))
                except:
                    pass

                    
               
                
            #print(f"object: {objects} \n")


            # Display each frame in its respective OpenCV window
            for i, frame in enumerate(frames):
                cv2.imshow(f'Basler Camera {i + 1}', frame)
            if frame_counter == 4:
                frame_counter = 0
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            frame_counter +=1 
            
    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        for camera in cameras:
            camera.StopGrabbing()
            camera.Close()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
