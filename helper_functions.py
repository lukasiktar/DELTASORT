#Helper for DELTASORT NIR+RGB setup
from pypylon import pylon
import cv2

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


def classify_object(channel_list, obj_id, pkg):
    min_len = min(len(v) for v in channel_list)
    img_vectors = [np.array(v[:min_len], dtype=np.float32) for v in channel_list]
    band_stack = np.stack(img_vectors, axis=-1)  # (N, 4)

    # SNV per sample
    mean = band_stack.mean(axis=-1, keepdims=True)
    std  = band_stack.std(axis=-1,  keepdims=True)
    snv  = (band_stack - mean) / (std + 1e-6)

    S0, S1, S2, S3 = snv[:, 0], snv[:, 1], snv[:, 2], snv[:, 3]

    diff_01 = (S0 - S1).mean()
    diff_02 = (S0 - S2).mean()
    diff_03 = (S0 - S3).mean()
    diff_12 = (S1 - S2).mean()
    diff_13 = (S1 - S3).mean()
    diff_23 = (S2 - S3).mean()

    diff_dict={
        "diff_01": (S0-S1).mean(),
        "diff_02": (S0-S2).mean(),
        "diff_03": (S0-S3).mean(),
        "diff_12": (S1-S2).mean(),
        "diff_13": (S1-S3).mean(),
        "diff_23": (S2-S3).mean(),
    }
    snv_dict={
        "S0": S0.mean(),
        "S1": S1.mean(),
        "S2": S2.mean(),
        "S3": S3.mean(),
    }
    #print( f"S0:{S0.mean()}, S1:{S1.mean()}, S2:{S2.mean()}, S3:{S3.mean()}")
    #print( f"diff_01: {diff_01}, diff_02: {diff_02}, diff_03: {diff_03}, diff_12: {diff_12}, diff_13: {diff_13}, diff_23: {diff_23},")

    # ── Step 6: Build 10-feature stack for classification ──
    # [N1, N2, N3, N4, R12, R13, R14, R23, R24, R34]
    # Convert each dict to a 1D numpy array first, then concatenate
    diff_stack = np.array(list(diff_dict.values()), dtype=np.float32)  # shape (6,)
    snv_stack  = np.array(list(snv_dict.values()),  dtype=np.float32)  # shape (4,)
    feature_stack = np.concatenate([snv_stack,diff_stack])  # (H, W, 10)

    x = pkg['scaler'].transform([feature_stack])
    pred = pkg['model'].predict(x)[0]
    prob = pkg['model'].predict_proba(x)[0]
    spectral_label = ("HDPE" if pred == 1 else "PET"), round(max(prob)*100, 1)


    # # ── spectral classification ──────────────────────
    # if diff_12 > 0.3 and diff_12 < 0.7:
    #     if diff_13 < 2.0:
    #         spectral_label = "PET"
    #     elif diff_13 >= 2.0:
    #         spectral_label = "W PET"
    #     else:
    #         spectral_label = "Unknown"
    # elif diff_12 >= 0.7:
    #     spectral_label = "T_HDPE"
    # elif diff_12 <= 0.3 and diff_12 >= 0.02:
    #     spectral_label = "W_HDPE"
    # else:
    #     spectral_label = "Unknown"

    

    return spectral_label, feature_stack

    


    


# #Object classificator
# def classify_object(channel_list):
#     #Separate objects into the channels
#     # Find the smallest dimensions across all images
#     min_h = min(img.shape[0] for img in channel_list)
#     min_w = min(img.shape[1] for img in channel_list)

#     # Resize all to the same size
#     resized = [cv2.resize(img, (min_w, min_h), interpolation=cv2.INTER_AREA) 
#                for img in channel_list]

#     I1, I2, I3, I4 = [img.astype(np.float32) for img in resized]

#     #print(f"I1: {I1.shape}, I2: {I2.shape}, I3: {I3.shape}, I4: {I4.shape}")

#     eps = 1e-6
#     R12 = (I1-I2)/(I1+I2+eps)
#     R13 = (I1-I3)/(I1+I3+eps)
#     R14 = (I1-I4)/(I1+I4+eps)
#     R23 = (I2-I3)/(I2+I3+eps)
#     R24 = (I2-I4)/(I2+I4+eps)
#     R34 = (I3-I4)/(I3+I4+eps)
#     # Scalar features: mean absolute ratio per channel pair
#     # HDPE = large spectral differences, PET = small differences
#     features = {
#         "R12": np.mean(np.abs(R12)),
#         "R13": np.mean(np.abs(R13)),
#         "R14": np.mean(np.abs(R14)),
#         "R23": np.mean(np.abs(R23)),
#         "R24": np.mean(np.abs(R24)),
#         "R34": np.mean(np.abs(R34)),
#     }

#     # Overall spectral variability score
#     variability_score = np.mean(list(features.values()))
#     print(f"Variability score: {variability_score:.4f} | Features: {features}")

#     # Log to CSV for threshold calibration
#     with open("intensity_log.csv", "a") as f:
#         feature_str = ",".join(f"{v:.4f}" for v in features.values())
#         f.write(f"{feature_str},{variability_score:.4f}\n")

#     # Classify — tune threshold from your CSV observations
#     # HDPE has high variability, PET has low variability
#     THRESHOLD = 0.15  # ← adjust after observing logged scores
#     label = "HDPE" if variability_score > THRESHOLD else "PET"

#     # Save normalized images only for visualization
#     for name, ratio in [("R12",R12),("R13",R13),("R14",R14),
#                          ("R23",R23),("R24",R24),("R34",R34)]:
#         vis = cv2.normalize(ratio, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
#         cv2.imwrite(f"DELTASORT_image_processing/24_results/{name}.png", vis)

#     return label, variability_score, features


    return 1

#Centroid tracker
import numpy as np
from scipy.spatial import distance as dist
from collections import OrderedDict

class CentroidTracker:
    def __init__(self, max_disappeared = 30):
        self.next_id = 0
        self.objects = OrderedDict()
        self.disappeared =OrderedDict()
        self.attributes = OrderedDict()
        self.channels = OrderedDict()
        self.material=OrderedDict()
        self.max_disappeared = max_disappeared

    def register(self, object):
        self.objects[self.next_id] = object
        self.attributes[self.next_id] = []
        self.channels[self.next_id] = []
        self.material[self.next_id] = None
        
        self.disappeared[self.next_id] = 0
        self.next_id +=1

    def deregister(self, obj_id):
        del self.objects[obj_id]
        del self.disappeared[obj_id]
        del self.attributes[obj_id]
        del self.channels[obj_id]
        del self.material[obj_id]

    def update(self, detections):
        

        #IF there are no detections, mark them as disappeared
        if len(detections)== 0:
            for obj_id in list(self.disappeared.keys()):
                self.disappeared[obj_id] +=1
                if self.disappeared[obj_id] > self.max_disappeared:
                    self.deregister(obj_id)
            return self.objects,self.attributes, self.channels, self.material
        #ELSE if the object are detected, calculate object dimensions and centroids
        input_objects = np.array([(x+w//2, y+h//2, w, h) for (x,y,w,h) in detections])
        input_centroids = input_objects[:, 0:2]

        #IF no existing objects then register all
        if len(self.objects) == 0:
            for object in input_objects:
                self.register(object)

        #ELSE register just the new ones
        else:
            obj_ids = list(self.objects.keys())
            obj_centroids = np.array([obj[:2] for obj in self.objects.values()])
           
            #Compute pairwise distances between the existing and new centroids
            D = dist.cdist(np.array(obj_centroids), input_centroids)

            #Match by smallest distance 
            rows = D.min(axis=1).argsort()     #Sorts the closest deteciton for each object from matrix D, argsort sorts the objects by how confident the match is
            cols = D.argmin(axis=1)[rows]   #Sorts which detection is each object closest to

            used_rows, used_cols = set(), set()

            #Assign new  objects with calculated centroids to known objects
            for row, col in zip(rows, cols):
                if row in used_rows or col in used_cols:
                    continue
                obj_id = obj_ids[row]
                self.objects[obj_id] = input_objects[col]
                self.disappeared[obj_id]=0
                used_rows.add(row)
                used_cols.add(col)

            #Handle unmatched existing objects
            for row in set(range(len(obj_ids)))-used_rows:
                obj_id = obj_ids[row]
                self.disappeared[obj_id] +=1
                if self.disappeared[obj_id] > self.max_disappeared:
                    self.deregister(obj_id)

            #Register new unmatched detections
            for col in set(range(len(input_objects)))- used_cols:
                self.register(input_objects[col])

        return self.objects,self.attributes, self.channels, self.material
    


