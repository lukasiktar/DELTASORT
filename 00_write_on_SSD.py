import cv2
import numpy as np
import os

SSD_PATH = "/media/deltasort1/ADATA SD620/camera_data"
os.makedirs(SSD_PATH, exist_ok=True)

img = np.zeros((100, 100, 3), dtype=np.uint8)  # black image

file_path = os.path.join(SSD_PATH, "test.jpg")

try:
    cv2.imwrite(file_path, img)
    print(f"Image written successfully: {file_path}")
except Exception as e:
    print(f" Failed to write image: {e}")
