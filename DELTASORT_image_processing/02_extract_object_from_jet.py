import cv2
import numpy as np

img = cv2.imread("/home/istrazivac/LukaSiktar/DELTASORT/results/channel1_jet.png")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)         

# Smooth a bit to avoid tiny fluctuations
gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

# Simple threshold to separate foreground from background
_, mask = cv2.threshold(gray_blur, 20, 255, cv2.THRESH_BINARY)  

# Connected components on mask
num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)  

print(f"Objects: {num_labels}")

best_label = -1
best_score = 1e9   # lower = more uniform

for lab in range(1, num_labels):          # skip background 0
    area = stats[lab, cv2.CC_STAT_AREA]
    if area < 700:                         # ignore tiny blobs
        continue

    comp_mask = (labels == lab)
    # Compute mean and variance over original color image
    pixels = img[comp_mask]
    mean  = pixels.mean(axis=0)
    var   = ((pixels - mean) ** 2).mean()  # scalar variance over all channels 

    score = var / area                     # prefer large, uniform regions
    if score < best_score:
        best_score = score
        best_label = lab
    

uniform_mask = (labels == best_label).astype("uint8") * 255
uniform_obj  = cv2.bitwise_and(img, img, mask=uniform_mask)  

cv2.imshow("Most uniform object", uniform_obj)
cv2.waitKey(0)
cv2.destroyAllWindows()
