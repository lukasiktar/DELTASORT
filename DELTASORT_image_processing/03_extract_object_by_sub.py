import cv2
import numpy as np
import os

os.makedirs("results_03", exist_ok=True)

image_paths = [
    "/home/istrazivac/LukaSiktar/DELTASORT/basler_samples/basler2_channel0.png",
    "/home/istrazivac/LukaSiktar/DELTASORT/basler_samples/basler2_channel1.png",
    "/home/istrazivac/LukaSiktar/DELTASORT/basler_samples/basler2_channel2.png",
    "/home/istrazivac/LukaSiktar/DELTASORT/basler_samples/basler2_channel3.png",
    "/home/istrazivac/LukaSiktar/DELTASORT/basler_samples/basler2_channel4.png"]

imgs_norm=[]
for i, path in enumerate(image_paths):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise IOError(f"Could not read image: {path}")

    # Percentile normalization
    p_low, p_high = np.percentile(img, (5, 99))
    img = np.clip(img, p_low, p_high)
    img_norm = ((img - p_low) / (p_high - p_low) * 255).astype(np.uint8)
    imgs_norm.append(img_norm)

    # Save normalized image
    cv2.imwrite(f"results_03/channel{i}_normalized.png", img_norm)


#The combination of img_jet0 and img_jet1
img_jet0 = imgs_norm[0]
img_jet1 = imgs_norm[1]

diff_jet01 = cv2.absdiff(img_jet0, img_jet1)

cv2.imwrite("results_03/diff_gray_01.png", diff_jet01)

ret, thresh_diff = cv2.threshold(diff_jet01,100,255,cv2.THRESH_BINARY)

kernel = np.ones((3,3), np.uint8)
thresh_diff = cv2.erode(thresh_diff, kernel, iterations=1)

cv2.imwrite("results_03/diff_thresh_01.png", thresh_diff)


#The combination of img_jet1 and img_jet2
img_jet1 = imgs_norm[1]
img_jet2 = imgs_norm[2]

diff_jet12 = cv2.absdiff(img_jet1, img_jet2)

cv2.imwrite("results_03/diff_gray_12.png", diff_jet12)

ret, thresh_diff = cv2.threshold(diff_jet12,100,255,cv2.THRESH_BINARY)

kernel = np.ones((3,3), np.uint8)
thresh_diff = cv2.erode(thresh_diff, kernel, iterations=1)

cv2.imwrite("results_03/diff_thresh_12.png", thresh_diff)

#The combination of img_jet2 and img_jet3
img_jet2 = imgs_norm[2]
img_jet3 = imgs_norm[3]

diff_jet23 = cv2.absdiff(img_jet2, img_jet3)

cv2.imwrite("results_03/diff_gray_23.png", diff_jet23)

ret, thresh_diff = cv2.threshold(diff_jet23,100,255,cv2.THRESH_BINARY)

kernel = np.ones((3,3), np.uint8)
thresh_diff = cv2.erode(thresh_diff, kernel, iterations=1)

cv2.imwrite("results_03/diff_thresh_23.png", thresh_diff)

#The combination of img_jet3 and img_jet4
img_jet3 = imgs_norm[3]
img_jet4 = imgs_norm[4]

diff_jet34 = cv2.absdiff(img_jet3, img_jet4)

cv2.imwrite("results_03/diff_gray_34.png", img_jet3)

ret, thresh_diff = cv2.threshold(img_jet3,100,255,cv2.THRESH_BINARY)

kernel = np.ones((3,3), np.uint8)
thresh_diff = cv2.erode(thresh_diff, kernel, iterations=1)

cv2.imwrite("results_03/diff_thresh_34.png", thresh_diff)



