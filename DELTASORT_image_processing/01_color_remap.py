import cv2
import numpy as np
import os

os.makedirs("results", exist_ok=True)

image_paths = [
    "/home/istrazivac/LukaSiktar/DELTASORT/basler_samples/basler2_channel0.png",
    "/home/istrazivac/LukaSiktar/DELTASORT/basler_samples/basler2_channel1.png",
    "/home/istrazivac/LukaSiktar/DELTASORT/basler_samples/basler2_channel2.png",
    "/home/istrazivac/LukaSiktar/DELTASORT/basler_samples/basler2_channel3.png",
    "/home/istrazivac/LukaSiktar/DELTASORT/basler_samples/basler2_channel4.png"]

img_jets=[]
for i, path in enumerate(image_paths):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise IOError(f"Could not read image: {path}")

    # Percentile normalization
    p_low, p_high = np.percentile(img, (5, 99))
    img = np.clip(img, p_low, p_high)
    img_norm = ((img - p_low) / (p_high - p_low) * 255).astype(np.uint8)

    # Save normalized image
    cv2.imwrite(f"results/channel{i}_normalized.png", img_norm)

    # Apply JET colormap (visualization only)
    img_jet = cv2.applyColorMap(img_norm, cv2.COLORMAP_JET)
    img_jets.append(img_jet)
    cv2.imwrite(f"results/channel{i}_jet.png", img_jet)


#The combination of img_jet0 and img_jet1
img_jet0 = img_jets[0]
img_jet1 = img_jets[1]

diff_jet01 = cv2.absdiff(img_jet0, img_jet1)
diff_gray_jet01 = cv2.cvtColor(diff_jet01, cv2.COLOR_BGR2GRAY)

cv2.imwrite("results/diff_gray_jet01.png", diff_gray_jet01)

ret, thresh_diff = cv2.threshold(diff_gray_jet01,100,255,cv2.THRESH_BINARY)

kernel = np.ones((3,3), np.uint8)
thresh_diff = cv2.erode(thresh_diff, kernel, iterations=1)

cv2.imwrite("results/diff_thresh_jet01.png", thresh_diff)


#The combination of img_jet1 and img_jet2
img_jet1 = img_jets[1]
img_jet2 = img_jets[2]

diff_jet12 = cv2.absdiff(img_jet1, img_jet2)
diff_gray_jet12 = cv2.cvtColor(diff_jet12, cv2.COLOR_BGR2GRAY)

cv2.imwrite("results/diff_gray_jet12.png", diff_gray_jet12)

ret, thresh_diff = cv2.threshold(diff_gray_jet12,100,255,cv2.THRESH_BINARY)

kernel = np.ones((3,3), np.uint8)
thresh_diff = cv2.erode(thresh_diff, kernel, iterations=1)

cv2.imwrite("results/diff_thresh_jet12.png", thresh_diff)

#The combination of img_jet2 and img_jet3
img_jet2 = img_jets[2]
img_jet3 = img_jets[3]

diff_jet23 = cv2.absdiff(img_jet2, img_jet3)
diff_gray_jet23 = cv2.cvtColor(diff_jet23, cv2.COLOR_BGR2GRAY)

cv2.imwrite("results/diff_gray_jet23.png", diff_gray_jet23)

ret, thresh_diff = cv2.threshold(diff_gray_jet23,100,255,cv2.THRESH_BINARY)

kernel = np.ones((3,3), np.uint8)
thresh_diff = cv2.erode(thresh_diff, kernel, iterations=1)

cv2.imwrite("results/diff_thresh_jet23.png", thresh_diff)

#The combination of img_jet3 and img_jet4
img_jet3 = img_jets[3]
img_jet4 = img_jets[4]

diff_jet34 = cv2.absdiff(img_jet3, img_jet4)
diff_gray_jet34 = cv2.cvtColor(diff_jet34, cv2.COLOR_BGR2GRAY)

cv2.imwrite("results/diff_gray_jet34.png", diff_gray_jet34)

ret, thresh_diff = cv2.threshold(diff_gray_jet34,100,255,cv2.THRESH_BINARY)

kernel = np.ones((3,3), np.uint8)
thresh_diff = cv2.erode(thresh_diff, kernel, iterations=1)

cv2.imwrite("results/diff_thresh_jet34.png", thresh_diff)



