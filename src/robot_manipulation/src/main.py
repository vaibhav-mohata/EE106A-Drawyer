import os
import numpy as np
import matplotlib.pyplot as plt
import sys

CWD = os.getcwd()
IMG_DIR = CWD + '/img/'
SAVE_IM_DIR = CWD + '/lab4_cam/src'
sys.path.insert(0, SAVE_IM_DIR)

# import save_image as s
# import save_webcam_img as cam_to_img 
import image_to_canny as im_2_can
import draw as d

##########################################################
#Wrapper program. Runs:
#1. save_image.py #take image from webcam and save in /img folder.
#2 image_preprocess.py #isolate face in image.
#2. image_to_canny.py #take raw image and run canny edge detection with adjustable thresholds. save edge image in folder.
#4. draw.py #takes cleaned image and creates trajectory for saweyer to draw.
##########################################################


#Step 1
# img, save_name = cam_to_img.main()

# #Step 2
# #TODO: preprocessing... do if needed, but for rn median filtering + blurring works fine

# #Step 3
# canny_img = im_2_can.main(save_name)

#testing
# img = cv2.imread(IMG_DIR + "heath_edges.jpg", 0)

# canny_img = im_2_can.main("abcd")
canny_img = im_2_can.read_image(IMG_DIR + "abcd_edges.jpg", grayscale=True)
canny_img_straight = canny_img.reshape((1, -1))
LIMIT = 1
# print(len([x for x in canny_img_straight[0] if x > LIMIT]))

# canny_img[canny_img < 10] = 0
canny_img[canny_img >= 1] = 1
print(canny_img.shape)
# print(canny_img)

plt.imshow(canny_img)
plt.show()

paths = list(d.array_to_path(canny_img))
paths = sorted(paths, key=lambda x: len(x), reverse=True)
print(paths)
# print(len(paths))
# for path in paths:
#     print(len(path))


