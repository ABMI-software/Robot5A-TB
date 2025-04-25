import numpy as np
import cv2 as cv
import glob
import yaml
import os
import matplotlib.pyplot as plt



def remove_distortion(camMatrix, distCoeff):
    root = os.getcwd()
    imgPath = os.path.join(root,'cvqsf.jpg')
    img = cv.imread(imgPath)
    height, width = img.shape[:2]
    camMatrixNew,roi = cv.getOptimalNewCameraMatrix(camMatrix, distCoeff, (width,height), 1, (width,height))
    imgUndist = cv.undistort(img, camMatrix, distCoeff, None, camMatrixNew)

    plt.figure()
    plt.subplot(121)
    plt.imshow(img)
    plt.subplot(122)
    plt.imshow(imgUndist)
    plt.show()
