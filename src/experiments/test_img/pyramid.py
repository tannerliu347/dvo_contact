import numpy as np, matplotlib as mpl, matplotlib.pyplot as plt, os
import urllib.request
import math
from PIL import Image
import scipy.ndimage # For image filtering
from scipy import signal
from scipy import fft
import imageio # For loading images
import cv2

def common_canvas_op(im0, im1, offset=1):
  size_im0 = np.shape(im0)
  size_im1 = np.shape(im1)

  canvas_size = max(size_im0[0], size_im1[0]), max(size_im0[1], size_im1[1]), 3
  canvas_im0 = np.zeros(canvas_size)
  canvas_im1 = np.zeros(canvas_size)

  canvas_im0[0:size_im0[0], 0:size_im0[1], :] = im0
  canvas_im1[0:size_im1[0], 0:size_im1[1], :] = im1

  return canvas_im0 + offset * canvas_im1

def pyramid_downsample(img, kernel_size=(5,5)):
  """
  Downsamples the given pyramid image.
  Input:
    - img: an image of shape M x N x C
    - kernel_size: a tuple representing the shape of the 2D kernel
  Returns: 
    - downsampled: an image of shape M/2 x N/2 x C
  """
  #############################################################################
  # TODO: Implement pyramid downsampling.                                     #
  #############################################################################
  original_sizes = np.shape(img)
  # print("DEBUG original size not correct? ", original_sizes)
  downsampled_size = original_sizes[0] // 2, original_sizes[1] // 2 # , original_sizes[2]

  downsampled = cv2.GaussianBlur(img, ksize=kernel_size, sigmaX=1)
  downsampled = downsampled[0:original_sizes[0]:2, 0:original_sizes[1]:2]
  #############################################################################
  #                              END OF YOUR CODE                             #
  #############################################################################
  return downsampled


if __name__ == "__main__":
    img1_intensity = cv2.imread("img1_old.png", cv2.IMREAD_GRAYSCALE)
    img2_intensity = cv2.imread("img2_old.png", cv2.IMREAD_GRAYSCALE)
    img1_depth = cv2.imread("img1_depth_old.png", cv2.IMREAD_ANYDEPTH)
    img2_depth = cv2.imread("img2_depth_old.png", cv2.IMREAD_ANYDEPTH)
    
    img1_pyramid_1 = pyramid_downsample(img1_intensity)
    img1_depth_pyramid_1 = pyramid_downsample(img1_depth)
    cv2.imwrite("img1_pyramid_1.png", img1_pyramid_1)
    cv2.imwrite("img1_depth_pyramid_1.png", img1_depth_pyramid_1.astype(np.uint16))
    
    img2_pyramid_1 = pyramid_downsample(img2_intensity)
    img2_depth_pyramid_1 = pyramid_downsample(img2_depth)
    cv2.imwrite("img2_pyramid_1.png", img2_pyramid_1)
    cv2.imwrite("img2_depth_pyramid_1.png", img2_depth_pyramid_1.astype(np.uint16))
    

    # print("DEBUG channel depth of img1_depth", img1_depth[100, 100])

