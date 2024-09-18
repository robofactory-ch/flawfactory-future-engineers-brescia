

import cv2
import numpy as np


def find_round_dir(black_img: np.ndarray):
  lower = 30
  upper = 90
  edges_img = cv2.Canny(black_img, lower, upper, 3)

  wall_heights = np.argmax(edges_img, axis=0)
  differences = np.diff(wall_heights)

  if np.sum(np.power(differences, 4.0)) > 0:
    print("Counter clockwise")
    return -1
  else:
    print("Clockwise")
    return 1

