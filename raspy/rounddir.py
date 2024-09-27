

import cv2
import numpy as np


def find_round_dir(black_img: np.ndarray):
  lower = 30
  upper = 90
  edges_img = cv2.Canny(black_img, lower, upper, 3)
  # make the bottom row all white, so we don't detect the floor
  edges_img = cv2.line(edges_img, (0, 0), (edges_img.shape[1], 0), 1, 1)

  wall_heights = np.argmax(edges_img, axis=0)

  print("wh", wall_heights)

  differences = np.diff(wall_heights)
  print("diff", differences)


  # count the number of positive and negative jumps in the differences
  # if there are more positive jumps, we're going counter-clockwise
  # if there are more negative jumps, we're going clockwise
  # jumps need to be at least 17 pixels high
  MIN_JUMP = 9
  counter_clockwise = np.sum(differences > MIN_JUMP)
  clockwise = np.sum(differences < -MIN_JUMP)
  # print("cc", counter_clockwise)
  # print("cw", clockwise)
  print("rounddir:", clockwise > counter_clockwise)
  print("cw ", clockwise)
  print("ccw", counter_clockwise)
  return -1 if clockwise > counter_clockwise else 1
