import numpy as np


class Pillar:
  def __init__(self, screen_x: int, width: int, height: int, color: str):
    self.screen_x = screen_x
    self.width = width
    self.height = height
    self.color = color

def extract_ROI(image: np.ndarray, startxy: list, endxy: list) -> np.ndarray:
  """
  Extracts the ROIs from the image
  """
  
  return image[startxy[1]:endxy[1], startxy[0]:endxy[0]]