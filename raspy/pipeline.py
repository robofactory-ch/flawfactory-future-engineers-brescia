import math
import cv2
import numpy as np

from config import configloader
from helpers import Pillar

class Pipeline:
  def __init__(self, configloader: configloader):
    self.configloader = configloader

  def undistort(self, image: np.ndarray):
    mtx, dist = np.array(self.configloader.get_property("camera")['mtx']), np.array(self.configloader.get_property("camera")['dist'])
    return cv2.undistort(image, mtx, dist, None, mtx)

  def crop(self, image: np.ndarray):
    crop_height = int(self.configloader.get_property("camera")['crop_height'])
    return image[crop_height:,:]

  def filter_RG_Bl(self, hsv: np.ndarray, color_image: np.ndarray):
    """
    Extracts the red, green and black colors from the image -> thiis used to detect the pillars and walls
    """
    redMin = tuple(self.configloader.get_property("filters")['REDLO'])
    redMax = tuple(self.configloader.get_property("filters")['REDHI'])
    greenMin = tuple(self.configloader.get_property("filters")['GREENLO'])
    greenMax = tuple(self.configloader.get_property("filters")['GREENHI'])
    grayThresh = int(self.configloader.get_property("filters")['GRAY'])


    # red filter
    # red is at 0 and also 180, accounting for HSV wraparound
    rMask1 = cv2.inRange(hsv, redMin, redMax)
    redMinList = list(redMin)
    redMinList = [180 - redMax[0], redMinList[1], redMinList[2]]
    redMin2 = tuple(redMinList)
    redMaxList = list(redMax)
    redMaxList = [180, redMaxList[1], redMaxList[2]]
    redMax2 = tuple(redMaxList)
    rMask2 = cv2.inRange(hsv, redMin2, redMax2)
    rMask = cv2.bitwise_or(rMask1, rMask2)
    # green filter
    gMask = cv2.inRange(hsv, greenMin, greenMax)
    # blur images to remove noise
    blurredR = cv2.medianBlur(rMask, 5)
    blurredG = cv2.medianBlur(gMask, 5)
    grayImage = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
    blurredImg = cv2.GaussianBlur(grayImage, (3, 3), 0)
    # edge detection
    lower = 30
    upper = 90
    blackimg = cv2.inRange(blurredImg, 0, grayThresh)
    # edgesImg = cv2.Canny(blackimg, lower, upper, 3)
    # combine images
    # return [edgesImg, blurredG, blurredR, blackimg]
    return {"green": blurredG, "red": blurredR, "black": blackimg}


  def filter_OB(self, hsv: np.ndarray):
    """
    Extracts the orange and blue colors from the image -> this is used to detect the turn markers
    """
    orangeMin = tuple(self.configloader.get_property("filters")['ORANGELO'])
    orangeMax = tuple(self.configloader.get_property("filters")['ORANGEHI'])
    blueMin = tuple(self.configloader.get_property("filters")['BLUELO'])
    blueMax = tuple(self.configloader.get_property("filters")['BLUEHI'])
    # orange filter
    oMask = cv2.inRange(hsv, orangeMin, orangeMax)
    # blue filter
    bMask = cv2.inRange(hsv, blueMin, blueMax)
    # blur images to remove noise
    blurredO = cv2.medianBlur(oMask, 5)
    blurredB = cv2.medianBlur(bMask, 5)
    # return [blurredO, blurredB]
    return {"orange": blurredO, "blue": blurredB}

  def get_pillars(self, imgIn: np.ndarray, type = "RED") -> list[Pillar]:
    """
      Extracts pillars from filtered image
    """
    minSize = float(configloader.get_property("contours")['minSize'])
    edges = cv2.Canny(cv2.medianBlur(cv2.copyMakeBorder(imgIn[:], 2, 2, 2, 2, cv2.BORDER_CONSTANT, value=0), 3), 30, 200)

    contours, hierarchy = cv2.findContours(edges, 
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    processedContours = []
    for contour in contours:
      size = cv2.contourArea(contour)
      if size > minSize:
        moment = cv2.moments(contour)
        if moment["m00"] != 0:
          x = int(moment["m10"] / moment["m00"])
          y = int(moment["m01"] / moment["m00"])
          # if abs(y-centerheight) > 45:
          #     continue

          _, _, w, h = cv2.boundingRect(contour)
          width = math.ceil(w)
          height = math.ceil(h)

          if height * width <= 40.0:
            continue

          processedContours.append(Pillar(x, width, height, type))
    return processedContours