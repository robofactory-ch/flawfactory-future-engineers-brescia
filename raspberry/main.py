import asyncio
import base64
import cv2
from base64 import b64decode
import numpy as np
from websockets import WebSocketServerProtocol, serve
import json
import math
import time
from config import *
from picamera2 import Picamera2
from libcamera import controls

picam2 = Picamera2()
picam2.start()

picam2.set_controls({
    "AwbEnable": False,
    # controls.AWB_TEMPERATURE: fixed_temperature
})

last_message_time = time.time()

cx_angles = np.arange(-319.5, 320.5, 1, dtype="d")
cx_angles = np.arctan2(cx_angles, 270)
sines = np.sin(cx_angles)
cosines = np.cos(cx_angles)

def get_cam_azi(i):
   return float(cx_angles[max(0, min(i, 639))])

class Wall:

   def __init__(self, type, dir, dist):
      self.type = type
      # self.observed_x = x
      # self.observed_y = y
      self.heading = dir
      self.distance = dist

class Contour:
   def __init__(self, type, x, width = 0.0, height = 0.0, distance = 0.0) -> None:
      self.type = type
      self.x = x
      self.width = width
      self.height = height

      self.heading = get_cam_azi(x)
      self.distance = distance

def crop(image: np.ndarray):
   return image[crop_hight:,:]

def color_filter(color_image: np.ndarray):

  color_image = crop(color_image)

  hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
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
  edgesImg = cv2.Canny(blackimg, lower, upper, 3)
  # combine images
  return [edgesImg, blurredG, blurredR, blackimg]

def get_contours(imgIn: np.ndarray, type = "RED"):
  edges = cv2.Canny(cv2.medianBlur(cv2.copyMakeBorder(imgIn[:], 2, 2, 2, 2, cv2.BORDER_CONSTANT, value=0), 3), 30, 200)

  contours, hierarchy = cv2.findContours(edges, 
      cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
  processedContours = []
  for contour in contours:
    size = cv2.contourArea(contour)
    if size > minContourSize:
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

        processedContours.append(Contour(type, x, width, height, estimate_depth(height)))
  return processedContours

def estimate_depth(h):
  """
    Intercept theorem / simmilar tri
  
  """
  calibration_factor = 300

  # if (h <= 5): return 3200.00

  h += 5

  # print("Calibration 700mm=", 700 / (1/h)) # calibration
  # print("Calibration 1500mm=", 1500 / (1/h)) # calibration

  calibration_factor = 14000.0
  # calibration_factor = 24000.0
  d = (1 / h) * calibration_factor

  # print(d)

  return np.minimum(d, 3200.0);


def drive(contours: list[Contour], walls: list[Wall]) -> tuple[float, float]:
  round_direction = True
  inputs = {
     "center": 0.0,
     "left": 0.0,
     "right": 0.0,
     "green": 0.0,
     "red": 0.0,
  }

  def contSort(c: Contour):
    return c.distance
    
  contours.sort(key=contSort)
  # wall avoidance
  for wall in walls:
    # center wall
    if wall.heading < 0.35 or wall.heading > 2*np.pi-0.35:
      if wall.distance <= 600:
         inputs['center'] = 1 if True else -1
    elif wall.heading > np.pi:
      if wall.distance <= 110:
        inputs['right'] = 2
      if wall.distance <= 200:
        inputs['left'] = 0.8
    elif wall.heading > 0.15:
      if wall.distance <= 110:
        inputs['left'] = -2
      if wall.distance <= 200:
        inputs['left'] = -0.8


  if len(contours) >= 1:
    c = contours[0]
    multi = 1
    key = "GREEN"
    if c.type == "RED":
      multi = -1
      key = "RED"

    close = False
    veryclose = False

    if c.distance <= 380:
      close = True
    if c.distance <= 150:
      veryclose = True
    
    a = 320
    clear = False
    if (c.x - (320+a)) >= 0 and c.type == "RED": # right side
      clear = True
    if (c.x - (320-a)) <= 0 and c.type == "GREEN": # left side
      clear = True
    clear = False
       
    if close and not clear:
      inputs[key] = 0.85 * multi
    if veryclose and not clear:
      inputs[key] = 2.25 * multi
         
  # print("steering inputs:", inputs)
  agg = max(-1.0, min(1.0, float(sum(inputs.values()))))
  print("aggregate:", agg)
  return (agg, 0.0)

def encode_image(image):
    retval, buffer = cv2.imencode('.jpg', image)
    base64_str = base64.b64encode(buffer).decode('utf-8')
    return base64_str


def process_walls(edges_img: np.ndarray, contours: list[Contour]):
  """

  based on the lowest edge in the image, a synthetic image of all wall bottoms is produced.
  On that image, we search for lines, wich we then locate in local space around the robot

  """

  walls = []

  wall_heights = np.argmax(edges_img, axis=0)

  for c in contours:
    half_width = (c.width + 4) // 2

    left_edge = max(c.x - half_width, 0)
    right_edge = min(c.x + half_width, 640)
    span = right_edge - left_edge


    wall_heights[left_edge: right_edge] = np.zeros(span)

  syntheticWallBottoms = np.zeros((240, 640), dtype="uint8")
  
  indices = np.dstack((wall_heights, np.arange(640)))
  syntheticWallBottoms[tuple(np.transpose(indices))] = 255

  # cv2.imshow("synthetic wall bottm edges", syntheticWallBottoms)

  lines = cv2.HoughLinesP(
                syntheticWallBottoms,
                1,
                np.pi/180,
                threshold=50,
                minLineLength=10,
                maxLineGap=20
                )
  
  lines = list(lines) if lines is not None else []
     
  def lineSort(line):
    return line[0][0]
  lines.sort(key=lineSort)
  newLines = []
  lastLine = [None]
  for line in lines:
    x1, y1, x2, y2 = line[0]
    if y1 == 0 or y2 == 0:
      continue
    if lastLine[0] != None:
      lastSlope = (lastLine[3] - lastLine[1]) / (lastLine[2] - lastLine[0])
      slope = (y2 - y1) / (x2 - x1)
      newY = lastLine[3] + (x1 - lastLine[2]) * lastSlope
      if abs(x1 - lastLine[2]) < 100 and abs(y1 - newY) < 5 and abs(math.atan2(slope, 1) - math.atan2(lastSlope, 1)) < math.pi / 50:
        newLines[len(newLines) - 1] = [newLines[len(newLines) - 1][0], newLines[len(newLines) - 1][1], x2, y2]
        lastLine = line[0]
        continue
    lastLine = line[0]
    newLines.append([x1, y1, x2, y2])

  viz2 = np.zeros((640, 640), dtype=np.uint8)
  viz3 = np.zeros((640, 640, 3), dtype=np.uint8)

  for line in newLines:
    x1, y1, x2, y2 = line
    viz2 = cv2.line(viz2, (x1, y1), (x2, y2), 255)
    # print("------")

    d1 = estimate_depth(y1)
    d2 = estimate_depth(y2)

    u1 = (np.tan(get_cam_azi(x1))) * d1
    v1 = d1

    # print(u1, v1)
    u2 = (np.tan(get_cam_azi(x2))) * d2
    v2 = d2

    A = (u2 - u1)
    B = (v1 - v2)
    C = (u1 * v2 - u2 * v1)

    D = np.sqrt(A**2 + B**2)

    wall_dist = -1.0
    if D != 0.0:
        wall_dist = abs(C / D)
    
    wall_rotation = -np.arctan2(B, A) % (np.pi*2)
    # print("wall_rotation:", wall_rotation, "(rad)")

    y = int(np.sin(wall_rotation) * wall_dist / 3200.0 * 640) + 320
    x = int(np.cos(wall_rotation) * wall_dist / 3200.0 * 640) + 320
    viz3 = cv2.rectangle(viz3, (x, y), (x+5, y+5), (255, 255, 255))

    walls += [Wall("WALL_UNKNOWN", wall_rotation, wall_dist)]
  
  for c in contours:
    y = int(np.sin(c.heading) * c.distance / 3200.0 * 640) + 320
    x = int(np.cos(c.heading) * c.distance / 3200.0 * 640) + 320
    viz3 = cv2.rectangle(viz3, (x, y), (x+5, y+5), (0, 255 if c.type == "GREEN" else 0, 255 if c.type == "RED" else 0))

  # cv2.imshow("walls found", viz2)

  viz3 = cv2.rectangle(viz3, (320, 320), (325, 325), (255, 0, 0))
  # cv2.imshow("walls found, bird", viz3)

  return walls, syntheticWallBottoms, viz3



async def image_stream(websocket: WebSocketServerProtocol, path):
# def main():

  global last_message_time

  while True:
    current_time = time.time()

    delta_time = current_time - last_message_time
    last_message_time = current_time



    # message = json.loads(message)
    # nparr = np.frombuffer(b64decode(message['image']), np.uint8)
    # img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    img = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_BGR2RGB)
    img = cv2.undistort(img, mtx, dist, None, mtx)

    # encoder = float(message['encoder']) * tire_dia
    # speed = float(message['speed']) * 2 * np.pi * tire_dia
    # steering_angle = float(message['steering_angle']) / 180 * np.pi

    # print(steering_angle)

    # cv2.imshow("Raw Stream", img)

    [edgesImg, blurredG, blurredR, blackimg] = color_filter(img)

    # cv2.imshow("Black", blackimg)
    # cv2.imshow("Edges", edgesImg)
    # cv2.imshow("Red", blurredR)
    # cv2.imshow("Green", blurredG)

    contoursG = get_contours(blurredG, "GREEN")
    contoursR = get_contours(blurredR, "RED")
    contours = [] + contoursG + contoursR

    walls, synths, viz3 = process_walls(edgesImg, contours)

    steering, accel = drive(contours, walls)

    await websocket.send(json.dumps({
       'steer': steering,
       'acceleration': 1.0
    }))

    viz = (np.dstack((np.zeros(blurredG.shape), blurredG, blurredR)) * 255.999) .astype(np.uint8)
    for c in contoursR:
        viz = cv2.line(viz, (c.x, 0), (c.x, 479), (0, 0, 255), 1)
    for c in contoursG:
        viz = cv2.line(viz, (c.x, 0), (c.x, 479), (0, 255, 0), 1)

    # viz = cv2.addWeighted(viz, 1, limg, 1, 0, dtype=0)


    data = {
                "a": encode_image(viz),
                "b": encode_image(viz3)
            }
    await websocket.send(json.dumps(data))


start_server = serve(image_stream, "0.0.0.0", 8765)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()