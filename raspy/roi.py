import asyncio
import base64
import json
import math
from time import sleep, time
import cv2
import numpy as np
from websockets import WebSocketServerProtocol, serve
from config import ConfigLoader
from helpers import Pillar, extract_ROI
from pipeline import Pipeline
from statemachine import StateMachine
from picamera2 import Picamera2

#TODO: round direction detection
#TODO: Add web viewer, config editor etc.
#TODO: Add serial communication
#TODO: Pillars handling
#TODO: PD tuning
#?: Webviewer controls for tuning colors, pd, and selecting stream

configloader = ConfigLoader("config.json")
pipeline = Pipeline(configloader)

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())
picam2.start()

picam2.set_controls({
    "AwbEnable": False,
    # controls.AWB_TEMPERATURE: fixed_temperature
})


def cycle():
  global sm, last_error, kp, kd

  # image reading, usually form camera
  img = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_RGB2BGR)
  
  undistorted = pipeline.undistort(img)
  color_image = pipeline.crop(undistorted)

  # copy for webviewer visualization
  viz = color_image.copy()

  # convert to hsv, for color filtering
  hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

  # center region-of-interest for detecting the turn marker lines
  roi_center_w, roi_center_h = 100, 30
  roi_center_x, roi_center_y = 320 - roi_center_w // 2, 180
  cv2.rectangle(viz, (roi_center_x, roi_center_y), (roi_center_x + roi_center_w, roi_center_y + roi_center_h), (0, 255, 0), 2)
  roi_center = extract_ROI(hsv_image, [roi_center_x, roi_center_y], [roi_center_x + roi_center_w, roi_center_y + roi_center_h])
  
  # filter out the orange and blue colors of turn markers
  orange_blue = pipeline.filter_OB(roi_center)
  rgbl = pipeline.filter_RG_Bl(hsv_image, color_image)

  # how much orange/blue is in the center region-of-interest?
  portion_orange = cv2.countNonZero(orange_blue["orange"]) / (orange_blue["orange"].shape[0] * orange_blue["orange"].shape[1])
  portion_blue = cv2.countNonZero(orange_blue["blue"]) / (orange_blue["blue"].shape[0] * orange_blue["blue"].shape[1])


  # l and r ROIs used for PD control, to keep the car in the middle of the track and away from walls
  roi_width = 100
  roi_left = extract_ROI(rgbl["black"], [0, 0], [roi_width, 480])
  roi_right = extract_ROI(rgbl["black"], [640-roi_width, 0], [640, 480])

  portion_black_l = cv2.countNonZero(roi_left) / (roi_left.shape[0] * roi_left.shape[1])
  portion_black_r = cv2.countNonZero(roi_right) / (roi_right.shape[0] * roi_right.shape[1])

  # print("err:", portion_black_l-portion_black_r,"left: ", portion_black_l, "right: ", portion_black_r)

  # filter out the red and green colors of the pillars and walls
  pillars_r = pipeline.get_pillars(rgbl["red"], "RED")
  pillars_g = pipeline.get_pillars(rgbl["green"], "GREEN")
  pillars = pillars_r + pillars_g

  # A state machine is used to model the car's behavior
  # This checks if the car should transition to a new state, and if so, transitions
  # states may be PD-CENTER, PD-RIGHT, PD-LEFT, TURNING-L, TURNING-R, etc.
  transition_res = sm.shouldTransitionState(portion_orange, portion_blue, pillars)
  if transition_res:
    print(f"Transitioning to {sm.current_state}")

  # PD control

  # This is the refrence value for the single side PD control, 
  # eg. how much black should be on the left side when the car follows the left outer wall
  REF_PORTION = 0.25

  # error value
  error = 0.0

  turn_correction = 0.8

  if sm.current_state == "PD-CENTER":
    error = portion_black_l - portion_black_r
    correction = error * kp + (error - last_error) * kd
  elif sm.current_state == "PD-RIGHT":
    error = 2 * (REF_PORTION - portion_black_r)
    correction = error * kp + (error - last_error) * kd
  elif sm.current_state == "PD-RIGHT":
    error = 2 * (portion_black_l - REF_PORTION)
    correction = error * kp + (error - last_error) * kd

  elif sm.current_state == "TURNING-L":
    correction = turn_correction
  elif sm.current_state == "TURNING-R":
    correction = -turn_correction

  #TODO: Implement Pillars

  
  else:
    correction = 0.0
  #TODO: Write to serial

  # viz stuff
  cv2.putText(viz, f"State: {sm.current_state} {round(time() - sm.last_state_time, 2)}s", (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
  cv2.putText(viz, f"Errs: {round(portion_black_l-0.25, 2)} {round(portion_black_l-portion_black_r, 2)} {round(0.25-portion_black_r, 2)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

  for p in pillars:
    cv2.line(viz, (p.screen_x, 0), (p.screen_x, 480), (0, 0, 255) if p.color == "RED" else (0, 255, 0), 2)

  last_error = error
  return {
      "viz": viz,
      "roi_left": roi_left,
      "roi_right": roi_right,
      "roi_center": roi_center,
      "red": rgbl["red"],
      "green": rgbl["green"],
      "black": rgbl["black"],
      "orange": orange_blue["orange"],
      "blue": orange_blue["blue"],
      "hsv_image": hsv_image,
      "color_image": color_image
  }


def main():
  global sm, last_error, kp, kd
  sm = StateMachine()
  last_error = 0.0

  kp = configloader.get_property("PD")['kp']
  kd = configloader.get_property("PD")['kd']

  while True:
    cycle()

def encode_image(image):
    retval, buffer = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    base64_str = base64.b64encode(buffer).decode('utf-8')
    return base64_str

async def img_stream(websocket: WebSocketServerProtocol, path):
  global sm, last_error, kp, kd
  sm = StateMachine()
  last_error = 0.0

  kp = configloader.get_property("PD")['kp']
  kd = configloader.get_property("PD")['kd']

  while True:
    products = cycle() 
    data = {
      "a": encode_image(products["viz"]),
      "b": encode_image(products["black"])
    }
    await websocket.send(json.dumps(data))
    

if __name__ == "__main__":
  start_server = serve(img_stream, "0.0.0.0", 8765)
  asyncio.get_event_loop().run_until_complete(start_server)
  asyncio.get_event_loop().run_forever()
  # main()