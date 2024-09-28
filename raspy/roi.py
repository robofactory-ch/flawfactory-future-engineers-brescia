import asyncio
import base64
import json
import math
import os
from time import sleep
import cv2
import numpy as np
import serial
import serial.tools.list_ports
from websockets import WebSocketServerProtocol, serve
from config import ConfigLoader
from helpers import Pillar, extract_ROI
from pipeline import Pipeline
from statemachine import StateMachine
from picamera2 import Picamera2
from rounddir import find_round_dir
import argparse

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



ports = serial.tools.list_ports.comports()


try:
  ser = serial.Serial(configloader.get_property("ArduinoSerialPort"), 9600)
except:
  print("Arduino not connected, available devices")
  ser = None
  for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))

def cycle():
  global sm, last_error, kp, kd

  # image reading, usually form camera
  img = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_RGB2BGR)
  
  # undistorted = pipeline.undistort(img)
  color_image = pipeline.crop(img)

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
  roi_left = extract_ROI(rgbl["black"], [0, 0], [roi_width, 150])
  roi_right = extract_ROI(rgbl["black"], [640-roi_width, 0], [640, 150])

  portion_black_l = cv2.countNonZero(roi_left) / (roi_left.shape[0] * roi_left.shape[1])
  portion_black_r = cv2.countNonZero(roi_right) / (roi_right.shape[0] * roi_right.shape[1])

  # print("err:", portion_black_l-portion_black_r,"left: ", portion_black_l, "right: ", portion_black_r)

  # filter out the red and green colors of the pillars and walls
  pillars_r = pipeline.get_pillars(rgbl["red"], "RED")
  pillars_g = pipeline.get_pillars(rgbl["green"], "GREEN")
  pillars = pillars_r + pillars_g

  pillars.sort(key=lambda x: x.width*x.height, reverse=True)

  # check if the next pillar is before or after a turn marker
  if len(pillars) > 0:
    next_pillar = pillars[0]
    next_pillar.width = min(next_pillar.width, 5)
    # for this extract a region-of-interest around the pillar
    roi_pillar_check = extract_ROI(hsv_image, [next_pillar.screen_x - int(next_pillar.width*0.35), 0], [next_pillar.screen_x + int(next_pillar.width*0.35), hsv_image.shape[1]])
    # filter out the orange and blue colors
    if not roi_pillar_check.shape[0] == 0 and not roi_pillar_check.shape[1] == 0:
      orange_blue_pillar = pipeline.filter_OB(roi_pillar_check)
      portion_orange_pillar = cv2.countNonZero(orange_blue_pillar["orange"]) / (orange_blue_pillar["orange"].shape[0] * orange_blue_pillar["orange"].shape[1])
      portion_blue_pillar = cv2.countNonZero(orange_blue_pillar["blue"]) / (orange_blue_pillar["blue"].shape[0] * orange_blue_pillar["blue"].shape[1])
      # check if the pillar is before or after a turn marker
      # print("pillar check", portion_orange_pillar, portion_blue_pillar)
      if portion_orange_pillar > 0.00005 or portion_blue_pillar > 0.00005:
        # print("Pillar is after a turn marker")
        pillars[0].ignore = True
      try:
        roi_straight_check = extract_ROI(rgbl["red"] if next_pillar.color == "RED" else rgbl["green"], [next_pillar.screen_x - int(next_pillar.width*2), 0], [next_pillar.screen_x + int(next_pillar.width*2), hsv_image.shape[1]])
        lower = 30
        upper = 90
        edges_img = cv2.Canny(roi_straight_check, lower, upper, 3)
        heights = np.argmax(edges_img, axis=0)
        print(heights)
        print("pillar diff", np.max(heights) - np.min(heights))
        if np.max(heights) - np.min(heights) > 3:
          print("big correction")
          pillars[0].big_correction = True
      except:
        pass
  # A state machine is used to model the car's behavior
  # This checks if the car should transition to a new state, and if so, transitions
  # states may be PD-CENTER, PD-RIGHT, PD-LEFT, TURNING-L, TURNING-R, etc.
  transition_res = sm.shouldTransitionState(portion_orange, portion_blue, pillars)
  if transition_res:
    print(f"Transitioning to {sm.current_state}")

  # PD control

  # This is the reference value for the single side PD control, 
  # eg. how much black should be on the left side when the car follows the left outer wall

  # pillar_ref = 0.35
  # if sm.next_pillar:
  #   if sm.next_pillar.ignore:
  #     pillar_ref = 0.48
    

  REF_PORTION = 0.45 if not sm.isPillarRound else 0.45

  # error value
  error = 0.0

  turn_correction = 0.75 if not sm.isPillarRound else 1.0

  PD_STATES = ["PD-CENTER", "PD-RIGHT", "PD-LEFT"]

  if sm.current_state == "TRACKING-PILLAR" and len(pillars) > 0:
    # attempt to keep the pillar in the center of the image
    error = float(pillars[0].screen_x - 320) / 640.0

  # follow the left wall, if we're going counter-clockwise
  if sm.current_state in PD_STATES and sm.round_dir == -1:
    error = REF_PORTION - portion_black_r

  # follow the right wall, if we're going clockwise
  if sm.current_state in PD_STATES and sm.round_dir == 1:
    error = portion_black_l - REF_PORTION


  correction = error * kp + (error - last_error) * kd

  
  if sm.current_state == "TURNING-L":
    correction = -turn_correction
  if sm.current_state == "TURNING-R":
    correction = turn_correction
  
  FIRSTPAHSETIME = 0.8
  if len(pillars) > 0:
    if pillars[0].big_correction:
      FIRSTPAHSETIME = 1.2

  if (sm.current_state == "AVOIDING-R" and sm.time_diff < FIRSTPAHSETIME):
    error = -0.75
  if (sm.current_state == "AVOIDING-G" and sm.time_diff > FIRSTPAHSETIME):
    error = -0.75*0.6
  if (sm.current_state == "AVOIDING-G" and sm.time_diff < FIRSTPAHSETIME):
    error = 0.75
  if (sm.current_state == "AVOIDING-R" and sm.time_diff > FIRSTPAHSETIME):
    error = 0.75*0.6

  if sm.current_state == "DONE":
    correction = 0.0
    print("---- DONE ----")
    if ser:
      message = "d" + str(int(0)) + "\n"
      ser.write(message.encode())
      message = "s0\n"
      ser.write(message.encode())
    # exit()
    sleep(5)
    exit()

  if sm.search_for_dir and sm.current_state == "STARTING":
    sm.round_dir += find_round_dir(black_img=rgbl["black"])

  
  # else:
  #   correction = 0.0
  

  correction = max(-1.0, min(1.0, correction))
  MAX_STEERING_ANGLE = -55.0
  steering_angle = correction * MAX_STEERING_ANGLE


  if ser:
    message = "d" + str(int(100)) + "\n"
    ser.write(message.encode())
    message = "s " + str(int(steering_angle)) + "\n"
    ser.write(message.encode())
  
  # print(error - last_error)

  # viz stuff
  cv2.putText(viz, f"State: {sm.current_state} {round(sm.time_diff, 2)}s", (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
  cv2.putText(viz, f"Errs: {round(portion_black_l-0.25, 2)} {round(portion_black_l-portion_black_r, 2)} {round(0.25-portion_black_r, 2)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
  cv2.putText(viz, f"Correction: {round(correction, 2)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
  cv2.putText(viz, f"{12 - sm.turns_left} / 12", (580, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
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
      "color_image": color_image,
      # "lines": pipeline.filter_OB(hsv_image)['orange'],
      # "parking": pipeline.filter_parking(hsv_image)
  }


def main():
  global sm, last_error, kp, kd
  sm = StateMachine(isPillarRound=pillars)
  last_error = 0.0

  kp = configloader.get_property("PD")['kp']
  kd = configloader.get_property("PD")['kd']

  try:
    while True:
      cycle()
  except (KeyboardInterrupt):
    if ser:
      ser.write("s0\n".encode())
      ser.write("d0\n".encode())
    exit()

def encode_image(image):
    retval, buffer = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 99])
    base64_str = base64.b64encode(buffer).decode('utf-8')
    return base64_str

async def img_stream(websocket: WebSocketServerProtocol, path):
  global sm, last_error, kp, kd
  sm = StateMachine(isPillarRound=pillars)
  last_error = 0.0

  kp = configloader.get_property("PD")['kp']
  kd = configloader.get_property("PD")['kd']

  has_sent_streams_info = False
  current_streams = ["viz", "black"]
  try:
    while True:
      products = cycle() 
      if not has_sent_streams_info:
        has_sent_streams_info = True
        await websocket.send(json.dumps({
          "streams": list(products.keys())
        }))
      
      # check if the websocket has sent a stream request, wait at most for 0.05 seconds
      try:
        res = json.loads(await asyncio.wait_for(websocket.recv(), timeout=0.01))
        current_streams[0] = res["streamA"]
        current_streams[1] = res["streamB"]
      except:
        pass

      data = {
        "a": encode_image(products[current_streams[0]]),
        "b": encode_image(products[current_streams[1]])
      }
      await websocket.send(json.dumps(data))
  except (KeyboardInterrupt):
    if ser:
      ser.write("s0\n".encode())
      ser.write("d0\n".encode())
    exit()
    

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Check if --headless flag was given.")
  parser.add_argument('--headless', action='store_true', help='Run in headless mode')
  parser.add_argument('--pillars', action='store_true', help='Run in pillar mode')
  parser.add_argument('--shutdown', action='store_true', help='Shutdown after run')
  args = parser.parse_args()
  headless = args.headless
  pillars = args.pillars
  shutdown = args.shutdown

  if ser:
    while True:
      msg = ser.readline().decode('utf-8')
      msg = msg.strip()
      if msg == "enable 1" :
        break

  if headless:
    main()
  else:
    start_server = serve(img_stream, "0.0.0.0", 8765)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
    if shutdown:
      os.system("sudo shutdown now")
