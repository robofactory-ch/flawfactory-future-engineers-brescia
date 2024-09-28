from time import time as _time

def time():
  SPEED = 102.0
  return _time() * (SPEED / 80.0)