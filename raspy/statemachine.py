import time

from helpers import Pillar


class StateMachine:

  current_state = "STARTING"
  last_state_time = 0.0
  round_dir = 0
  turns_left = 12

  def __init__(self, isPillarRound: bool = False) -> None:
    self.last_state_time = time()
    self.isPillarRound = isPillarRound

  def transitionState(self, new_state: str):
    self.current_state = new_state
    self.last_state_time = time()

  def shouldTransitionState(self, portion_orange: float, portion_blue: float, pillars: list[Pillar]):
    time_diff = time() - self.last_state_time
    
    if self.current_state == "STARTING":
      if self.round_dir != 0:
        self.transitionState("PD-CENTER")
        return True
      else:
        #TODO: Implement round direction detection
        round_dir = -1
        return False


    # Hold the current state for a minimum of x seconds
    if time_diff < 0.15:
      return False
    

    # We always finnish a turn after this timeout, no matter what. 
    # PD shall deal with straightening out
    TURN_TIMEOUT = 0.5
    if self.current_state in ["TURNING-L", "TURNING-R"]:
      if time_diff > TURN_TIMEOUT:
        self.transitionState("PD-CENTER")
        return True
      return False
    
    # If we run up on a turn marker, we should turn in the corresponding direction.
    # If we are already turning, and we see the other color, 
    # we should stop turning and straighten out with PD
    MIN_PORTION = 0.25
    if portion_blue > MIN_PORTION:
      if self.current_state != "TURNING-R":
        self.transitionState("TURNING-L")
      else:
        self.transitionState("PD-CENTER")
      return True
    
    if portion_orange > MIN_PORTION:
      if self.current_state != "TURNING-L":
        self.transitionState("TURNING-R")
      else:
        self.transitionState("PD-CENTER")
      return True
    
    if len(pillars) > 0 and self.isPillarRound:
      next_pillar = pillars[0]
      if self.current_state == "PD-CENTER":
        if next_pillar.color == "RED":
          self.transitionState("PD-RIGHT")
        else:
          self.transitionState("PD-LEFT")
        return True
      
    return False