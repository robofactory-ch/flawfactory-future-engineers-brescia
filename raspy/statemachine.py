from fftime import time

from helpers import Pillar

class StateMachine:

  current_state = "STARTING"
  last_state_time = 0.0
  round_dir = 0
  turns_left = 12

  time_diff = 0.0

  search_for_dir = True

  _scheduled_state = None
  _time_last_avoid = -5

  _scheduled_state_block = False

  next_pillar = None

  first_line_found = None

  avoid_big = False

  def __init__(self, isPillarRound: bool = False) -> None:
    self.last_state_time = time()
    self.isPillarRound = isPillarRound

  def transitionState(self, new_state: str):
    self.current_state = new_state
    self.last_state_time = time()
  
  def scheduleStateTransition(self, new_state: str, time_diff: float, block=True):
    self._scheduled_state = (new_state, time() + time_diff)
    self._scheduled_state_block = block

  def shouldTransitionState(self, portion_orange: float, portion_blue: float, pillars: list[Pillar]):
    time_diff = time() - self.last_state_time
    self.time_diff = time_diff

    if self._scheduled_state is not None:
      new_state, scheduled_time = self._scheduled_state
      if time() > scheduled_time:
        print("Scheduled state transition to", new_state)
        self.transitionState(new_state)
        self._scheduled_state = None
        return True
      if self._scheduled_state_block:
        return False
    
    if self.current_state == "STARTING":
      if abs(self.round_dir) > 10:
        self.round_dir = 1 if self.round_dir > 0 else -1
        self.search_for_dir = False
        self.transitionState("PD-CENTER")
        return True
      else:
        self.search_for_dir = True
        return False


    if self.current_state == "PD-CENTER" and self.turns_left <= 0 and self._scheduled_state is None:
      self.scheduleStateTransition("DONE", 3.1, False)
      return True
    
    # Hold the current state for a minimum of x seconds
    HOLD_STATES = [["TURNING-L", "TURNING-R", "PD-CENTER"]]
    if self.current_state in HOLD_STATES and time_diff < 0.4:
      return False
    
    if len(pillars) > 0 and self.isPillarRound:
      # Pillars are already sorted by distance, as their projected size is proportional to their distance
      next_pillar = pillars[0]
      # print("Next pillar:", next_pillar.color, "height:", next_pillar.height)
      if self.current_state == "PD-CENTER":
        self.next_pillar = next_pillar
        if next_pillar.height*next_pillar.width > 190 and (not next_pillar.ignore):
          self.transitionState("TRACKING-PILLAR")
          return True
      elif self.current_state == "TRACKING-PILLAR" or self.current_state == "PD-CENTER":
        if next_pillar.height*next_pillar.width > 530 and (time() - self._time_last_avoid) > 1.0:
          # self._time_last_avoid = time()
          self.transitionState(f"AVOIDING-{'R' if next_pillar.color == 'RED' else 'G'}")
          self.next_pillar = None
          return True

    if self.current_state == "TRACKING-PILLAR":
      # HOW?? Pillars has been lost, tracking is no more
      if len(pillars) == 0:
        print("Lost the pillar, tracking aborted")
        self.transitionState("PD-CENTER")
        return True

    if self.current_state == "AVOIDING-R" or self.current_state == "AVOIDING-G":
      if time_diff > 2.5: # Avoid for 0.6 seconds, huck and pray
        self.transitionState("PD-CENTER")
        self._time_last_avoid = time()
        self.avoid_big = False
        return True
    

    # We always finnish a turn after this timeout, no matter what. 
    # PD shall deal with straightening out
    TURN_TIMEOUT = 0.85
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
      if self.current_state != "TURNING-R" and self.round_dir < 0:
        self.turns_left -= 1
        if self.isPillarRound:
          # to = 0.45
          # if self.next_pillar:
          #   is_outer = (self.next_pillar.color == "RED" and self.round_dir == -1) or (self.next_pillar.color == "GREEN" and self.round_dir == 1)
          #   self.scheduleStateTransition("TURNING-L", 0.7 if is_outer else 0.2)
          # else:
            self.scheduleStateTransition("TURNING-L", 0.3)
        else:
          self.scheduleStateTransition("TURNING-L", 0.7 if not self.isPillarRound else 0.45)
      else:
        self.transitionState("PD-CENTER")
      return True
    
    if portion_orange > MIN_PORTION:
      if self.current_state != "TURNING-L" and self.round_dir > 0:
        self.turns_left -= 1
        if self.isPillarRound:
          # to = 0.45
          # if self.next_pillar:
          #   is_outer = (self.next_pillar.color == "RED" and self.round_dir == -1) or (self.next_pillar.color == "GREEN" and self.round_dir == 1)
          #   self.scheduleStateTransition("TURNING-R", 0.7 if is_outer else 0.2)
          # else:
            self.scheduleStateTransition("TURNING-R", 0.3)
        else:
          self.scheduleStateTransition("TURNING-R", 0.7 if not self.isPillarRound else 0.45)
      else:
        self.transitionState("PD-CENTER")
      return True
    
      
    return False