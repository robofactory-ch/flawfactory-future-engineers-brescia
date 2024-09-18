import json

class ConfigLoader:
  def __init__(self, file_path):
    self.file_path = file_path
    self.config = {}
    self.load_config()

  def load_config(self):
    with open(self.file_path, 'r') as file:
      self.config = json.load(file)

  def get_property(self, key):
    return self.config.get(key)

  def save_config(self):
    with open(self.file_path, 'w') as file:
      json.dump(self.config, file, indent=4)
