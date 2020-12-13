'''
Created on 30.12.2018

@author: ED
'''

name = "Exersize"
desc = "Exersize Machine Patterns"

def showInfo():
    print(name + " - " + desc)
    
class ExersizeSetting():
  def __init__(self, force_target=None, velocity_target=None, position_target=None, acceleration_target=None):
    self.force_target = force_target
    self.velocity_target = velocity_target
    self.position_target = position_target
    self.acceleration_target = acceleration_target
    
class Exersize():
  def inital_setting(self, position, velocity, force_N, flux_mA):
    return ExersizeSetting(10, 0.1, 0, 10)
  
  def update_setting(self, position, velocity, force_N, flux_mA):
    return ExersizeSetting()
    
