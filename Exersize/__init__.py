'''
Created on 12.12.2020

@author: Matt
'''

name = "Exersize"
desc = "Exersize patterns and updates"

def showInfo():
    print(name + " - " + desc)
    
class ExersizeSetting():
  def __init__(self, force_target=None, velocity_target=None, position_target=None, acceleration_target=None):
    self.force_target = force_target
    self.velocity_target = velocity_target
    self.position_target = position_target
    self.acceleration_target = acceleration_target
    
    
class Exersize():
  def inital_setting(self, position, velocity, force_N):
    return ExersizeSetting(10, 0.1, 30, 10)
  
  def update_setting(self, position, velocity, force_N):
    return ExersizeSetting(force_target=30)
    
