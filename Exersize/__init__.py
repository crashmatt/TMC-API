'''
Created on 12.12.2020

@author: Matt
'''

name = "Exersize"
desc = "Exersize patterns and updates"

import math

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
    

class ElasticExersize(Exersize):
  def __init__(self, Kspring=100, preload=25, Fmax=100):
    self.Kspring = Kspring
    self.Fmax = Fmax
    self.preload = preload
    
  def update_setting(self, position, velocity, force_N):
    force = self.preload + ( math.fabs(position) * self.Kspring)
    if force > self.Fmax:
      force = self.Fmax
    return ExersizeSetting(force_target=force)



class MassExersize(Exersize):
  def __init__(self, mass=10.0, static_mass=1.0, t_dvfilt=0.1, Fmax=150):
    self.mass = mass
    self.static_mass = static_mass
    self.filtered_delta_velocity = 0
    self.t_dvfilt = t_dvfilt
    self.last_velocity = 0
    self.Fmax = Fmax
    
  def update_setting(self, position, velocity, force_N):
    delta_v = velocity - self.last_velocity
    ddv = self.filtered_delta_velocity - delta_v
    ddv_filt = ddv * self.t_dvfilt
    self.filtered_delta_velocity += ddv_filt
    
    accel = 9.8 + self.filtered_delta_velocity
    force = accel * mass
    
    if force > self.Fmax:
      force = self.Fmax

    self.last_velocity = velocity

    return ExersizeSetting(force_target=force, acceleration_target=9.8)
