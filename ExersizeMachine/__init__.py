'''
Created on 12.12.2020

@author: Matt
'''

name = "ExersizeMachine"
desc = "Exersize Machine Configuration"

import math

def showInfo():
    print(name + " - " + desc)
    
#https://forum.esk8.news/t/wiki-what-motor-parameters-mean/30089
#https://www.motioncontroltips.com/faq-whats-the-relationship-between-current-and-dc-motor-output-torque/    
    
class ExersizeMachine():
  def __init__(self, gear_ratio=1.0, kV=100, Iidle=1.0, Ddrum=0.1, current_scale=1.0):
    self.gear_ratio = gear_ratio
    self.kV = kV
    self.Iidle = Iidle
    self.drum_diam = Ddrum
    
    self.current_scale = current_scale

#     self.motor_flux_linkage = 60 / (2 * math.pi * kV)
#     self.motor_torque_constant = self.motor_flux_linkage * 1.5 * pole_pairs
    
    self.motor_torque_constant = 60 / (2 * math.pi * kV)    
    self.drag_torque = self.motorCurrentToMotorTorque(self.Iidle)

  def lineForceToMotorTorque(self, force):
      drum_torque = force * self.drum_diam * 0.5
      motor_torque =  drum_torque * self.gear_ratio
      return motor_torque
    
  def motorTorqueToLineForce(self, motor_torque):
      drum_torque =  motor_torque * self.gear_ratio
      line_force = drum_torque / (self.drum_diam * 0.5)

  def motorCurrentToMotorTorque(self, current):
      return self.motor_torque_constant * current
    
  def motorTorqueToTorqueDemand(self, torque):
      return int(torque * self.current_scale)
  