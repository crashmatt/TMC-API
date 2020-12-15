#!/usr/bin/env python3
'''
Created on 26.02.2019

@author: ED
'''

import time
import numpy as np
import math
from PyTrinamic.connections.ConnectionManager import ConnectionManager
from PyTrinamic.evalboards.TMC4671_eval import TMC4671_eval
from PyTrinamic.ic.TMC4671.TMC4671 import TMC4671 as TMC4671_IC
from PyTrinamic.connections.uart_ic_interface import uart_ic_interface
from asyncio.tasks import sleep

from Exersize import Exersize
from ExersizeMachine import ExersizeMachine

def reject_outliers_2(data, m=2.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d / (mdev if mdev else 1.)
    return data[s < m]
  
  
  
def calibrate_adc_offset(TMC4671):
  print("start ADC calibration...")

  " Switch to stopped mode "
  TMC4671.writeRegister(TMC4671.registers.OPENLOOP_VELOCITY_TARGET, 0)
  TMC4671.writeRegister(TMC4671.registers.MODE_RAMP_MODE_MOTION, TMC4671.registers.MOTION_MODE_STOPPED)
  TMC4671.writeRegister(TMC4671.registers.PHI_E_SELECTION, TMC4671.registers.PHI_E_EXTERNAL)
  TMC4671.writeRegister(TMC4671.registers.PHI_E_EXT, 0)
  TMC4671.writeRegister(TMC4671.registers.UQ_UD_EXT, 0)
  
  " calibrate_adc_offsets "
  measurementTime = 1.0
  measurements = 0
  adc_expected_offset = 32767
  adc_offset_limit = 5000
  
  lAdcI0Raw = list()
  # lAdcI0Filt = list()
  # lAdcI0Offset = list()
  
  lAdcI1Raw = list()
  # lAdcI1Filt = list()
  # lAdcI1Offset = list()
  
  startTime = time.time()
  TMC4671.writeRegister(TMC4671.registers.ADC_RAW_ADDR, 0)
  
  while ((time.time() - startTime) <= measurementTime):
      measurements += 1
  
      " read adc values "    
      adc_i0 = TMC4671.readRegisterField(TMC4671.fields.ADC_I0_RAW)
      adc_i1 = TMC4671.readRegisterField(TMC4671.fields.ADC_I1_RAW)
      #print("ADC_I0_Value: %d" % adc_i0)
      #print("ADC_I1_Value: %d" % adc_i1)
      lAdcI0Raw.append(adc_i0)
      lAdcI1Raw.append(adc_i1)
    
  " filter offsets " 
  adcI0Mean = np.mean(lAdcI0Raw)
  adcI1Mean = np.mean(lAdcI1Raw)

  " update offsets "
  TMC4671.writeRegisterField(TMC4671.fields.ADC_I0_OFFSET, int(adcI0Mean))
  TMC4671.writeRegisterField(TMC4671.fields.ADC_I1_OFFSET, int(adcI1Mean))
    
  adc_i0_offset = int(TMC4671.readRegisterField(TMC4671.fields.ADC_I0_OFFSET))
  adc_i1_offset = int(TMC4671.readRegisterField(TMC4671.fields.ADC_I1_OFFSET))
  
  print("ADC_I0_Offset: %d" % adc_i0_offset)
  print("ADC_I1_Offset: %d" % adc_i1_offset)
  
  if math.fabs(adc_i0_offset - adc_expected_offset) > adc_offset_limit:
    print("ADC I0offset larger than expected")
    return False
  
  if math.fabs(adc_i1_offset - adc_expected_offset) > adc_offset_limit:
    print("ADC I0offset larger than expected")
    return False
  
  return True
    
    
def calibrate_abn_encoder_active(TMC4671, encoderResolution, polePairs, 
                                pre_pos_reverse=True, pre_pos_revs=0, pre_pos_timeout=5.0, flux_mA=1500, repeats=5, velocity=30, overshoot_time=0.2):
  " ===== 2) estimate the encoder offset ====="
  
  " Init encoder (mode 0) "
  " put a voltage on the motor and wait 1 second for alignment "
  TMC4671.writeRegister(TMC4671.registers.OPENLOOP_VELOCITY_TARGET, 0)
  TMC4671.writeRegister(TMC4671.registers.MODE_RAMP_MODE_MOTION, 0x00000008)
  TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000)
  TMC4671.writeRegister(TMC4671.registers.PHI_E_SELECTION, TMC4671.registers.PHI_E_EXTERNAL)
  TMC4671.writeRegister(TMC4671.registers.PHI_E_EXT, 0x00000000)
  TMC4671.writeRegister(TMC4671.registers.UQ_UD_EXT, flux_mA)
  time.sleep(1)  
  
  
  " clear abn_decoder_count "
  TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_COUNT, 0x00000000)
  TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_COUNT_N, 0x00000000)
  
  print("abn_decoder_count:" + str(TMC4671.readRegister(TMC4671.registers.ABN_DECODER_COUNT)))
  
  " Switch to open loop velocity mode "
  TMC4671.writeRegister(TMC4671.registers.PHI_E_SELECTION, TMC4671.registers.PHI_E_OPEN_LOOP)
  
  openloop_velocity = velocity

  timeout = time.time() + pre_pos_timeout
  
  if pre_pos_revs != 0:      
    if pre_pos_reverse:
      set_velocity = -openloop_velocity
    else:
      set_velocity = openloop_velocity
      
    print("Setting pre position velocity to:" + str(set_velocity) )
    TMC4671.writeRegister(TMC4671.registers.OPENLOOP_VELOCITY_TARGET, set_velocity)
      
    rev_count = 0
    while rev_count < pre_pos_revs:
      TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_COUNT_N, 0x00000000)

      if time.time() > timeout:
        print("Active ABN calibrate initial position timeout")       
        TMC4671.writeRegister(TMC4671.registers.OPENLOOP_VELOCITY_TARGET, 0)
        TMC4671.writeRegister(TMC4671.registers.UQ_UD_EXT, 0)
        return
      
      decoderCountN = TMC4671.readRegister(TMC4671.registers.ABN_DECODER_COUNT_N)
      if decoderCountN != 0:
        rev_count += 1
        TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_COUNT_N, 0x00000000)
        

    print("Pre-position decoder N count:", decoderCountN)
    TMC4671.writeRegister(TMC4671.registers.OPENLOOP_VELOCITY_TARGET, 0)
    time.sleep(0.5)
  
  raw_encoder_offsets = []
  encoder_offsets = []
  direction = False
  
  for iter in range(1, repeats):
    TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_COUNT_N, 0x00000000)
    
    if direction:
      set_velocity =  -openloop_velocity
    else:
      set_velocity =  openloop_velocity
    direction = not direction
    
    print("Setting N edge detect velocity to:" + str(set_velocity) )
    TMC4671.writeRegister(TMC4671.registers.OPENLOOP_VELOCITY_TARGET, set_velocity)
  
    startTime = time.time()
    loopCnt = 0
    while True:
      decoderCountN = TMC4671.readRegister(TMC4671.registers.ABN_DECODER_COUNT_N)      
      decoderCount = TMC4671.readRegister(TMC4671.registers.ABN_DECODER_COUNT)      
      
#       if loopCnt%100 == 0:
#         print("dec: " + str(decoderCount) + " dec_n: " + str(decoderCountN) )
#       loopCnt += 1
  
      " timeout after 3 seconds "
      if (time.time()-startTime) > 3:
          return False;
        
      if decoderCountN != 0:
        break;
        
    raw_encoder_offsets.append(decoderCountN)
    
    decoderCountN_offset = decoderCountN % (encoderResolution / polePairs)
    
    print("abn_decoder_count:" + str(decoderCount))
    print("abn_decoder_count_n:" + str(decoderCountN))
    print("=> estimated encoder offset: " + str(decoderCountN_offset))
    
    encoder_offsets.append(decoderCountN_offset)
    
    time.sleep(overshoot_time)  
    
  print("halt motor")
  TMC4671.writeRegister(TMC4671.registers.OPENLOOP_VELOCITY_TARGET, 0)
    
  " ===== 3) use the estimated offset ====="
  
  print("=> encoder offsets: ", encoder_offsets)
    
  average_offset = np.average(encoder_offsets)
  print("=> average encoder offset: " + str(average_offset))
  
  trimmed_encoder_offsets = reject_outliers_2(np.array(encoder_offsets))

  print("=> trimmed encoder offsets: ", trimmed_encoder_offsets)

  trimmed_average_offset = np.average(trimmed_encoder_offsets)
  print("=> average trimmed encoder offset: " + str(trimmed_average_offset))
  
  " write offset "
  TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_PHI_E_PHI_M_OFFSET, int(trimmed_average_offset))
  
  print("raw_encoder_offsets:", raw_encoder_offsets)
  
  TMC4671.writeRegister(TMC4671.registers.UQ_UD_EXT, 0)
  
  return True
  
  
def calibrate_abn_encoder_passive(TMC4671, encoderResolution, polePairs, measurements=3, flux_mA=1500):
  " ===== 2) estimate the encoder offset ====="
  
  " Init encoder (mode 0) "
  " put a voltage on the motor and wait for alignment "
  TMC4671.writeRegister(TMC4671.registers.OPENLOOP_VELOCITY_TARGET, 0)
  TMC4671.writeRegister(TMC4671.registers.MODE_RAMP_MODE_MOTION, 0x00000008)
  TMC4671.writeRegister(TMC4671.registers.PHI_E_SELECTION, TMC4671.registers.PHI_E_EXTERNAL)
  TMC4671.writeRegister(TMC4671.registers.PHI_E_EXT, 0x00000000)
  TMC4671.writeRegister(TMC4671.registers.UQ_UD_EXT, flux_mA)
  time.sleep(1)
  
  " clear abn_decoder_count "
  TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_COUNT, 0x00000000)
  TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_COUNT_N, 0x00000000)
  
  print("abn_decoder_count:" + str(TMC4671.readRegister(TMC4671.registers.ABN_DECODER_COUNT)))
  
  " Switch to stopped mode "  
  TMC4671.writeRegister(TMC4671.registers.UQ_UD_EXT, 0)
  TMC4671.writeRegister(TMC4671.registers.MODE_RAMP_MODE_MOTION, TMC4671.registers.MOTION_MODE_STOPPED)
  
  openloop_velocity = 30
  
  raw_encoder_offsets = []
  encoder_offsets = []
  direction = False
  
  last_encoder_Ncount = 0
  decoderCountN = 0
  read_count = 0

  while(read_count < measurements):
    decoderCountN = TMC4671.readRegister(TMC4671.registers.ABN_DECODER_COUNT_N)      

    #Filter faled value  
    if int(decoderCountN) != 454793344:
      if decoderCountN != last_encoder_Ncount:
        read_count += 1
        last_encoder_Ncount = decoderCountN  
        raw_encoder_offsets.append(decoderCountN)
        
        decoderCountN_offset = decoderCountN % (encoderResolution / polePairs)
        encoder_offsets.append(decoderCountN_offset)
        
  #      print("abn_decoder_count:" + str(decoderCount))
        print("abn_decoder_count_n:" + str(decoderCountN))
        print("=> estimated encoder offset: " + str(decoderCountN_offset))
            
  " ===== 3) use the estimated offset ====="
  
  print("=> encoder offsets: ", encoder_offsets)
    
  average_offset = np.average(encoder_offsets)
  print("=> average encoder offset: " + str(average_offset))
  
  trimmed_encoder_offsets = reject_outliers_2(np.array(encoder_offsets))

  print("=> trimmed encoder offsets: ", trimmed_encoder_offsets)

  trimmed_average_offset = np.average(trimmed_encoder_offsets)
  print("=> average trimmed encoder offset: " + str(trimmed_average_offset))
  
  " write offset "
  TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_PHI_E_PHI_M_OFFSET, int(trimmed_average_offset))
  
  print("raw_encoder_offsets:", raw_encoder_offsets)
  
  return True
    
  
def main():
  connectionManager = ConnectionManager()
  myInterface = connectionManager.connect()
  
  if isinstance(myInterface, uart_ic_interface):
      # Create an TMC4671 IC class which communicates directly over UART
      TMC4671 = TMC4671_IC(myInterface)
  else:
      # Create an TMC4671-Eval class which communicates over the Landungsbruecke via TMCL
      TMC4671 = TMC4671_eval(myInterface)
  
  " read ChipInfo "
  
  TMC4671.showChipInfo
  
  " ===== 1) base configuration ====="
  
  DRIVER_TEETH = 24
  IDLER_TEETH = 50
  DRUM_TEETH = 60
  
  polePairs = 7
  encoderResolution = 4096
  drum_diam = 0.04
  GEAR_RATIO = (DRIVER_TEETH * DRIVER_TEETH) / (IDLER_TEETH * DRUM_TEETH)

  print("GEAR_RATIO:", GEAR_RATIO)
  
  current_scale = 10000 * 230 / 255
  exersizeMachine = ExersizeMachine(gear_ratio=GEAR_RATIO, kV=130, Iidle=1.1, Ddrum=0.04, current_scale=current_scale)

      
  # 
  # " Motor type &  PWM configuration "
  # TMC4671.writeRegister(TMC4671.registers.MOTOR_TYPE_N_POLE_PAIRS, 0x00030000 | polePairs)
  # TMC4671.writeRegister(TMC4671.registers.PWM_POLARITIES, 0x00000000)
  # TMC4671.writeRegister(TMC4671.registers.PWM_MAXCNT, int(0x00000F9F))
  # TMC4671.writeRegister(TMC4671.registers.PWM_BBM_H_BBM_L, 0x00000505)
  # TMC4671.writeRegister(TMC4671.registers.PWM_SV_CHOP, 0x00000007)
  # 
  # " ADC configuration "
  # TMC4671.writeRegister(TMC4671.registers.ADC_I_SELECT, 0x18000100)
  # TMC4671.writeRegister(TMC4671.registers.dsADC_MCFG_B_MCFG_A, 0x00100010)
  # TMC4671.writeRegister(TMC4671.registers.dsADC_MCLK_A, 0x20000000)
  # TMC4671.writeRegister(TMC4671.registers.dsADC_MCLK_B, 0x00000000)
  # TMC4671.writeRegister(TMC4671.registers.dsADC_MDEC_B_MDEC_A, int(0x014E014E))
  # TMC4671.writeRegister(TMC4671.registers.ADC_I0_SCALE_OFFSET, 0x01008218)
  # TMC4671.writeRegister(TMC4671.registers.ADC_I1_SCALE_OFFSET, 0x0100820A)
  
  " ABN encoder settings "
  #TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_MODE, 0x00001000)
  # TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_PPR, enocoderResolution)
  TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_COUNT, 0x0)
  TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x0)
  
  " Open loop settings "
  TMC4671.writeRegister(TMC4671.registers.OPENLOOP_MODE, 0x00000000)
  TMC4671.writeRegister(TMC4671.registers.OPENLOOP_ACCELERATION, 0x00000080)
  
  " Limits "
  TMC4671.writeRegister(TMC4671.registers.PID_TORQUE_FLUX_LIMITS, 1500)
  
#   " PI settings "
#   TMC4671.writeRegister(TMC4671.registers.PID_TORQUE_P_TORQUE_I, 0x01000100)
#   TMC4671.writeRegister(TMC4671.registers.PID_FLUX_P_FLUX_I, 0x01000100)
  
  if not calibrate_adc_offset(TMC4671):
    print("Calibration failed. Stopping")
    myInterface.close()
    return;
    
  activeCalibrateEncoder = True

  if activeCalibrateEncoder:
    if not calibrate_abn_encoder_active(TMC4671,  encoderResolution, polePairs, pre_pos_revs=2, flux_mA=1700):
      print("Active Encoder Calibration failed. Stopping")
      myInterface.close()
      
  else:
    if not calibrate_abn_encoder_passive(TMC4671,  encoderResolution, polePairs, flux_mA=1500):
      print("Passive Encoder Calibration failed. Stopping")
      myInterface.close()
    
    
  " ===== 4) go to encoder mode ===== "
   
  " Feedback selection "
  TMC4671.writeRegister(TMC4671.registers.PHI_E_SELECTION, TMC4671.registers.PHI_E_ABN)
  TMC4671.writeRegister(TMC4671.registers.VELOCITY_SELECTION, TMC4671.registers.VELOCITY_PHI_E_ABN)
   
   
  # 
  # " ===== 5) pull in line ====="   
   
   
  line_in_velocity = 500
  line_in_flux_limit = 1700
  line_in_timeout = 1.0
  
  " Switch to torque mode "
  TMC4671.writeRegister(TMC4671.registers.PID_TORQUE_FLUX_TARGET, 0)
  TMC4671.writeRegister(TMC4671.registers.MODE_RAMP_MODE_MOTION, TMC4671.registers.MOTION_MODE_VELOCITY)
  TMC4671.writeRegister(TMC4671.registers.PID_TORQUE_FLUX_TARGET, line_in_flux_limit) 
  TMC4671.writeRegister(TMC4671.registers.PID_TORQUE_FLUX_LIMITS, line_in_flux_limit) 
  
  TMC4671.writeRegister(TMC4671.registers.PID_ACCELERATION_LIMIT, 100)
  TMC4671.writeRegister(TMC4671.registers.PID_VELOCITY_TARGET, line_in_velocity)

  start_time = time.time()
  timeout = start_time + 10.0
  line_in_exit = False
  line_stopped = False;
  line_stopped_timeout = 0
  
  while(not line_in_exit):
    velocity = TMC4671.readRegister(TMC4671.registers.PID_VELOCITY_ACTUAL, signed=True)
    abs_velocity = math.fabs(velocity)
    if abs_velocity > (line_in_velocity*10):
#       print("Invalid abs velocity:" + str(abs_velocity))
      continue
    else:
      
      now = time.time()
      if now > timeout:
        print("Line end timeout")
        line_in_exit = True

      velocity_delta = math.fabs(line_in_velocity - velocity);
#       print("line retrieve velocity: " + str(velocity), "velocity_delta: " + str(velocity_delta))
        
      if velocity_delta > (line_in_velocity * 0.5):
        if line_stopped == False:
          print("Line stop detected")
          line_stopped_timeout = time.time() + line_in_timeout
        line_stopped = True
      else:
        if line_stopped == True:
            print("Line started")
        line_stopped = False
        
      if line_stopped and (time.time() > line_stopped_timeout):
          print("Line stopped timeout")
          line_in_exit = True
        

      delta_time = now - start_time

  print("Starting exersize")
                 
  TMC4671.writeRegister(TMC4671.registers.PID_VELOCITY_TARGET, 0)
  TMC4671.writeRegister(TMC4671.registers.PID_VELOCITY_LIMIT, 12000)  
  TMC4671.writeRegister(TMC4671.registers.PID_ACCELERATION_LIMIT, 1000)  
  TMC4671.writeRegister(TMC4671.registers.PID_POSITION_ACTUAL, 0)
#  TMC4671.writeRegister(TMC4671.registers.PID_POSITION_LIMIT_LOW, 0)

  TMC4671.writeRegister(TMC4671.registers.MODE_RAMP_MODE_MOTION, TMC4671.registers.MOTION_MODE_POSITION)
  
  exersize = Exersize()
  
  while(1):
    velocity = TMC4671.readRegister(TMC4671.registers.PID_VELOCITY_ACTUAL, signed=True)
    position = TMC4671.readRegister(TMC4671.registers.PID_POSITION_ACTUAL, signed=True)
    torque_flux = TMC4671.readRegister(TMC4671.registers.PID_TORQUE_FLUX_ACTUAL, signed=True)

    torque = (torque_flux >> 16) & 0xFFFF
    if torque & 0x8000:
      torque = torque - 0x10000
    
    flux = (torque_flux & 0xFFFF)
    if flux & 0x8000:
      flux = flux - 0x10000
      
# //    motor_rotation = position / (256 * encoderResolution)
    motor_position = float(position) / ( float(1<<16) * polePairs)
    drum_position = motor_position * GEAR_RATIO
    line_postion = drum_position * (math.pi * drum_diam)
    
    torque_motor_Nm = exersizeMachine.torqueDemandToMotorTorque(torque)    
    force_line_N = exersizeMachine.motorTorqueToLineForce(torque_motor_Nm)

    #force_line_N = exersizeMachine.motorTorqueToLineForce(motor_torque)
    
    motor_rev_rate = velocity  / ( polePairs * 60 )
    drum_rev_rate = motor_rev_rate * GEAR_RATIO
    line_velocity = drum_rev_rate * math.pi * drum_diam
    
    exersize_setting = exersize.update_setting(position, velocity, force_line_N)

    extend_torque = exersizeMachine.lineForceToMotorTorque(exersize_setting.force_target) - exersizeMachine.drag_torque
    contract_torque = exersizeMachine.lineForceToMotorTorque(exersize_setting.force_target) + exersizeMachine.drag_torque
  
#     print("extend_torque:", extend_torque, " contract_torque:", contract_torque)
  
    extend_torque_demand = exersizeMachine.motorTorqueToTorqueDemand(extend_torque)
    if extend_torque_demand < 0:
      extend_torque_demand = 0
    
    contract_torque_demand = exersizeMachine.motorTorqueToTorqueDemand(contract_torque)
    if contract_torque_demand < 0:
      contract_torque_demand = 0

    str = "dist:{:<+2.3f} vel{:<1.3f} force:{:<+3.1f}".format(line_postion,  line_velocity, force_line_N)
    print(str)
    if(line_velocity < 0.01):
      TMC4671.writeRegister(TMC4671.registers.PID_TORQUE_FLUX_LIMITS, extend_torque_demand )
    else:
      TMC4671.writeRegister(TMC4671.registers.PID_TORQUE_FLUX_LIMITS, contract_torque_demand )


      
  # 
  # " ===== 5) make a testdrive ====="
  # 
  # maxVelocity = 0
  # minVelocity = 0
  # 
  # TMC4671.writeRegister(TMC4671.registers.PID_VELOCITY_LIMIT, 200)
  # TMC4671.writeRegister(TMC4671.registers.PID_ACCELERATION_LIMIT, 200)
  # 
  # FLUX_LIM = 2000
  # TORQUE_LIM = 2500
  # 
  # print("rotate right...")
  # 
  # torque_flux_target = (int(FLUX_LIM) & 0xFFFF) | ( (int(TORQUE_LIM) << 16) & 0xFFFF0000)
  # print("torque_flux_target:0x%X" % torque_flux_target);
  # TMC4671.writeRegister(TMC4671.registers.PID_TORQUE_FLUX_TARGET, torque_flux_target)
  # 
  # 
  # startTime = time.time()
  # while True:
  #     velocity = TMC4671.readRegister(TMC4671.registers.PID_VELOCITY_ACTUAL, signed=True)
  #     print("velocity: " + str(velocity))
  #     if velocity > maxVelocity:
  #         maxVelocity = velocity
  #  
  #     " stop after 3 seconds "
  #     if (time.time()-startTime) > 2:
  #         break
  # 
  # print("rotate left...")
  # torque_flux_target = (int(-FLUX_LIM) & 0xFFFF) | ( (int(-TORQUE_LIM) << 16) & 0xFFFF0000)
  # print("torque_flux_target:0x%X" % torque_flux_target);
  # TMC4671.writeRegister(TMC4671.registers.PID_TORQUE_FLUX_TARGET, torque_flux_target)
  # # TMC4671.writeRegister(TMC4671.registers.PID_TORQUE_FLUX_TARGET, -FLUX_LIM - (TORQUE_LIM * 2^16) )
  #    
  # startTime = time.time()
  # while True:
  #     velocity = TMC4671.readRegister(TMC4671.registers.PID_VELOCITY_ACTUAL, signed=True)
  #     print("velocity: " + str(velocity))
  #     if velocity < minVelocity:
  #         minVelocity = velocity
  #    
  #     " stop after 3 seconds "
  #     if (time.time()-startTime) > 2:
  #         break
  # 
  print("stop motor")
  TMC4671.writeRegister(TMC4671.registers.PID_TORQUE_FLUX_TARGET, 0)
  # 
  # 
  # " ===== 6) short summary ====="
  # 
  # print(" === summary === ")
  # print("abn_decoder_count_n:" + str(decoderCountN))
  # print("estimated encoder offset: " + str(decoderCountN_offset))
  # print("maxVelocity:" + str(maxVelocity))
  # print("minVelocity:" + str(minVelocity))
     
  
  myInterface.close()



if __name__ == "__main__":
    # execute only if run as a script
    main()