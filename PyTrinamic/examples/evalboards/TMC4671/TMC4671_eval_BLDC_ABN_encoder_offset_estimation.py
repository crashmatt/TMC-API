#!/usr/bin/env python3
'''
Created on 26.02.2019

@author: ED
'''

import time
import numpy as np
from PyTrinamic.connections.ConnectionManager import ConnectionManager
from PyTrinamic.evalboards.TMC4671_eval import TMC4671_eval
from PyTrinamic.ic.TMC4671.TMC4671 import TMC4671 as TMC4671_IC
from PyTrinamic.connections.uart_ic_interface import uart_ic_interface


def reject_outliers_2(data, m=2.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d / (mdev if mdev else 1.)
    return data[s < m]
  
  
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
  
  polePairs = 7
  encoderResolution = 4096
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
  TMC4671.writeRegister(TMC4671.registers.PID_TORQUE_FLUX_LIMITS, 1000)
  
  " PI settings "
  TMC4671.writeRegister(TMC4671.registers.PID_TORQUE_P_TORQUE_I, 0x01000100)
  TMC4671.writeRegister(TMC4671.registers.PID_FLUX_P_FLUX_I, 0x01000100)
  
  
  
  " ===== 2) estimate the encoder offset ====="
  
  " Init encoder (mode 0) "
  " put a voltage on the motor and wait 1 second for alignment "
  TMC4671.writeRegister(TMC4671.registers.MODE_RAMP_MODE_MOTION, 0x00000008)
  TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_PHI_E_PHI_M_OFFSET, 0x00000000)
  TMC4671.writeRegister(TMC4671.registers.PHI_E_SELECTION, TMC4671.registers.PHI_E_EXTERNAL)
  TMC4671.writeRegister(TMC4671.registers.PHI_E_EXT, 0x00000000)
  TMC4671.writeRegister(TMC4671.registers.UQ_UD_EXT, 2000)
  time.sleep(1)
  
  " clear abn_decoder_count "
  TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_COUNT, 0x00000000)
  
  print("abn_decoder_count:" + str(TMC4671.readRegister(TMC4671.registers.ABN_DECODER_COUNT)))
  
  " Switch to open loop velocity mode "
  TMC4671.writeRegister(TMC4671.registers.PHI_E_SELECTION, TMC4671.registers.PHI_E_OPEN_LOOP)
  
  openloop_velocity = 30
  
  raw_encoder_offsets = []
  encoder_offsets = []
  direction = False
  
  for iter in range(1,10):
    TMC4671.writeRegister(TMC4671.registers.ABN_DECODER_COUNT_N, 0x00000000)
    
    if direction:
      set_velocity =  -openloop_velocity
    else:
      set_velocity =  openloop_velocity
    direction = not direction
      
    print("Setting velocity to:" + str(set_velocity) )
    TMC4671.writeRegister(TMC4671.registers.OPENLOOP_VELOCITY_TARGET, set_velocity)
  
   
    startTime = time.time()
    loopCnt = 0
    while True:
      decoderCountN = TMC4671.readRegister(TMC4671.registers.ABN_DECODER_COUNT_N)      
      decoderCount = TMC4671.readRegister(TMC4671.registers.ABN_DECODER_COUNT)      
      
#       if loopCnt%100 == 0:
#         print("dec: " + str(decoderCount) + " dec_n: " + str(decoderCountN) )
#       loopCnt += 1
  
      " stop after 3 seconds "
      if (time.time()-startTime) > 3:
          break
        
      if decoderCountN != 0:
        break;
    
    if decoderCountN == 0:
      print("Zero decoder count. decoderCountN:" + str(decoderCountN) + " Aborting offset estimation")
      TMC4671.writeRegister(TMC4671.registers.OPENLOOP_VELOCITY_TARGET, 0)
      TMC4671.writeRegister(TMC4671.registers.UQ_UD_EXT, 0)
    
    raw_encoder_offsets.append(decoderCountN)
    
    decoderCountN_offset = decoderCountN % (encoderResolution / polePairs)
    
    print("abn_decoder_count:" + str(decoderCount))
    print("abn_decoder_count_n:" + str(decoderCountN))
    print("=> estimated encoder offset: " + str(decoderCountN_offset))
    
    encoder_offsets.append(decoderCountN_offset)
    
    time.sleep(0.1)  
    
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
  
  
  # 
  # " ===== 4) got to encoder mode ===== "
  # 
  # " Feedback selection "
  # TMC4671.writeRegister(TMC4671.registers.PHI_E_SELECTION, TMC4671.registers.PHI_E_ABN)
  # TMC4671.writeRegister(TMC4671.registers.VELOCITY_SELECTION, TMC4671.registers.VELOCITY_PHI_M_ABN)
  # 
  # " Switch to torque mode "
  # TMC4671.writeRegister(TMC4671.registers.MODE_RAMP_MODE_MOTION, TMC4671.registers.MOTION_MODE_TORQUE)
  # 
  # 
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