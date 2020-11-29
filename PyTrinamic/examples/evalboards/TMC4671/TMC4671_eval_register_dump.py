#!/usr/bin/env python3
'''
Dump all register values of the TMC4671 IC.

The connection to a Landungsbrücke is established over USB. TMCL commands are
used for communicating with the IC.

Created on 04.11.2019

@author: JM
'''
import PyTrinamic
from PyTrinamic.connections.ConnectionManager import ConnectionManager
from PyTrinamic.evalboards.TMC4671_eval import TMC4671_eval

import time

def register_dump(TMC4671):
  count = 0
  print("CHIPINFO_DATA:                     0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.CHIPINFO_DATA)))
  print("CHIPINFO_ADDR:                     0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.CHIPINFO_ADDR)))
  print("ADC_RAW_DATA:                      0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ADC_RAW_DATA)))
  print("ADC_RAW_ADDR:                      0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ADC_RAW_ADDR)))
  print("dsADC_MCFG_B_MCFG_A:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.dsADC_MCFG_B_MCFG_A)))
  print("dsADC_MCLK_A:                      0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.dsADC_MCLK_A)))
  print("dsADC_MCLK_B:                      0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.dsADC_MCLK_B)))
  print("dsADC_MDEC_B_MDEC_A:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.dsADC_MDEC_B_MDEC_A)))
  print("ADC_I1_SCALE_OFFSET:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ADC_I1_SCALE_OFFSET)))
  print("ADC_I0_SCALE_OFFSET:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ADC_I0_SCALE_OFFSET)))
  count += 10
  print("ADC_I_SELECT:                      0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ADC_I_SELECT)))
  print("ADC_I1_I0_EXT:                     0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ADC_I1_I0_EXT)))
  print("DS_ANALOG_INPUT_STAGE_CFG:         0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.DS_ANALOG_INPUT_STAGE_CFG)))
  print("AENC_0_SCALE_OFFSET:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_0_SCALE_OFFSET)))
  print("AENC_1_SCALE_OFFSET:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_1_SCALE_OFFSET)))
  print("AENC_2_SCALE_OFFSET:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_2_SCALE_OFFSET)))
  print("AENC_SELECT:                       0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_SELECT)))
  print("ADC_IWY_IUX:                       0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ADC_IWY_IUX)))
  print("ADC_IV:                            0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ADC_IV)))
  print("AENC_WY_UX:                        0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_WY_UX)))
  count += 10
  print("AENC_VN:                           0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_VN)))
  print("PWM_POLARITIES:                    0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PWM_POLARITIES)))
  print("PWM_MAXCNT:                        0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PWM_MAXCNT)))
  print("PWM_BBM_H_BBM_L:                   0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PWM_BBM_H_BBM_L)))
  print("PWM_SV_CHOP:                       0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PWM_SV_CHOP)))
  print("MOTOR_TYPE_N_POLE_PAIRS:           0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.MOTOR_TYPE_N_POLE_PAIRS)))
  print("PHI_E_EXT:                         0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PHI_E_EXT)))
  print("PHI_M_EXT:                         0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PHI_M_EXT)))
  print("POSITION_EXT:                      0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.POSITION_EXT)))
  print("OPENLOOP_MODE:                     0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.OPENLOOP_MODE)))
  count += 10
  print("OPENLOOP_ACCELERATION:             0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.OPENLOOP_ACCELERATION)))
  print("OPENLOOP_VELOCITY_TARGET:          0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.OPENLOOP_VELOCITY_TARGET)))
  print("OPENLOOP_VELOCITY_ACTUAL:          0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.OPENLOOP_VELOCITY_ACTUAL)))
  print("OPENLOOP_PHI:                      0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.OPENLOOP_PHI)))
  print("UQ_UD_EXT:                         0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.UQ_UD_EXT)))
  print("ABN_DECODER_MODE:                  0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ABN_DECODER_MODE)))
  print("ABN_DECODER_PPR:                   0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ABN_DECODER_PPR)))
  print("ABN_DECODER_COUNT:                 0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ABN_DECODER_COUNT)))
  print("ABN_DECODER_COUNT_N:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ABN_DECODER_COUNT_N)))
  print("ABN_DECODER_PHI_E_PHI_M_OFFSET:    0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ABN_DECODER_PHI_E_PHI_M_OFFSET)))
  count += 10
  print("ABN_DECODER_PHI_E_PHI_M:           0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ABN_DECODER_PHI_E_PHI_M)))
  print("ABN_2_DECODER_MODE:                0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ABN_2_DECODER_MODE)))
  print("ABN_2_DECODER_PPR:                 0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ABN_2_DECODER_PPR)))
  print("ABN_2_DECODER_COUNT:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ABN_2_DECODER_COUNT)))
  print("ABN_2_DECODER_COUNT_N:             0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ABN_2_DECODER_COUNT_N)))
  print("ABN_2_DECODER_PHI_M_OFFSET:        0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ABN_2_DECODER_PHI_M_OFFSET)))
  print("ABN_2_DECODER_PHI_M:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ABN_2_DECODER_PHI_M)))
  print("HALL_MODE:                         0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.HALL_MODE)))
  print("HALL_POSITION_060_000:             0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.HALL_POSITION_060_000)))
  print("HALL_POSITION_180_120:             0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.HALL_POSITION_180_120)))
  count += 10
  print("HALL_POSITION_300_240:             0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.HALL_POSITION_300_240)))
  print("HALL_PHI_E_PHI_M_OFFSET:           0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.HALL_PHI_E_PHI_M_OFFSET)))
  print("HALL_DPHI_MAX:                     0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.HALL_DPHI_MAX)))
  print("HALL_PHI_E_INTERPOLATED_PHI_E:     0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.HALL_PHI_E_INTERPOLATED_PHI_E)))
  print("HALL_PHI_M:                        0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.HALL_PHI_M)))
  print("AENC_DECODER_MODE:                 0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_DECODER_MODE)))
  print("AENC_DECODER_N_MASK_N_THRESHOLD:   0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_DECODER_N_MASK_N_THRESHOLD)))
  print("AENC_DECODER_PHI_A_RAW:            0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_DECODER_PHI_A_RAW)))
  print("AENC_DECODER_PHI_A_OFFSET:         0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_DECODER_PHI_A_OFFSET)))
  print("AENC_DECODER_PHI_A:                0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_DECODER_PHI_A)))
  count += 10
  print("AENC_DECODER_PPR:                  0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_DECODER_PPR)))
  print("AENC_DECODER_COUNT:                0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_DECODER_COUNT)))
  print("AENC_DECODER_COUNT_N:              0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_DECODER_COUNT_N)))
  print("AENC_DECODER_PHI_E_PHI_M_OFFSET:   0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_DECODER_PHI_E_PHI_M_OFFSET)))
  print("AENC_DECODER_PHI_E_PHI_M:          0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_DECODER_PHI_E_PHI_M)))
  print("AENC_DECODER_POSITION:             0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.AENC_DECODER_POSITION)))
  print("PIDIN_TORQUE_TARGET_FLUX_TARGET:   0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PIDIN_TORQUE_TARGET_FLUX_TARGET)))
  print("PIDIN_VELOCITY_TARGET:             0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PIDIN_VELOCITY_TARGET)))
  print("PIDIN_POSITION_TARGET:             0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PIDIN_POSITION_TARGET)))
  print("CONFIG_DATA:                       0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.CONFIG_DATA)))
  count += 10
  print("CONFIG_ADDR:                       0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.CONFIG_ADDR)))
  print("VELOCITY_SELECTION:                0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.VELOCITY_SELECTION)))
  print("POSITION_SELECTION:                0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.POSITION_SELECTION)))
  print("PHI_E_SELECTION:                   0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PHI_E_SELECTION)))
  print("PHI_E:                             0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PHI_E)))
  print("PID_FLUX_P_FLUX_I:                 0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_FLUX_P_FLUX_I)))
  print("PID_TORQUE_P_TORQUE_I:             0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_TORQUE_P_TORQUE_I)))
  print("PID_VELOCITY_P_VELOCITY_I:         0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_VELOCITY_P_VELOCITY_I)))
  print("PID_POSITION_P_POSITION_I:         0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_POSITION_P_POSITION_I)))
  print("PID_TORQUE_FLUX_TARGET_DDT_LIMITS: 0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_TORQUE_FLUX_TARGET_DDT_LIMITS)))
  count += 10
  print("PIDOUT_UQ_UD_LIMITS:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PIDOUT_UQ_UD_LIMITS)))
  print("PID_TORQUE_FLUX_LIMITS:            0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_TORQUE_FLUX_LIMITS)))
  print("PID_ACCELERATION_LIMIT:            0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_ACCELERATION_LIMIT)))
  print("PID_VELOCITY_LIMIT:                0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_VELOCITY_LIMIT)))
  print("PID_POSITION_LIMIT_LOW:            0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_POSITION_LIMIT_LOW)))
  print("POSITION_LIMIT_HIGH:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.POSITION_LIMIT_HIGH)))
  print("MODE_RAMP_MODE_MOTION:             0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.MODE_RAMP_MODE_MOTION)))
  print("PID_TORQUE_FLUX_TARGET:            0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_TORQUE_FLUX_TARGET)))
  print("PID_TORQUE_FLUX_OFFSET:            0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_TORQUE_FLUX_OFFSET)))
  print("PID_VELOCITY_TARGET:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_VELOCITY_TARGET)))
  count += 10
  print("PID_VELOCITY_OFFSET:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_VELOCITY_OFFSET)))
  print("PID_POSITION_TARGET:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_POSITION_TARGET)))
  print("PID_TORQUE_FLUX_ACTUAL:            0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_TORQUE_FLUX_ACTUAL)))
  print("PID_VELOCITY_ACTUAL:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_VELOCITY_ACTUAL)))
  print("PID_POSITION_ACTUAL:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_POSITION_ACTUAL)))
  print("PID_ERROR_DATA:                    0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_ERROR_DATA)))
  print("PID_ERROR_ADDR:                    0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PID_ERROR_ADDR)))
  print("INTERIM_DATA:                      0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.INTERIM_DATA)))
  print("INTERIM_ADDR:                      0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.INTERIM_ADDR)))
  print("WATCHDOG_CFG:                      0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.WATCHDOG_CFG)))
  count += 10
  print("ADC_VM_LIMITS:                     0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.ADC_VM_LIMITS)))
  print("INPUTS_RAW:                        0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.INPUTS_RAW)))
  print("OUTPUTS_RAW:                       0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.OUTPUTS_RAW)))
  print("STEP_WIDTH:                        0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.STEP_WIDTH)))
  print("UART_BPS:                          0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.UART_BPS)))
  print("UART_ADDRS:                        0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.UART_ADDRS)))
  print("GPIO_dsADCI_CONFIG:                0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.GPIO_dsADCI_CONFIG)))
  print("STATUS_FLAGS:                      0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.STATUS_FLAGS)))
  print("STATUS_MASK:                       0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.STATUS_MASK)))
  print("MOTION_MODE_STOPPED:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.MOTION_MODE_STOPPED)))
  count += 10
  print("MOTION_MODE_TORQUE:                0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.MOTION_MODE_TORQUE)))
  print("MOTION_MODE_VELOCITY:              0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.MOTION_MODE_VELOCITY)))
  print("MOTION_MODE_POSITION:              0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.MOTION_MODE_POSITION)))
  print("MOTION_MODE_UQ_UD_EXT:             0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.MOTION_MODE_UQ_UD_EXT)))
  print("PHI_E_EXTERNAL:                    0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PHI_E_EXTERNAL)))
  print("PHI_E_OPEN_LOOP:                   0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PHI_E_OPEN_LOOP)))
  print("PHI_E_ABN:                         0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PHI_E_ABN)))
  print("PHI_E_HALL:                        0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PHI_E_HALL)))
  print("PHI_E_AENC:                        0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PHI_E_AENC)))
  print("PHI_A_AENC:                        0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.PHI_A_AENC)))
  count += 10
  print("VELOCITY_PHI_E_SELECTION:          0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.VELOCITY_PHI_E_SELECTION)))
  print("VELOCITY_PHI_E_EXT:                0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.VELOCITY_PHI_E_EXT)))
  print("VELOCITY_PHI_E_OPENLOOP:           0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.VELOCITY_PHI_E_OPENLOOP)))
  print("VELOCITY_PHI_E_ABN:                0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.VELOCITY_PHI_E_ABN)))
  print("VELOCITY_PHI_E_HAL:                0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.VELOCITY_PHI_E_HAL)))
  print("VELOCITY_PHI_E_AENC:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.VELOCITY_PHI_E_AENC)))
  print("VELOCITY_PHI_A_AENC:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.VELOCITY_PHI_A_AENC)))
  print("VELOCITY_PHI_M_ABN:                0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.VELOCITY_PHI_M_ABN)))
  print("VELOCITY_PHI_M_ABN_2:              0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.VELOCITY_PHI_M_ABN_2)))
  print("VELOCITY_PHI_M_AENC:               0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.VELOCITY_PHI_M_AENC)))
  count += 10
  print("VELOCITY_PHI_M_HAL:                0x{0:08X}".format(TMC4671.readRegister(TMC4671.registers.VELOCITY_PHI_M_HAL)))
  count += 1
  
  return count

def axis_param(TMC4671, repeats):
  for repeat in range(0, repeats):
    axis_param = TMC4671.connection.axisParameter(32, 0)
  print("axis param:", axis_param)

  
def main():
  PyTrinamic.showInfo()
  
  connectionManager = ConnectionManager()
  myInterface = connectionManager.connect()
  TMC4671 = TMC4671_eval(myInterface)  

  start = time.time()
  reg_count = register_dump(TMC4671)
  end = time.time()
  elapsed = end - start
  print(reg_count, "registers in:", elapsed, "s")
  print(reg_count/elapsed, "registers per second")
  
  
  start = time.time()
  repeats = reg_count
  axis_param(TMC4671, repeats);
  end = time.time()
  elapsed = end - start
  print(repeats, "param reads in:", elapsed, "s")
  print(repeats/elapsed, "param reads per second")
  
  myInterface.close()
  


if __name__ == "__main__":
    # execute only if run as a script
    main()
    