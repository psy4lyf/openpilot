import struct

import common.numpy_fast as np
from selfdrive.config import Conversions as CV
 #2018.09.03 DV modified remove specific car

# *** Honda specific ***
def can_cksum(mm):
  s = 0
  for c in mm:
    c = ord(c)
    s += (c>>4)
    s += c & 0xF
  s = 8-s
  s %= 0x10
  return s


def fix(msg, addr):
  msg2 = msg[0:-1] + chr(ord(msg[-1]) | can_cksum(struct.pack("I", addr)+msg))
  return msg2

#2018.09.12 this one without alive and checksum
#def make_can_msg(addr, dat, alt):
  #return [addr, 0, dat, alt]

#2018.09.12 12:01PM EST this is for Honda message with alive and checksum
def make_can_msg(addr, dat, idx, alt):
  if idx is not None:
    dat += chr(idx << 4)
    dat = fix(dat, addr)
  return [addr, 0, dat, alt]

#2018.09.01 this one we should add fingerprint CAR.SOUL ,
def create_brake_command(packer, apply_brake, pcm_override, pcm_cancel_cmd, chime, fcw, idx):
  """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""
  brakelights = apply_brake > 0
  pcm_fault_cmd = False

  values = {
    "CRUISE_OVERRIDE": pcm_override,
    "CRUISE_FAULT_CMD": pcm_fault_cmd,
    "CRUISE_CANCEL_CMD": pcm_cancel_cmd,
    "SET_ME_0X80": 0x80,
    "BRAKE_LIGHTS": brakelights,
    "CHIME": chime,
    "FCW": fcw << 1,  # TODO: Why are there two bits for fcw? According to dbc file the first bit should also work
  }
  return packer.make_can_msg("BRAKE_COMMAND", 0, values, idx)

def create_brake_command_soul(packer, apply_brake, idx):
  """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""
  brake_rq = apply_brake > 0
  if brake_rq == True:
    x = 0xCC05
  else:
    x = 0x0000

  values = {
    "BRAKE_COMMAND_pedal_command": apply_brake,   # computer
    "BRAKE_COMMAND_magic": x ,
  }
  return packer.make_can_msg("SOUL_BRAKE_COMMAND", 0, values, idx)  #remove idx no need for alive counter and checksum

def create_brake_enable_soul(packer, apply_brake, idx):
  """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""

  brake_rq = apply_brake > 0

  if brake_rq == True:
    x = 0xCC05
  else:
    x = 0x0000

  values = {
    "BRAKE_ENABLE_magic": x,
  }
  return packer.make_can_msg("SOUL_BRAKE_ENABLE", 0, values, idx) #remove idx no need for alive counter and checksum

def create_brake_disable_soul(packer, apply_brake, idx):
  """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""
  brake_rq = apply_brake > 0

  if brake_rq == False:
    x = 0xCC05
  else:
    x = 0x0000

  values = {
    "BRAKE_DISABLE_magic": x,
  }
  return packer.make_can_msg("SOUL_BRAKE_DISABLE", 0, values, idx) #remove idx no need for alive counter and checksum


def create_gas_command(packer, gas_amount, idx):
  """Creates a CAN message for the Honda DBC GAS_COMMAND."""
  enable = gas_amount > 0.001

  values = {} #initializing the value dict empty initially
  if enable == True:
    x_gas = 0xCC05
  else:
    x_gas = 0x0000

  if enable:
    values["THROTTLE_COMMAND_magic"] = x_gas
    values["THROTTLE_COMMAND_pedal_command"] = gas_amount 

  return packer.make_can_msg("THROTTLE_COMMAND", 0, values, idx) #remove idx no need for alive counter and checksum

def create_gas_command_enable(packer, gas_amount, idx):
  """Creates a CAN message for the Honda DBC GAS_COMMAND."""
  enable = gas_amount > 0.001

  values = {} #initializing the value dict empty initially
  if enable == True:
    x_gas_enable = 0xCC05
  else:
    x_gas_enable = 0x0000

  if enable:
    values["THROTTLE_ENABLE_magic"] = x_gas_enable

  return packer.make_can_msg("THROTTLE_ENABLE", 0, values, idx) #remove idx no need for alive counter and checksum

def create_gas_command_disable(packer, gas_amount, idx):
  """Creates a CAN message for the Honda DBC GAS_COMMAND."""
  disable = gas_amount < 0.001

  values = {} #initializing the value dict empty initially
  if disable == True:
    x_gas_disable = 0xCC05
  else:
    x_gas_disable = 0x0000

  if disable:
    values["THROTTLE_DISABLE_magic"] = x_gas_disable

  return packer.make_can_msg("THROTTLE_DISABLE", 0, values, idx) #remove idx no need for alive counter and checksum


def create_steering_control(packer, apply_steer, lkas_active, idx):
  """Creates a CAN message for the Honda DBC STEERING_CONTROL."""
  if lkas_active == True:
    x_steering_enable = 0xCC05
  else:
    x_steering_enable = 0x0000
  values = {
    "STEERING_COMMAND_magic": apply_steer if lkas_active else 0,
    "STEERING_COMMAND_pedal_command": x_steering_enable,
  }
  return packer.make_can_msg("STEERING_COMMAND", 0 , values, idx) #remove idx no need for alive counter and checksum

def create_steering_control_enable(packer, lkas_active, idx):
  """Creates a CAN message for the Honda DBC STEERING_CONTROL."""
  #print("lkas_active kiacan.py")
  #print(lkas_active)

  if lkas_active == True:
    x_steering_control_enable = 0xCC05
  else:
    x_steering_control_enable = 0x0000

  values= {
    "STEERING_ENABLE_magic": x_steering_control_enable
    }
  return packer.make_can_msg("STEERING_ENABLE", 0, values, idx) #remove idx no need for alive counter and checksum

def create_steering_control_disable(packer, lkas_active, idx):
  """Creates a CAN message for the Honda DBC STEERING_CONTROL."""
  if lkas_active == False:
    x_steering_disable = 0xCC05
  else:
    x_steering_disable = 0x0000
  values = {
    "STEERING_DISABLE_magic": x_steering_disable
  }
  return packer.make_can_msg("STEERING_DISABLE", 0, values, idx) #remove idx no need for alive counter and checksum



def create_ui_commands(packer, pcm_speed, hud, idx):
  """Creates an iterable of CAN messages for the UIs."""
  commands = []

  acc_hud_values = {
      'PCM_SPEED': pcm_speed * CV.MS_TO_KPH,
      'PCM_GAS': hud.pcm_accel,
      'CRUISE_SPEED': hud.v_cruise,
      'ENABLE_MINI_CAR': hud.mini_car,
      'HUD_LEAD': hud.car,
      'SET_ME_X03': 0x03,
      'SET_ME_X03_2': 0x03,
      'SET_ME_X01': 0x01,
  }
  commands.append(packer.make_can_msg("ACC_HUD", 0, acc_hud_values, idx))

  lkas_hud_values = {
    'SET_ME_X41': 0x41,
    'SET_ME_X48': 0x48,
    'STEERING_REQUIRED': hud.steer_required,
    'SOLID_LANES': hud.lanes,
    'BEEP': hud.beep,
  }
  commands.append(packer.make_can_msg('LKAS_HUD', 0, lkas_hud_values, idx))

 #2018.09.12 1:09PM use in honda for auto control highbeam
  #radar_hud_values = {
 #     'ACC_ALERTS': hud.acc_alert,
  #    'LEAD_SPEED': 0x1fe,  # What are these magic values
  #    'LEAD_STATE': 0x7,
  #    'LEAD_DISTANCE': 0x1e,
 # }
 # commands.append(packer.make_can_msg('RADAR_HUD', 0, radar_hud_values, idx)) #2018.09.03 change to bus 0
  return commands

#2018.09.06 12:37AM add comment, this message for radar on bus  1,  channel 2 can of panda
def create_radar_commands(v_ego, car_fingerprint, new_radar_config, idx):
  """Creates an iterable of CAN messages for the radar system."""
  commands = []
  v_ego_kph = np.clip(int(round(v_ego * CV.MS_TO_KPH)), 0, 255)
  speed = struct.pack('!B', v_ego_kph)

  msg_0x300 = ("\xf9" + speed + "\x8a\xd0" +
               ("\x20" if idx == 0 or idx == 3 else "\x00") +
               "\x00\x00")

  msg_0x301 = "\x00\x00\x50\x02\x51\x00\x00"
  commands.append(make_can_msg(0x300, msg_0x300, idx, 1))

  commands.append(make_can_msg(0x301, msg_0x301, idx, 1))
  return commands

def spam_buttons_command(packer, button_val, idx):
  values = {
    'CRUISE_BUTTONS': button_val,
    'CRUISE_SETTING': 0,
  }
  return packer.make_can_msg("SCM_BUTTONS", 0, values, idx)
