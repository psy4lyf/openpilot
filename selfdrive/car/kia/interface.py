#!/usr/bin/env python
import os
import numpy as np
from cereal import car, log
from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, EventTypes as ET, get_events
from selfdrive.controls.lib.vehicle_model import VehicleModel
#2018.09.02 DV add kia soul
from selfdrive.car.kia.carstate import CarState, get_can_parser
from selfdrive.car.kia.values import CruiseButtons, CM, BP, AH, CAR ,DBC

from selfdrive.controls.lib.planner import A_ACC_MAX

#try:
  #from selfdrive.car.honda.carcontroller import CarController
#except ImportError:
 # CarController = None

#2018.09.02 DV add car kia soul
try:
  from selfdrive.car.kia.carcontroller import CarController
except ImportError:
  CarController = None

# msgs sent for steering controller by camera module on can 0.
# those messages are mutually exclusive on CRV and non-CRV cars
#CAMERA_MSGS = [0xe4, 0x194] #message on Honda car

CAMERA_MSGS_SOUL = [0x82] #Kia Soul Steering command message



def compute_gb_honda(accel, speed):
  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  return float(accel) / 4.8 - creep_brake


def get_compute_gb_acura():
  # generate a function that takes in [desired_accel, current_speed] -> [-1.0, 1.0]
  # where -1.0 is max brake and 1.0 is max gas
  # see debug/dump_accel_from_fiber.py to see how those parameters were generated
  w0 = np.array([[ 1.22056961, -0.39625418,  0.67952657],
                 [ 1.03691769,  0.78210306, -0.41343188]])
  b0 = np.array([ 0.01536703, -0.14335321, -0.26932889])
  w2 = np.array([[-0.59124422,  0.42899439,  0.38660881],
                 [ 0.79973811,  0.13178682,  0.08550351],
                 [-0.15651935, -0.44360259,  0.76910877]])
  b2 = np.array([ 0.15624429,  0.02294923, -0.0341086 ])
  w4 = np.array([[-0.31521443],
                 [-0.38626176],
                 [ 0.52667892]])
  b4 = np.array([-0.02922216])

  def compute_output(dat, w0, b0, w2, b2, w4, b4):
    m0 = np.dot(dat, w0) + b0
    m0 = leakyrelu(m0, 0.1)
    m2 = np.dot(m0, w2) + b2
    m2 = leakyrelu(m2, 0.1)
    m4 = np.dot(m2, w4) + b4
    return m4

  def leakyrelu(x, alpha):
    return np.maximum(x, alpha * x)

  def _compute_gb_acura(accel, speed):
    # linearly extrap below v1 using v1 and v2 data
    v1 = 5.
    v2 = 10.
    dat = np.array([accel, speed])
    if speed > 5.:
      m4 = compute_output(dat, w0, b0, w2, b2, w4, b4)
    else:
      dat[1] = v1
      m4v1 = compute_output(dat, w0, b0, w2, b2, w4, b4)
      dat[1] = v2
      m4v2 = compute_output(dat, w0, b0, w2, b2, w4, b4)
      m4 = (speed - v1) * (m4v2 - m4v1) / (v2 - v1) + m4v1
    return float(m4)

  return _compute_gb_acura

#borrow from subaru interface.py
# canbus number define as 0
class CanBus(object):
  def __init__(self):
    self.powertrain = 0
    self.obstacle = 1

class CarInterface(object):
  def __init__(self, CP, sendcan=None):
    self.CP = CP

    self.frame = 0
    self.last_enable_pressed = 0
    self.last_enable_sent = 0
    self.gas_pressed_prev = False
    self.brake_pressed_prev = False
    self.can_invalid_count = 0

    self.cp = get_can_parser(CP)  # 2018.09.10 2:53PM move to 108
    # *** init the major playeselfdrive/car/kia/carstate.pyrs ***
   # canbus = CanBus()
    self.CS = CarState(CP)    #2018.09.07 11:33AM remove copy from subaru add in canbus borrow from subaru interface.py
    self.VM = VehicleModel(CP)
   # self.cp = get_can_parser(CP)    #2018.09.05 borrow from subaru delete powertrain

    # sending if read only is False
    if sendcan is not None:
      self.sendcan = sendcan
      #2018.09.05 11:41PM change dbc_name to canbus
      self.CC = CarController(self.cp.dbc_name, CP.carFingerprint, CP.enableCamera)  # 2018.09.10 add CP.carFingerprint
      #print("self.cc interface.py dp.dbc_name")
      #print(self.cp.dbc_name)
      #print("interface.py CP.carFingerprint")
     # print(CP.carFingerprint)
     # print(CP.enableCamera)


   # if self.CS.CP.carFingerprint == CAR.DUMMY:   #2018.09.06 12:43AM dummy car for not use
    #    self.compute_gb = get_compute_gb_acura()
   # else:
    self.compute_gb = compute_gb_honda  #2018.09.06 2:56PM remove parentheses

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):
    # limit the pcm accel cmd if:
    # - v_ego exceeds v_target, or
    # - a_ego exceeds a_target and v_ego is close to v_target

    eA = a_ego - a_target
    valuesA = [1.0, 0.1]
    bpA = [0.3, 1.1]

    eV = v_ego - v_target
    valuesV = [1.0, 0.1]
    bpV = [0.0, 0.5]

    valuesRangeV = [1., 0.]
    bpRangeV = [-1., 0.]

    # only limit if v_ego is close to v_target
    speedLimiter = interp(eV, bpV, valuesV)
    accelLimiter = max(interp(eA, bpA, valuesA), interp(eV, bpRangeV, valuesRangeV))

    # accelOverride is more or less the max throttle allowed to pcm: usually set to a constant
    # unless aTargetMax is very high and then we scale with it; this help in quicker restart

    return float(max(0.714, a_target / A_ACC_MAX)) * min(speedLimiter, accelLimiter)

  @staticmethod
  def get_params(candidate, fingerprint):

    ret = car.CarParams.new_message()
    ret.carName = "kia"
    ret.carFingerprint = candidate

        #remove #radar off can from base
    #2018.09.16 10:09PMEST set all output
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

    #ret.safetyModel = car.CarParams.SafetyModels.kia
    #2018.09.16 10:11PMEST set to all output
    ret.safetyModel = car.CarParams.SafetyModels.ALLOUTPUT
     #ret.enableCamera = not any(x for x in CAMERA_MSGS if x in fingerprint)
    ret.enableCamera = not any(x for x in CAMERA_MSGS_SOUL if x in fingerprint)  #2018.09.02 DV change for KIA message
      #ret.enableGasInterceptor = 0x201 in fingerprint
    ret.enableGasInterceptor = 0x93 in fingerprint   #2018.09.02 DV change for gas interceptor to throttle report
    cloudlog.warn("ECU Camera Simulated: %r", ret.enableCamera)
    cloudlog.warn("ECU Gas Interceptor: %r", ret.enableGasInterceptor)

    ret.enableCruise = not ret.enableGasInterceptor

    # kg of standard extra cargo to count for drive, gas, etc...
    std_cargo = 136

    # FIXME: hardcoding honda civic 2016 touring params so they can be used to
    # scale unknown params for other cars
    mass_civic = 2923 * CV.LB_TO_KG + std_cargo
    wheelbase_civic = 2.70
    centerToFront_civic = wheelbase_civic * 0.4
    centerToRear_civic = wheelbase_civic - centerToFront_civic
    rotationalInertia_civic = 2500
    tireStiffnessFront_civic = 192150
    tireStiffnessRear_civic = 202500

    # Optimized car params: tire_stiffness_factor and steerRatio are a result of a vehicle
    # model optimization process. Certain Hondas have an extra steering sensor at the bottom
    # of the steering rack, which improves controls quality as it removes the steering column
    # torsion from feedback.
    # Tire stiffness factor fictitiously lower if it includes the steering column torsion effect.
    # For modeling details, see p.198-200 in "The Science of Vehicle Dynamics (2014), M. Guiggiani"

    ret.steerKiBP, ret.steerKpBP = [[0.], [0.]]

    # # 2018.09.04
    # full torque for 20 deg at 80mph means 0.00007818594 comment from hyundai
    ret.steerKf = 0.00006 # conservative feed-forward


    #2018.09.02 DV add Kia soul #TODO need to modified paramater
    if candidate == CAR.SOUL:
      stop_and_go = False
    #  ret.safetyParam = 8 # define in /boardd/boardd.cc
      ret.mass = 3410. * CV.LB_TO_KG + std_cargo
      ret.wheelbase = 2.66
      ret.centerToFront = ret.wheelbase * 0.41
      ret.steerRatio = 16.0   # 12.3 is spec end-to-end
      tire_stiffness_factor = 0.677
      ret.steerKpV, ret.steerKiV = [[0.6], [0.18]]
      ret.steerKiBP, ret.steerKpBP = [[0.], [0.]] # 2018.09.04 from hyundai
      ret.longitudinalKpBP = [0., 5., 35.]
      ret.longitudinalKpV = [1.2, 0.8, 0.5]
      ret.longitudinalKiBP = [0., 35.]
      ret.longitudinalKiV = [0.18, 0.12]

    elif candidate == CAR.SOUL1:
      stop_and_go = False
      #ret.safetyParam = 8  # define in /boardd/boardd.cc
      ret.mass = 3410. * CV.LB_TO_KG + std_cargo
      ret.wheelbase = 2.66
      ret.centerToFront = ret.wheelbase * 0.41
      ret.steerRatio = 16.0  # 12.3 is spec end-to-end
      tire_stiffness_factor = 0.677
      ret.steerKpV, ret.steerKiV = [[0.6], [0.18]]
      ret.steerKiBP, ret.steerKpBP = [[0.], [0.]]  # 2018.09.04 from hyundai
      ret.longitudinalKpBP = [0., 5., 35.]
      ret.longitudinalKpV = [1.2, 0.8, 0.5]
      ret.longitudinalKiBP = [0., 35.]
      ret.longitudinalKiV = [0.18, 0.12]

    elif candidate == CAR.SOUL2:
      stop_and_go = False
     # ret.safetyParam = 8  # define in /boardd/boardd.cc
      ret.mass = 3410. * CV.LB_TO_KG + std_cargo
      ret.wheelbase = 2.66
      ret.centerToFront = ret.wheelbase * 0.41
      ret.steerRatio = 16.0  # 12.3 is spec end-to-end
      tire_stiffness_factor = 0.677
      ret.steerKpV, ret.steerKiV = [[0.6], [0.18]]
      ret.steerKiBP, ret.steerKpBP = [[0.], [0.]]  # 2018.09.04 from hyundai
      ret.longitudinalKpBP = [0., 5., 35.]
      ret.longitudinalKpV = [1.2, 0.8, 0.5]
      ret.longitudinalKiBP = [0., 35.]
      ret.longitudinalKiV = [0.18, 0.12]

    else:
      raise ValueError("unsupported car %s" % candidate)

     #TODO 2018.09.04 should modified our steering control type to match kia angle
    ret.steerControlType = car.CarParams.SteerControlType.torque
    #ret.steerControlType = car.CarParams.SteerControlType.angle    # 2018.09.06 11:19PM reference in cereal car.capnp

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter. Otherwise, add 0.5 mph margin to not
    # conflict with PCM acc
    ret.minEnableSpeed = -1. if (stop_and_go or ret.enableGasInterceptor) else 25.5 * CV.MPH_TO_MS

    centerToRear = ret.wheelbase - ret.centerToFront
    # TODO: get actual value, for now starting with reasonable value for
    # TODO: 2018.09.02 need to scale this value for Kia soul and other car
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = rotationalInertia_civic * \
                            ret.mass * ret.wheelbase**2 / (mass_civic * wheelbase_civic**2)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront = (tireStiffnessFront_civic * tire_stiffness_factor) * \
                             ret.mass / mass_civic * \
                             (centerToRear / ret.wheelbase) / (centerToRear_civic / wheelbase_civic)
    ret.tireStiffnessRear = (tireStiffnessRear_civic * tire_stiffness_factor) * \
                            ret.mass / mass_civic * \
                            (ret.centerToFront / ret.wheelbase) / (centerToFront_civic / wheelbase_civic)

    # no rear steering, at least on the listed cars above
    ret.steerRatioRear = 0.

    #2018.09.02 Add separation for Kia soul and other below
    if candidate == CAR.SOUL:
      # no max steer limit VS speed
      ret.steerMaxBP = [0.]  # m/s
      ret.steerMaxV = [1.]  # max steer allowed #Todo:2018.09.02 tune in car and change

      ret.gasMaxBP = [0.]  # m/s
      ret.gasMaxV = [0.6] if ret.enableGasInterceptor else [0.]  # max gas allowed  #TODO: 2018.09.02 confirm in car
      ret.brakeMaxBP = [5., 20.]  # m/s
      ret.brakeMaxV = [1., 0.8]  # max brake allowed

      ret.longPidDeadzoneBP = [0.]
      ret.longPidDeadzoneV = [0.]

      ret.stoppingControl = True
      ret.steerLimitAlert = True
      ret.startAccel = 0.5

      ret.steerActuatorDelay = 0.1
      ret.steerRateCost = 0.5

    elif candidate == CAR.SOUL1:
      # no max steer limit VS speed
      ret.steerMaxBP = [0.]  # m/s
      ret.steerMaxV = [1.]  # max steer allowed #Todo:2018.09.02 tune in car and change

      ret.gasMaxBP = [0.]  # m/s
      ret.gasMaxV = [0.6] if ret.enableGasInterceptor else [0.]  # max gas allowed  #TODO: 2018.09.02 confirm in car
      ret.brakeMaxBP = [5., 20.]  # m/s
      ret.brakeMaxV = [1., 0.8]  # max brake allowed

      ret.longPidDeadzoneBP = [0.]
      ret.longPidDeadzoneV = [0.]

      ret.stoppingControl = True
      ret.steerLimitAlert = True
      ret.startAccel = 0.5

      ret.steerActuatorDelay = 0.1
      ret.steerRateCost = 0.5

    elif candidate == CAR.SOUL2:
      # no max steer limit VS speed
      ret.steerMaxBP = [0.]  # m/s
      ret.steerMaxV = [1.]  # max steer allowed #Todo:2018.09.02 tune in car and change

      ret.gasMaxBP = [0.]  # m/s
      ret.gasMaxV = [0.6] if ret.enableGasInterceptor else [0.]  # max gas allowed  #TODO: 2018.09.02 confirm in car
      ret.brakeMaxBP = [5., 20.]  # m/s
      ret.brakeMaxV = [1., 0.8]  # max brake allowed

      ret.longPidDeadzoneBP = [0.]
      ret.longPidDeadzoneV = [0.]

      ret.stoppingControl = True
      ret.steerLimitAlert = True
      ret.startAccel = 0.5

      ret.steerActuatorDelay = 0.1
      ret.steerRateCost = 0.5

    else:
      # no max steer limit VS speed
      ret.steerMaxBP = [0.]  # m/s
      ret.steerMaxV = [1.]   # max steer allowed

      ret.gasMaxBP = [0.]  # m/s
      ret.gasMaxV = [0.6] if ret.enableGasInterceptor else [0.] # max gas allowed
      ret.brakeMaxBP = [5., 20.]  # m/s
      ret.brakeMaxV = [1., 0.8]   # max brake allowed

      ret.longPidDeadzoneBP = [0.]
      ret.longPidDeadzoneV = [0.]

      ret.stoppingControl = True
      ret.steerLimitAlert = True
      ret.startAccel = 0.5

      ret.steerActuatorDelay = 0.1
      ret.steerRateCost = 0.5

    return ret

  # returns a car.CarState
  def update(self, c):
    # ******************* do can recv *******************
    canMonoTimes = []

    self.cp.update(int(sec_since_boot() * 1e9), False)

    self.CS.update(self.cp)

    # create message
    ret = car.CarState.new_message()

    #2018.09.05 add generic toggle for testing steering max#
    #ret.genericToggle = self.CS.generic_toggle

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.aEgo = self.CS.a_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    #2018.09.04 change to vehicle yaw rate values
    ret.yawRate = self.VM.yaw_rate(self.CS.angle_steers * CV.DEG_TO_RAD, self.CS.v_ego)
    ## ret.yawRate = self.CS.yaw_rate  2018.09.10 use yaw calculation from angle
    ret.standstill = self.CS.standstill
    ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # gas pedal
    #ret.gas = self.CS.car_gas / 256.0
    ret.gas = self.CS.car_gas    #2018.09.13 12:39AM change gas to car_gas without deviding
    #TODO 2018.09.05 check Throttle report operator override match this
    if not self.CP.enableGasInterceptor:
      ret.gasPressed = self.CS.pedal_gas > 0
    else:
      ret.gasPressed = self.CS.user_gas_pressed

    # brake pedal
    ret.brake = self.CS.user_brake
    ret.brakePressed = self.CS.brake_pressed != 0
    # FIXME: read sendcan for brakelights
    brakelights_threshold = 0.02 if self.CS.CP.carFingerprint == CAR.SOUL else 0.1  #2018.09.03 change CIVIC to soul
    ret.brakeLights = bool(self.CS.brake_switch or
                           c.actuators.brake > brakelights_threshold)

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers
   # ret.steeringRate = self.CS.angle_steers_rate  #2018.09.11 TODO some can duplication error? or not reading

    # gear shifter lever (#2018.09.04 define in carstate.py
    ret.gearShifter = self.CS.gear_shifter


    #2018.09.05 #TODO find steering torque value from steering wheel
    ret.steeringTorque = self.CS.steer_torque_driver
    ret.steeringPressed = self.CS.steer_override  #2018.09.02 this come values py when steering operator override =1

    #2018.09.02 TODO need to check is matter, we not using kia cruise control
    # cruise state
    #ret.cruiseState.enabled = self.CS.pcm_acc_status != 0
    ret.cruiseState.enabled = False  #2018.09.11 if use Openpilot cruise, no need, this should set to 0, need set to False
    #ret.cruiseState.speed = self.CS.v_cruise_pcm * CV.KPH_TO_MS #2018.09.11 check in controlsd.py if not enable cruise, this not needed
    ret.cruiseState.speed = 0  # 2018.09.11 check in controlsd.py if not enable cruise, this not needed, set to 0
    ret.cruiseState.available = bool(self.CS.main_on)  #2018.09.11 not use anywhere else beside interface.py
    ret.cruiseState.speedOffset = self.CS.cruise_speed_offset  #2018.09.11 finally not use in controlsd.py if we use interceptor and enablecruise=false
    ret.cruiseState.standstill = False

    # TODO: button presses
    buttonEvents = []
    ret.leftBlinker = bool(self.CS.left_blinker_on)
    ret.rightBlinker = bool(self.CS.right_blinker_on)

    ret.doorOpen = not self.CS.door_all_closed
    ret.seatbeltUnlatched = not self.CS.seatbelt

    if self.CS.left_blinker_on != self.CS.prev_left_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'leftBlinker'
      be.pressed = self.CS.left_blinker_on != 0
      buttonEvents.append(be)

    if self.CS.right_blinker_on != self.CS.prev_right_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'rightBlinker'
      be.pressed = self.CS.right_blinker_on != 0
      buttonEvents.append(be)

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'unknown'
      if self.CS.cruise_buttons != 0:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == CruiseButtons.RES_ACCEL:
        be.type = 'accelCruise'
      elif but == CruiseButtons.DECEL_SET:
        be.type = 'decelCruise'
      elif but == CruiseButtons.CANCEL:
        be.type = 'cancel'
      elif but == CruiseButtons.MAIN:
        be.type = 'altButton3'
      buttonEvents.append(be)

    if self.CS.cruise_setting != self.CS.prev_cruise_setting:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'unknown'
      if self.CS.cruise_setting != 0:
        be.pressed = True
        but = self.CS.cruise_setting
      else:
        be.pressed = False
        but = self.CS.prev_cruise_setting
      if but == 1:
        be.type = 'altButton1'
      # TODO: more buttons?
      buttonEvents.append(be)
    ret.buttonEvents = buttonEvents

    # events
    # TODO: I don't like the way capnp does enums
    # These strings aren't checked at compile time
    events = []
    if not self.CS.can_valid:
      self.can_invalid_count += 1
      if self.can_invalid_count >= 5:
        events.append(create_event('commIssue', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    else:
      self.can_invalid_count = 0
    if self.CS.steer_error:
      events.append(create_event('steerUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE, ET.PERMANENT]))
    elif self.CS.steer_warning:
      events.append(create_event('steerTempUnavailable', [ET.WARNING]))
    if self.CS.brake_error:
      events.append(create_event('brakeUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE, ET.PERMANENT]))
    if not ret.gearShifter == 'drive':
      events.append(create_event('wrongGear', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.doorOpen:
      events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.seatbeltUnlatched:
      events.append(create_event('seatbeltNotLatched', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if self.CS.esp_disabled:
      events.append(create_event('espDisabled', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if not self.CS.main_on:
      events.append(create_event('wrongCarMode', [ET.NO_ENTRY, ET.USER_DISABLE]))
    if ret.gearShifter == "reverse":
      events.append(create_event('reverseGear', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))

  #remove brake hold and #electric parking brake

    if self.CP.enableCruise and ret.vEgo < self.CP.minEnableSpeed:
      events.append(create_event('speedTooLow', [ET.NO_ENTRY]))

    # disable on pedals rising edge or when brake is pressed and speed isn't zero
    if (ret.gasPressed and not self.gas_pressed_prev) or \
       (ret.brakePressed and (not self.brake_pressed_prev or ret.vEgo > 0.001)):
      events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))

    if ret.gasPressed:
      events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))

    # it can happen that car cruise disables while comma system is enabled: need to
    # keep braking if needed or if the speed is very low
    if self.CP.enableCruise and not ret.cruiseState.enabled and c.actuators.brake <= 0.:
      # non loud alert if cruise disbales below 25mph as expected (+ a little margin)
      if ret.vEgo < self.CP.minEnableSpeed + 2.:
        events.append(create_event('speedTooLow', [ET.IMMEDIATE_DISABLE]))
      else:
        events.append(create_event("cruiseDisabled", [ET.IMMEDIATE_DISABLE]))
    if self.CS.CP.minEnableSpeed > 0 and ret.vEgo < 0.001:
      events.append(create_event('manualRestart', [ET.WARNING]))

    cur_time = sec_since_boot()
    enable_pressed = False
    # handle button presses
    for b in ret.buttonEvents:

      # do enable on both accel and decel buttons
      if b.type in ["accelCruise", "decelCruise"] and not b.pressed:
        self.last_enable_pressed = cur_time
        enable_pressed = True

      # do disable on button down
      if b.type == "cancel" and b.pressed:
        events.append(create_event('buttonCancel', [ET.USER_DISABLE]))

    if self.CP.enableCruise:
      # KEEP THIS EVENT LAST! send enable event if button is pressed and there are
      # NO_ENTRY events, so controlsd will display alerts. Also not send enable events
      # too close in time, so a no_entry will not be followed by another one.
      # TODO: button press should be the only thing that triggers enble
      if ((cur_time - self.last_enable_pressed) < 0.2 and
          (cur_time - self.last_enable_sent) > 0.2 and
          ret.cruiseState.enabled) or \
         (enable_pressed and get_events(events, [ET.NO_ENTRY])):
        events.append(create_event('buttonEnable', [ET.ENABLE]))
        self.last_enable_sent = cur_time
    elif enable_pressed:
      events.append(create_event('buttonEnable', [ET.ENABLE]))

    ret.events = events
    ret.canMonoTimes = canMonoTimes

    # update previous brake/gas pressed
    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed

    # cast to reader so it can't be modified
    return ret.as_reader()

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c, perception_state=log.Live20Data.new_message()):
    if c.hudControl.speedVisible:
      hud_v_cruise = c.hudControl.setSpeed * CV.MS_TO_KPH
    else:
      hud_v_cruise = 255

    hud_alert = {
      "none": AH.NONE,
      "fcw": AH.FCW,
      "steerRequired": AH.STEER,
      "brakePressed": AH.BRAKE_PRESSED,
      "wrongGear": AH.GEAR_NOT_D,
      "seatbeltUnbuckled": AH.SEATBELT,
      "speedTooHigh": AH.SPEED_TOO_HIGH}[str(c.hudControl.visualAlert)]

    snd_beep, snd_chime = {
      "none": (BP.MUTE, CM.MUTE),
      "beepSingle": (BP.SINGLE, CM.MUTE),
      "beepTriple": (BP.TRIPLE, CM.MUTE),
      "beepRepeated": (BP.REPEATED, CM.MUTE),
      "chimeSingle": (BP.MUTE, CM.SINGLE),
      "chimeDouble": (BP.MUTE, CM.DOUBLE),
      "chimeRepeated": (BP.MUTE, CM.REPEATED),
      "chimeContinuous": (BP.MUTE, CM.CONTINUOUS)}[str(c.hudControl.audibleAlert)]

    pcm_accel = int(clip(c.cruiseControl.accelOverride,0,1)*0xc6)

    self.CC.update(self.sendcan, c.enabled, self.CS, self.frame, \
      c.actuators, \
      c.cruiseControl.speedOverride, \
      c.cruiseControl.override, \
      c.cruiseControl.cancel, \
      pcm_accel, \
      perception_state.radarErrors, \
      hud_v_cruise, c.hudControl.lanesVisible, \
      hud_show_car = c.hudControl.leadVisible, \
      hud_alert = hud_alert, \
      snd_beep = snd_beep, \
      snd_chime = snd_chime)

    self.frame += 1
