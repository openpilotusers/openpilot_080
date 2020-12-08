from numpy import clip
from common.realtime import DT_CTRL
from cereal import car, log, messaging
from common.numpy_fast import interp
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.carstate import GearShifter
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfa_mfa, \
                                             create_scc11, create_scc12, create_scc13, create_scc14, \
                                             create_scc42a, create_scc7d0, create_fca11, create_fca12, create_mdps12
from selfdrive.car.hyundai.values import Buttons, SteerLimitParams, CAR, FEATURES
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.longcontrol import LongCtrlState

from selfdrive.controls.lib.pathplanner import LANE_CHANGE_SPEED_MIN

from common.params import Params
import common.log as trace1
import common.CTime1000 as tm

VisualAlert = car.CarControl.HUDControl.VisualAlert

# Accel Hard limits
ACCEL_HYST_GAP = 0.1  # don't change accel command for small oscillations within this value
ACCEL_MAX = 2.  # 2.0 m/s2
ACCEL_MIN = -3.5  # 3.5   m/s2
ACCEL_SCALE = 1.

def accel_hysteresis(accel, accel_steady):

  # for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if accel > accel_steady + ACCEL_HYST_GAP:
    accel_steady = accel - ACCEL_HYST_GAP
  elif accel < accel_steady - ACCEL_HYST_GAP:
    accel_steady = accel + ACCEL_HYST_GAP
  accel = accel_steady

  return accel, accel_steady

def accel_rate_limit(accel_lim, prev_accel_lim):

  if accel_lim > 0:
    if accel_lim > prev_accel_lim:
      accel_lim = min(accel_lim, prev_accel_lim + 0.02)
    else:
      accel_lim = max(accel_lim, prev_accel_lim - 0.035)
  else:
    if accel_lim < prev_accel_lim:
      accel_lim = max(accel_lim, prev_accel_lim - 0.035)
    else:
      accel_lim = min(accel_lim, prev_accel_lim + 0.01)

  return accel_lim

def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart):

  sys_warning = (visual_alert == VisualAlert.steerRequired)
  if sys_warning:
      sys_warning = 1 if fingerprint in [CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 3

  if enabled or sys_warning:
      sys_state = 3
  else:
      sys_state = 1

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  #if left_lane_depart:
  #  left_lane_warning = 1 if fingerprint in [CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 2
  #if right_lane_depart:
  #  right_lane_warning = 1 if fingerprint in [CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.cp_oplongcontrol = CP.openpilotLongitudinalControl
    self.packer = CANPacker(dbc_name)
    self.accel_steady = 0
    self.accel_lim_prev = 0.
    self.accel_lim = 0.
    self.steer_rate_limited = False
    self.usestockscc = True
    self.lead_visible = False
    self.lead_debounce = 0
    self.prev_gapButton = 0
    self.current_veh_speed = 0
    self.lfainFingerprint = CP.lfaAvailable
    self.vdiff = 0
    self.resumebuttoncnt = 0
    self.lastresumeframe = 0
    self.fca11supcnt = self.fca11inc = self.fca11alivecnt = self.fca11cnt13 = self.scc11cnt = self.scc12cnt = 0
    self.counter_init = False
    self.fca11maxcnt = 0xD
    self.radarDisableActivated = False
    self.radarDisableResetTimer = 0
    self.radarDisableOverlapTimer = 0
    self.sendaccmode = not CP.radarDisablePossible
    self.enabled = False
    self.sm = messaging.SubMaster(['controlsState'])

    self.lanechange_manual_timer = 0
    self.emergency_manual_timer = 0
    self.driver_steering_torque_above = False
    self.driver_steering_torque_above_timer = 100

    self.acc_standstill_timer = 0
    
    self.params = Params()
    self.gapsettingdance = int(self.params.get('OpkrCruiseGapSet'))
    self.opkr_autoresume = int(self.params.get('OpkrAutoResume'))

    self.opkr_maxanglelimit = int(self.params.get('OpkrMaxAngleLimit'))

    self.timer1 = tm.CTime1000("time")
    
    self.angle_differ_range = [0, 45]
    self.steerMax_range = [int(self.params.get('SteerMaxBaseAdj')), SteerLimitParams.STEER_MAX]
    self.steerDeltaUp_range = [int(self.params.get('SteerDeltaUpAdj')), 5]
    self.steerDeltaDown_range = [int(self.params.get('SteerDeltaDownAdj')), 10]

    self.steerMax = int(self.params.get('SteerMaxBaseAdj'))
    self.steerDeltaUp = int(self.params.get('SteerDeltaUpAdj'))
    self.steerDeltaDown = int(self.params.get('SteerDeltaDownAdj'))
    self.steerMax_prev = int(self.params.get('SteerMaxBaseAdj'))
    self.steerDeltaUp_prev = int(self.params.get('SteerDeltaUpAdj'))
    self.steerDeltaDown_prev = int(self.params.get('SteerDeltaDownAdj'))
    self.steerMax_timer = 0
    self.steerDeltaUp_timer = 0
    self.steerDeltaDown_timer = 0

    if CP.lateralTuning.which() == 'pid':
      self.str_log2 = 'TUNE={:0.2f}/{:0.3f}/{:0.5f}'.format(CP.lateralTuning.pid.kpV[1], CP.lateralTuning.pid.kiV[1], CP.lateralTuning.pid.kf)
    elif CP.lateralTuning.which() == 'indi':
      self.str_log2 = 'TUNE={:03.1f}/{:03.1f}/{:03.1f}/{:03.1f}'.format(CP.lateralTuning.indi.innerLoopGain, CP.lateralTuning.indi.outerLoopGain, CP.lateralTuning.indi.timeConstant, CP.lateralTuning.indi.actuatorEffectiveness)
    elif CP.lateralTuning.which() == 'lqr':
      self.str_log2 = 'TUNE={:04.0f}/{:05.3f}/{:06.4f}'.format(CP.lateralTuning.lqr.scale, CP.lateralTuning.lqr.ki, CP.lateralTuning.lqr.dcGain)

    self.p = SteerLimitParams

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert,
             left_lane, right_lane, left_lane_depart, right_lane_depart,
             set_speed, lead_visible, lead_dist, lead_vrel, lead_yrel, sm):

    self.enabled = enabled
    # gas and brake
    self.accel_lim_prev = self.accel_lim
    apply_accel = actuators.gas - actuators.brake

    apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady)
    apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX)

    self.accel_lim = apply_accel
    apply_accel = accel_rate_limit(self.accel_lim, self.accel_lim_prev)

    param = self.p

    path_plan = sm['pathPlan']
    self.outScale = path_plan.outputScale

    self.angle_steers_des = path_plan.angleSteers - path_plan.angleOffset
    self.angle_steers = CS.out.steeringAngle
    self.angle_diff = abs(self.angle_steers_des) - abs(self.angle_steers)

    if abs(self.outScale) >= 1 and CS.out.vEgo > 8:
      self.steerMax_prev = interp(self.angle_diff, self.angle_differ_range, self.steerMax_range)
      if self.steerMax_prev > self.steerMax:
        self.steerMax = self.steerMax_prev
      self.steerDeltaUp_prev = interp(self.angle_diff, self.angle_differ_range, self.steerDeltaUp_range)
      if self.steerDeltaUp_prev > self.steerDeltaUp:
        self.steerDeltaUp = self.steerDeltaUp_prev
      self.steerDeltaDown_prev = interp(self.angle_diff, self.angle_differ_range, self.steerDeltaDown_range)
      if self.steerDeltaDown_prev > self.steerDeltaDown:
        self.steerDeltaDown = self.steerDeltaDown_prev
    else:
      self.steerMax_timer += 1
      self.steerDeltaUp_timer += 1
      self.steerDeltaDown_timer += 1
      if self.steerMax_timer > 20:
        self.steerMax -= 5
        self.steerMax_timer = 0
        if self.steerMax < int(self.params.get('SteerMaxBaseAdj')):
          self.steerMax = int(self.params.get('SteerMaxBaseAdj'))
      if self.steerDeltaUp_timer > 100:
        self.steerDeltaUp -= 1
        self.steerDeltaUp_timer = 0
        if self.steerDeltaUp <= int(self.params.get('SteerDeltaUpAdj')):
          self.steerDeltaUp = int(self.params.get('SteerDeltaUpAdj'))
      if self.steerDeltaDown_timer > 50:
        self.steerDeltaDown -= 1
        self.steerDeltaDown_timer = 0
        if self.steerDeltaDown <= int(self.params.get('SteerDeltaDownAdj')):
          self.steerDeltaDown = int(self.params.get('SteerDeltaDownAdj'))

    param.STEER_MAX = min(SteerLimitParams.STEER_MAX, self.steerMax)
    param.STEER_DELTA_UP = max(int(self.params.get('SteerDeltaUpAdj')), self.steerDeltaUp)
    param.STEER_DELTA_DOWN = max(int(self.params.get('SteerDeltaDownAdj')), self.steerDeltaDown)

    # Steering Torque
    if 0 <= self.driver_steering_torque_above_timer < 100:
      new_steer = actuators.steer * self.steerMax * (self.driver_steering_torque_above_timer / 100)
    else:
      new_steer = actuators.steer * self.steerMax
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, param)
    self.steer_rate_limited = new_steer != apply_steer

    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    self.high_steer_allowed = True if self.car_fingerprint in FEATURES["allow_high_steer"] else False
    if self.opkr_maxanglelimit >= 90:
      lkas_active = enabled and abs(CS.out.steeringAngle) < self.opkr_maxanglelimit
    else:
      lkas_active = enabled

    if (( CS.out.leftBlinker and not CS.out.rightBlinker) or ( CS.out.rightBlinker and not CS.out.leftBlinker)) and CS.out.vEgo < LANE_CHANGE_SPEED_MIN:
      self.lanechange_manual_timer = 50
    if CS.out.leftBlinker and CS.out.rightBlinker:
      self.emergency_manual_timer = 50
    if self.lanechange_manual_timer:
      lkas_active = 0
    if self.lanechange_manual_timer > 0:
      self.lanechange_manual_timer -= 1
    if self.emergency_manual_timer > 0:
      self.emergency_manual_timer -= 1

    if abs(CS.out.steeringTorque) > 200 and CS.out.vEgo < LANE_CHANGE_SPEED_MIN:
      self.driver_steering_torque_above = True
    else:
      self.driver_steering_torque_above = False

    if self.driver_steering_torque_above == True:
      self.driver_steering_torque_above_timer -= 1
      if self.driver_steering_torque_above_timer <= 0:
        self.driver_steering_torque_above_timer = 0
    elif self.driver_steering_torque_above == False:
      self.driver_steering_torque_above_timer += 5
      if self.driver_steering_torque_above_timer >= 100:
        self.driver_steering_torque_above_timer = 100

    if not lkas_active:
      apply_steer = 0

    if CS.CP.radarOffCan:
      self.usestockscc = not self.cp_oplongcontrol
    elif (CS.cancel_button_count == 3) and self.cp_oplongcontrol:
      self.usestockscc = not self.usestockscc

    if self.prev_gapButton != CS.cruise_buttons:  # gap change.
      if CS.cruise_buttons == 3:
        self.gapsettingdance -= 1
      if self.gapsettingdance < 1:
        self.gapsettingdance = 4
      self.prev_gapButton = CS.cruise_buttons

    self.apply_steer_last = apply_steer

    sys_warning, sys_state, left_lane_warning, right_lane_warning =\
      process_hud_alert(enabled, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart)

    speed_conv = CV.MS_TO_MPH if CS.is_set_speed_in_mph else CV.MS_TO_KPH

    self.clu11_speed = CS.clu11["CF_Clu_Vanz"]

    enabled_speed = 38 if CS.is_set_speed_in_mph else 55

    if self.clu11_speed > enabled_speed or not lkas_active or CS.out.gearShifter != GearShifter.drive:
      enabled_speed = self.clu11_speed

    self.current_veh_speed = int(CS.out.vEgo * speed_conv)

    self.clu11_cnt = frame % 0x10

    can_sends = []

    self.lfa_available = True if self.lfainFingerprint or self.car_fingerprint in FEATURES["send_lfa_mfa"] else False

    can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled,
                                   left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, self.lfa_available, 0))

    if CS.CP.mdpsHarness:  # send lkas11 bus 1 if mdps
      can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled,
                                   left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, self.lfa_available, 1))

      can_sends.append(create_clu11(self.packer, 1, CS.clu11, Buttons.NONE, enabled_speed, self.clu11_cnt))

    str_log1 = 'TQ={:03.0f}  FR={:03.0f} ST={:03.0f}/{:01.0f}/{:01.0f}  CG={:01.0f}'.format(abs(new_steer), self.timer1.sampleTime(), self.steerMax, self.steerDeltaUp, self.steerDeltaDown, CS.out.cruiseGapSet)
    trace1.printf1('{}  {}'.format(str_log1, self.str_log2))

    if pcm_cancel_cmd and CS.scc12["ACCMode"] != 0 and not CS.out.standstill:
      self.vdiff = 0.
      self.resumebuttoncnt = 0
      can_sends.append(create_clu11(self.packer, CS.CP.sccBus, CS.clu11, Buttons.CANCEL, self.current_veh_speed, self.clu11_cnt))
    elif CS.out.cruiseState.standstill and CS.scc12["ACCMode"] != 0 and CS.vrelative > 0:
      self.vdiff += (CS.vrelative - self.vdiff)
      if (frame - self.lastresumeframe > 5) and (self.vdiff > .1 or CS.lead_distance > 4.5):
        can_sends.append(create_clu11(self.packer, CS.CP.sccBus, CS.clu11, Buttons.RES_ACCEL, self.current_veh_speed, self.resumebuttoncnt))
        self.resumebuttoncnt += 1
        if self.resumebuttoncnt > 5:
          self.lastresumeframe = frame
          self.resumebuttoncnt = 0
    else:
      self.vdiff = 0.
      self.resumebuttoncnt = 0

    if CS.out.vEgo < 2.:
      self.sm.update(0)
      long_control_state = self.sm['controlsState'].longControlState
      self.acc_standstill = True if long_control_state == LongCtrlState.stopping else False
      if self.acc_standstill == True:
        self.acc_standstill_timer += 1
        if self.acc_standstill_timer >= 200:
          self.acc_standstill_timer = 200
      else:
        self.acc_standstill_timer = 0
    else:
      self.acc_standstill = False

    if lead_visible:
      self.lead_visible = True
      self.lead_debounce = 50
    elif self.lead_debounce > 0:
      self.lead_debounce -= 1
    else:
      self.lead_visible = lead_visible

    self.setspeed = set_speed * speed_conv

    if enabled:
      self.sendaccmode = enabled

    if CS.CP.radarDisablePossible:
      self.radarDisableOverlapTimer += 1
      self.radarDisableResetTimer = 0
      if self.radarDisableOverlapTimer >= 30:
        self.radarDisableActivated = True
        if 200 > self.radarDisableOverlapTimer > 36:
          if frame % 41 == 0 or self.radarDisableOverlapTimer == 37:
            can_sends.append(create_scc7d0(b'\x02\x10\x03\x00\x00\x00\x00\x00'))
          elif frame % 43 == 0 or self.radarDisableOverlapTimer == 37:
            can_sends.append(create_scc7d0(b'\x03\x28\x03\x01\x00\x00\x00\x00'))
          elif frame % 19 == 0 or self.radarDisableOverlapTimer == 37:
            can_sends.append(create_scc7d0(b'\x02\x10\x85\x00\x00\x00\x00\x00'))  # this disables RADAR for
      else:
        self.counter_init = False
        can_sends.append(create_scc7d0(b'\x02\x10\x90\x00\x00\x00\x00\x00'))  # this enables RADAR
        can_sends.append(create_scc7d0(b'\x03\x29\x03\x01\x00\x00\x00\x00'))
    elif self.radarDisableActivated:
      can_sends.append(create_scc7d0(b'\x02\x10\x90\x00\x00\x00\x00\x00'))  # this enables RADAR
      can_sends.append(create_scc7d0(b'\x03\x29\x03\x01\x00\x00\x00\x00'))
      self.radarDisableOverlapTimer = 0
      if frame % 50 == 0:
        self.radarDisableResetTimer += 1
        if self.radarDisableResetTimer > 2:
          self.radarDisableActivated = False
          self.counter_init = True
    else:
      self.radarDisableOverlapTimer = 0
      self.radarDisableResetTimer = 0

    if (frame % 50 == 0 or self.radarDisableOverlapTimer == 37) and \
            CS.CP.radarDisablePossible and self.radarDisableOverlapTimer >= 30:
      can_sends.append(create_scc7d0(b'\x02\x3E\x00\x00\x00\x00\x00\x00'))

    if self.lead_visible:
      self.objdiststat = 1 if lead_dist < 25 else 2 if lead_dist < 40 else \
                         3 if lead_dist < 60 else 4 if lead_dist < 80 else 5
    else:
      self.objdiststat = 0

    # send scc to car if longcontrol enabled and SCC not on bus 0 or ont live
    if (CS.CP.sccBus == 2 or not self.usestockscc or self.radarDisableActivated) and self.counter_init:
      if frame % 2 == 0:
        self.scc12cnt += 1
        self.scc12cnt %= 0xF
        self.scc11cnt += 1
        self.scc11cnt %= 0x10
        self.fca11supcnt += 1
        self.fca11supcnt %= 0xF

        if self.fca11alivecnt == 1:
          self.fca11inc = 0
          if self.fca11cnt13 == 3:
            self.fca11maxcnt = 0x9
            self.fca11cnt13 = 0
          else:
            self.fca11maxcnt = 0xD
            self.fca11cnt13 += 1
        else:
          self.fca11inc += 4

        self.fca11alivecnt = self.fca11maxcnt - self.fca11inc

        can_sends.append(create_scc11(self.packer, enabled,
                                      self.setspeed, self.lead_visible, lead_dist, lead_vrel, lead_yrel,
                                      self.gapsettingdance,
                                      CS.out.standstill, CS.scc11,
                                      self.usestockscc, CS.CP.radarOffCan, self.scc11cnt, self.sendaccmode))

        if CS.brake_check == 1 or CS.mainsw_check == 1:
          can_sends.append(create_scc12(self.packer, apply_accel, enabled,
                                      self.acc_standstill, CS.out.gasPressed, 1,
                                      CS.out.stockAeb,
                                      CS.scc12, self.usestockscc, CS.CP.radarOffCan, self.scc12cnt))
        else:
          can_sends.append(create_scc12(self.packer, apply_accel, enabled,
                                      self.acc_standstill, CS.out.gasPressed, CS.out.brakePressed,
                                      CS.out.stockAeb,
                                      CS.scc12, self.usestockscc, CS.CP.radarOffCan, self.scc12cnt))

        can_sends.append(create_scc14(self.packer, enabled, self.usestockscc, CS.out.stockAeb, apply_accel,
                                      CS.scc14, self.objdiststat, CS.out.gasPressed, self.acc_standstill, CS.out.vEgo))
        if CS.CP.fcaBus == -1:
          can_sends.append(create_fca11(self.packer, CS.fca11, self.fca11alivecnt, self.fca11supcnt))

      if frame % 20 == 0:
        can_sends.append(create_scc13(self.packer, CS.scc13))
        if CS.CP.fcaBus == -1:
          can_sends.append(create_fca12(self.packer))
      if frame % 50 == 0:
        can_sends.append(create_scc42a(self.packer))
    else:
      self.counter_init = True
      self.scc12cnt = CS.scc12init["CR_VSM_Alive"]
      self.scc11cnt = CS.scc11init["AliveCounterACC"]
      self.fca11alivecnt = CS.fca11init["CR_FCA_Alive"]
      self.fca11supcnt = CS.fca11init["Supplemental_Counter"]

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.lfa_available:
      can_sends.append(create_lfa_mfa(self.packer, frame, enabled))

    can_sends.append(create_mdps12(self.packer, frame, CS.mdps12))

    return can_sends
