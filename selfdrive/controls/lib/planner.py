#!/usr/bin/env python3
import math
from datetime import datetime
import time
import numpy as np
from common.params import Params
from common.numpy_fast import interp

import cereal.messaging as messaging
from cereal import car
from common.realtime import sec_since_boot
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.speed_smoother import speed_smoother
from selfdrive.controls.lib.longcontrol import LongCtrlState, MIN_CAN_SPEED
from selfdrive.controls.lib.fcw import FCWChecker
from selfdrive.controls.lib.long_mpc import LongitudinalMpc
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX
from common.op_params import opParams
op_params = opParams()
osm = op_params.get('osm')
smart_speed = op_params.get('smart_speed')
smart_speed_max_vego = op_params.get('smart_speed_max_vego')
offset_limit = op_params.get('offset_limit')
default_brake_distance = op_params.get('default_brake_distance')
eco_mode = op_params.get('eco_mode')
curvature_factor = opParams().get('curvature_factor')
MAX_SPEED = 255.0
NO_CURVATURE_SPEED = 90.0

LON_MPC_STEP = 0.2  # first step is 0.2s
MAX_SPEED_ERROR = 2.0
AWARENESS_DECEL = -0.2     # car smoothly decel at .2m/s^2 when user is distracted

# lookup tables VS speed to determine min and max accels in cruise
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MIN_V = [-1.0, -.8, -.67, -.5, -.30]
_A_CRUISE_MIN_BP = [   0., 5.,  10., 20.,  40.]

# need fast accel at very low speed for stop and go
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MAX_V = [1.2, 1.2, 0.65, .4]
_A_CRUISE_MAX_V_FOLLOWING = [1.6, 1.6, 0.65, .4]
_A_CRUISE_MAX_BP = [0.,  6.4, 22.5, 40.]

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


def calc_cruise_accel_limits(v_ego, following):
  a_cruise_min = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V)

  if following:
    a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_FOLLOWING)
  else:
    a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V)
  return np.vstack([a_cruise_min, a_cruise_max])


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego**2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max**2 - a_y**2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class Planner():
  def __init__(self, CP):
    self.CP = CP
    self.op_params = opParams()

    self.mpc1 = LongitudinalMpc(1)
    self.mpc2 = LongitudinalMpc(2)

    self.v_acc_start = 0.0
    self.a_acc_start = 0.0

    self.v_acc = 0.0
    self.v_acc_future = 0.0
    self.a_acc = 0.0
    self.v_cruise = 0.0
    self.a_cruise = 0.0
    self.v_model = 0.0
    self.a_model = 0.0
    self.osm = True

    self.longitudinalPlanSource = 'cruise'
    self.fcw_checker = FCWChecker()
    self.path_x = np.arange(192)

    self.params = Params()
    self.first_loop = True
    self.offset = 0
    self.last_time = 0
    self.speed_ahead_distance_prev = 1000

  def choose_solution(self, v_cruise_setpoint, enabled):
    if enabled:
      solutions = {'cruise': self.v_cruise}
      if self.mpc1.prev_lead_status:
        solutions['mpc1'] = self.mpc1.v_mpc
      if self.mpc2.prev_lead_status:
        solutions['mpc2'] = self.mpc2.v_mpc

      slowest = min(solutions, key=solutions.get)

      self.longitudinalPlanSource = slowest
      # Choose lowest of MPC and cruise
      if slowest == 'mpc1':
        self.v_acc = self.mpc1.v_mpc
        self.a_acc = self.mpc1.a_mpc
      elif slowest == 'mpc2':
        self.v_acc = self.mpc2.v_mpc
        self.a_acc = self.mpc2.a_mpc
      elif slowest == 'cruise':
        self.v_acc = self.v_cruise
        self.a_acc = self.a_cruise

    self.v_acc_future = min([self.mpc1.v_mpc_future, self.mpc2.v_mpc_future, v_cruise_setpoint])

  def update(self, sm, pm, CP, VM, PP):
    """Gets called when new radarState is available"""
    cur_time = sec_since_boot()

    # we read offset value every 5 seconds
    fixed_offset = 0.0
    fixed_offset = op_params.get('speed_offset')
    if self.last_time > 5:
      try:
        #self.offset = int(self.params.get("SpeedLimitOffset", encoding='utf8'))
        self.offset = 0
      except (TypeError,ValueError):
        #self.params.delete("SpeedLimitOffset")
        self.offset = 0
      self.osm = self.params.get("LimitSetSpeed", encoding='utf8') == "1"
      self.last_time = 0
    self.last_time = self.last_time + 1

    v_ego = sm['carState'].vEgo

    long_control_state = sm['controlsState'].longControlState
    v_cruise_kph = sm['controlsState'].vCruise
    force_slow_decel = sm['controlsState'].forceDecel

    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise_setpoint = v_cruise_kph * CV.KPH_TO_MS

    lead_1 = sm['radarState'].leadOne
    lead_2 = sm['radarState'].leadTwo

    enabled = (long_control_state == LongCtrlState.pid) or (long_control_state == LongCtrlState.stopping)
    following = lead_1.status and lead_1.dRel < 45.0 and lead_1.vLeadK > v_ego and lead_1.aLeadK > 0.0

    speed_ahead_distance = default_brake_distance
    v_speedlimit = NO_CURVATURE_SPEED
    v_curvature_map = NO_CURVATURE_SPEED
    v_speedlimit_ahead = 0
    now = datetime.now()
    try:
      if sm['liveMapData'].speedLimitValid and osm and self.osm and (sm['liveMapData'].lastGps.timestamp -time.mktime(now.timetuple()) * 1000) < 10000 and (smart_speed or smart_speed_max_vego > v_ego):
        speed_limit = sm['liveMapData'].speedLimit
        if speed_limit is not None:
          v_speedlimit = speed_limit
          # offset is in percentage,.
          if v_ego > offset_limit:
            v_speedlimit = v_speedlimit * (1. + self.offset/100.0)
          if v_speedlimit > fixed_offset:
            v_speedlimit = v_speedlimit + fixed_offset
      else:
        speed_limit = None
      if sm['liveMapData'].speedLimitAheadValid and osm and self.osm and sm['liveMapData'].speedLimitAheadDistance < speed_ahead_distance and (sm['liveMapData'].lastGps.timestamp -time.mktime(now.timetuple()) * 1000) < 10000 and (smart_speed or smart_speed_max_vego > v_ego):
        distanceatlowlimit = 50
        if sm['liveMapData'].speedLimitAhead < 21/3.6:
          distanceatlowlimit = speed_ahead_distance = (v_ego - sm['liveMapData'].speedLimitAhead)*3.6*2
          if distanceatlowlimit < 50:
            distanceatlowlimit = 0
          distanceatlowlimit = min(distanceatlowlimit,100)
          speed_ahead_distance = (v_ego - sm['liveMapData'].speedLimitAhead)*3.6*5
          speed_ahead_distance = min(speed_ahead_distance,300)
          speed_ahead_distance = max(speed_ahead_distance,50)
        if speed_limit is not None and sm['liveMapData'].speedLimitAheadDistance > distanceatlowlimit and v_ego + 3 < sm['liveMapData'].speedLimitAhead + (speed_limit - sm['liveMapData'].speedLimitAhead)*sm['liveMapData'].speedLimitAheadDistance/speed_ahead_distance:
          speed_limit_ahead = sm['liveMapData'].speedLimitAhead + (speed_limit - sm['liveMapData'].speedLimitAhead)*(sm['liveMapData'].speedLimitAheadDistance - distanceatlowlimit)/(speed_ahead_distance - distanceatlowlimit)
        else:
          speed_limit_ahead = sm['liveMapData'].speedLimitAhead
      if sm['liveMapData'].curvatureValid and sm['liveMapData'].distToTurn < speed_ahead_distance and osm and self.osm and (sm['liveMapData'].lastGps.timestamp -time.mktime(now.timetuple()) * 1000) < 10000:
        curvature = abs(sm['liveMapData'].curvature)
        radius = 1/max(1e-4, curvature) * curvature_factor
        if radius > 500:
          c=0.9 # 0.9 at 1000m = 108 kph
        elif radius > 250:
          c = 3.5-13/2500*radius # 2.2 at 250m 84 kph
        else:
          c= 3.0 - 2/625 *radius # 3.0 at 15m 24 kph
        v_curvature_map = math.sqrt(c*radius)
        v_curvature_map = min(NO_CURVATURE_SPEED, v_curvature_map)
    except KeyError:
      pass

    decel_for_turn = bool(v_curvature_map < min([v_cruise_setpoint, v_speedlimit, v_ego + 1.]))

    # Calculate speed for normal cruise control
    if enabled and not self.first_loop and not sm['carState'].brakePressed and not sm['carState'].gasPressed:
      accel_limits = [float(x) for x in calc_cruise_accel_limits(v_ego, following)]
      jerk_limits = [min(-0.1, accel_limits[0]), max(0.1, accel_limits[1])]  # TODO: make a separate lookup for jerk tuning
      accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngle, accel_limits, self.CP)

      if force_slow_decel:
        # if required so, force a smooth deceleration
        accel_limits_turns[1] = min(accel_limits_turns[1], AWARENESS_DECEL)
        accel_limits_turns[0] = min(accel_limits_turns[0], accel_limits_turns[1])

      if decel_for_turn and sm['liveMapData'].distToTurn < speed_ahead_distance and not following:
        time_to_turn = max(1.0, sm['liveMapData'].distToTurn / max((v_ego + v_curvature_map)/2, 1.))
        required_decel = min(0, (v_curvature_map - v_ego) / time_to_turn)
        accel_limits[0] = max(accel_limits[0], required_decel)
      #if v_speedlimit_ahead < v_speedlimit and v_ego > v_speedlimit_ahead and sm['liveMapData'].speedLimitAheadDistance > 1.0 and not following:
      #  required_decel = min(0, (v_speedlimit_ahead*v_speedlimit_ahead - v_ego*v_ego)/(sm['liveMapData'].speedLimitAheadDistance*2))
      #  required_decel = max(required_decel, -3.0)
      #  decel_for_turn = True
      #  accel_limits[0] = required_decel
      #  accel_limits[1] = required_decel
      #  self.a_acc_start = required_decel
      #  v_speedlimit_ahead = v_ego

      if (v_ego*3.6) <= 50:
        if sm['liveMapData'].speedLimitAhead and sm['liveMapData'].speedLimitAheadDistance < (v_ego*3.6*2.5):
          if self.speed_ahead_distance_prev > sm['liveMapData'].speedLimitAheadDistance:
            self.speed_ahead_distance_prev = sm['liveMapData'].speedLimitAheadDistance
            v_speedlimit_ahead = sm['liveMapData'].speedLimitAhead
          else:
            v_speedlimit_ahead = 0
        elif not sm['liveMapData'].speedLimitAhead:
          self.speed_ahead_distance_prev = 1000
      elif (v_ego*3.6) <= 70:
        if sm['liveMapData'].speedLimitAhead and sm['liveMapData'].speedLimitAheadDistance < (v_ego*3.6*3.5):
          if self.speed_ahead_distance_prev > sm['liveMapData'].speedLimitAheadDistance:
            self.speed_ahead_distance_prev = sm['liveMapData'].speedLimitAheadDistance
            v_speedlimit_ahead = sm['liveMapData'].speedLimitAhead
          else:
            v_speedlimit_ahead = 0
        elif not sm['liveMapData'].speedLimitAhead:
          self.speed_ahead_distance_prev = 1000
      elif (v_ego*3.6) > 70:
        if sm['liveMapData'].speedLimitAhead and sm['liveMapData'].speedLimitAheadDistance < (v_ego*3.6*4.5):
          if self.speed_ahead_distance_prev > sm['liveMapData'].speedLimitAheadDistance:
            self.speed_ahead_distance_prev = sm['liveMapData'].speedLimitAheadDistance
            v_speedlimit_ahead = sm['liveMapData'].speedLimitAhead
          else:
            v_speedlimit_ahead = 0
        elif not sm['liveMapData'].speedLimitAhead:
          self.speed_ahead_distance_prev = 1000

      #v_cruise_setpoint = min([v_cruise_setpoint, v_curvature_map, v_speedlimit, v_speedlimit_ahead])
      v_cruise_setpoint = min([v_cruise_setpoint, v_curvature_map, v_speedlimit])

      self.v_cruise, self.a_cruise = speed_smoother(self.v_acc_start, self.a_acc_start,
                                                    v_cruise_setpoint,
                                                    accel_limits_turns[1], accel_limits_turns[0],
                                                    jerk_limits[1], jerk_limits[0],
                                                    LON_MPC_STEP)

      # cruise speed can't be negative even is user is distracted
      self.v_cruise = max(self.v_cruise, 0.)
    else:
      starting = long_control_state == LongCtrlState.starting
      a_ego = min(sm['carState'].aEgo, 0.0)
      reset_speed = MIN_CAN_SPEED if starting else v_ego
      reset_accel = self.CP.startAccel if starting else a_ego
      self.v_acc = reset_speed
      self.a_acc = reset_accel
      self.v_acc_start = reset_speed
      self.a_acc_start = reset_accel
      self.v_cruise = reset_speed
      self.a_cruise = reset_accel

    self.mpc1.set_cur_state(self.v_acc_start, self.a_acc_start)
    self.mpc2.set_cur_state(self.v_acc_start, self.a_acc_start)

    self.mpc1.update(pm, sm['carState'], lead_1)
    self.mpc2.update(pm, sm['carState'], lead_2)

    self.choose_solution(v_cruise_setpoint, enabled)

    # determine fcw
    if self.mpc1.new_lead:
      self.fcw_checker.reset_lead(cur_time)

    blinkers = sm['carState'].leftBlinker or sm['carState'].rightBlinker
    fcw = self.fcw_checker.update(self.mpc1.mpc_solution, cur_time,
                                  sm['controlsState'].active,
                                  v_ego, sm['carState'].aEgo,
                                  lead_1.dRel, lead_1.vLead, lead_1.aLeadK,
                                  lead_1.yRel, lead_1.vLat,
                                  lead_1.fcw, blinkers) and not sm['carState'].brakePressed
    if fcw:
      cloudlog.info("FCW triggered %s", self.fcw_checker.counters)

    radar_dead = not sm.alive['radarState']

    radar_errors = list(sm['radarState'].radarErrors)
    radar_fault = car.RadarData.Error.fault in radar_errors
    radar_can_error = car.RadarData.Error.canError in radar_errors

    # **** send the plan ****
    plan_send = messaging.new_message('plan')

    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState', 'radarState'])

    plan_send.plan.mdMonoTime = sm.logMonoTime['model']
    plan_send.plan.radarStateMonoTime = sm.logMonoTime['radarState']

    # longitudal plan
    plan_send.plan.vCruise = float(self.v_cruise)
    plan_send.plan.aCruise = float(self.a_cruise)
    plan_send.plan.vStart = float(self.v_acc_start)
    plan_send.plan.aStart = float(self.a_acc_start)
    plan_send.plan.vTarget = float(self.v_acc)
    plan_send.plan.aTarget = float(self.a_acc)
    plan_send.plan.vTargetFuture = float(self.v_acc_future)
    plan_send.plan.hasLead = self.mpc1.prev_lead_status
    plan_send.plan.longitudinalPlanSource = self.longitudinalPlanSource

    plan_send.plan.vCurvature = float(v_curvature_map)
    plan_send.plan.decelForTurn = bool(decel_for_turn)
    plan_send.plan.mapValid = True

    radar_valid = not (radar_dead or radar_fault)
    plan_send.plan.radarValid = bool(radar_valid)
    plan_send.plan.radarCanError = bool(radar_can_error)

    plan_send.plan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.rcv_time['radarState']

    # Send out fcw
    plan_send.plan.fcw = fcw

    # Send radarstate(dRel, vRel, yRel)
    plan_send.plan.dRel1 = lead_1.dRel
    plan_send.plan.yRel1 = lead_1.yRel
    plan_send.plan.vRel1 = lead_1.vRel
    plan_send.plan.dRel2 = lead_2.dRel
    plan_send.plan.yRel2 = lead_2.yRel
    plan_send.plan.vRel2 = lead_2.vRel
    plan_send.plan.status2 = lead_2.status
    plan_send.plan.targetSpeed = v_cruise_setpoint * CV.MS_TO_KPH
    plan_send.plan.targetSpeedCamera = v_speedlimit_ahead * CV.MS_TO_KPH

    pm.send('plan', plan_send)

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_acc_sol = self.a_acc_start + (CP.radarTimeStep / LON_MPC_STEP) * (self.a_acc - self.a_acc_start)
    v_acc_sol = self.v_acc_start + CP.radarTimeStep * (a_acc_sol + self.a_acc_start) / 2.0
    self.v_acc_start = v_acc_sol
    self.a_acc_start = a_acc_sol

    self.first_loop = False
