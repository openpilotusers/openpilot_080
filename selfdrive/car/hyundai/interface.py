#!/usr/bin/env python3
from cereal import car
from common.params import Params
from selfdrive.config import Conversions as CV
from selfdrive.car.hyundai.values import CAR, Buttons
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase

EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.buttonEvents = []
    self.cp2 = self.CS.get_can2_parser(CP)
    self.visiononlyWarning = False
    self.belowspeeddingtimer = 0.

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 1.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "hyundai"
    ret.safetyModel = car.CarParams.SafetyModel.hyundai

    params = Params()
    PidKp = int(params.get('PidKp')) * 0.01
    PidKi = int(params.get('PidKi')) * 0.001
    PidKf = int(params.get('PidKf')) * 0.00001
    InnerLoopGain = int(params.get('InnerLoopGain')) * 0.1
    OuterLoopGain = int(params.get('OuterLoopGain')) * 0.1
    TimeConstant = int(params.get('TimeConstant')) * 0.1
    ActuatorEffectiveness = int(params.get('ActuatorEffectiveness')) * 0.1
    Scale = int(params.get('Scale')) * 1.0
    LqrKi = int(params.get('LqrKi')) * 0.001
    DcGain = int(params.get('DcGain')) * 0.0001
    LqrSteerMaxV = int(params.get('SteerMaxvAdj')) * 0.1

    # Most Hyundai car ports are community features for now
    ret.communityFeature = False

    tire_stiffness_factor = int(params.get('TireStiffnessFactorAdj')) * 0.01
    ret.steerActuatorDelay = int(params.get('SteerActuatorDelayAdj')) * 0.01
    ret.steerRateCost = int(params.get('SteerRateCostAdj')) * 0.01
    ret.steerLimitTimer = int(params.get('SteerLimitTimerAdj')) * 0.01
    ret.steerRatio = int(params.get('SteerRatioAdj')) * 0.1

    if candidate == CAR.GENESIS:
      ret.mass = 1900. + STD_CARGO_KG
      ret.wheelbase = 3.01
    elif candidate == CAR.GENESIS_G70:
      ret.mass = 1640. + STD_CARGO_KG
      ret.wheelbase = 2.84
    elif candidate == CAR.GENESIS_G80:
      ret.mass = 1855. + STD_CARGO_KG
      ret.wheelbase = 3.01
    elif candidate == CAR.GENESIS_G90:
      ret.mass = 2200
      ret.wheelbase = 3.15
    elif candidate in [CAR.SANTA_FE]:
      ret.mass = 1694 + STD_CARGO_KG
      ret.wheelbase = 2.766
    elif candidate in [CAR.SONATA, CAR.SONATA_HEV]:
      ret.mass = 1513. + STD_CARGO_KG
      ret.wheelbase = 2.84
    elif candidate in [CAR.SONATA19, CAR.SONATA19_HEV]:
      ret.mass = 4497. * CV.LB_TO_KG
      ret.wheelbase = 2.804
    elif candidate == CAR.PALISADE:
      ret.mass = 1999. + STD_CARGO_KG
      ret.wheelbase = 2.90
    elif candidate in [CAR.ELANTRA, CAR.ELANTRA_GT_I30]:
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
    elif candidate == CAR.KONA:
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
    elif candidate in [CAR.KONA_HEV, CAR.KONA_EV]:
      ret.mass = 1685. + STD_CARGO_KG
      ret.wheelbase = 2.7
    elif candidate in [CAR.IONIQ_HEV, CAR.IONIQ_EV]:
      ret.mass = 1490. + STD_CARGO_KG   #weight per hyundai site https://www.hyundaiusa.com/ioniq-electric/specifications.aspx
      ret.wheelbase = 2.7
    elif candidate in [CAR.GRANDEUR, CAR.GRANDEUR_HEV]:
      ret.mass = 1640. + STD_CARGO_KG
      ret.wheelbase = 2.845
    elif candidate == CAR.VELOSTER:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
    elif candidate == CAR.NEXO:
      ret.mass = 1885. + STD_CARGO_KG
      ret.wheelbase = 2.79
    # kia
    elif candidate == CAR.SORENTO:
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
    elif candidate in [CAR.OPTIMA, CAR.OPTIMA_HEV]:
      ret.wheelbase = 2.80
      ret.mass = 1595. + STD_CARGO_KG
    elif candidate == CAR.STINGER:
      ret.mass = 1825.0 + STD_CARGO_KG
      ret.wheelbase = 2.906 # https://www.kia.com/us/en/stinger/specs
    elif candidate == CAR.FORTE:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
    elif candidate == CAR.CEED:
      ret.mass = 1350. + STD_CARGO_KG
      ret.wheelbase = 2.65
    elif candidate == CAR.SPORTAGE:
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
    elif candidate in [CAR.NIRO_HEV, CAR.NIRO_EV]:
      ret.mass = 1737. + STD_CARGO_KG
      ret.wheelbase = 2.7
    elif candidate in [CAR.CADENZA, CAR.CADENZA_HEV]:
      ret.mass = 1640. + STD_CARGO_KG
      ret.wheelbase = 2.845

    if int(params.get('LateralControlMethod')) == 0:
      ret.lateralTuning.pid.kf = PidKf
      ret.lateralTuning.pid.kpBP = [0., 9.]
      ret.lateralTuning.pid.kpV = [0.1, PidKp]
      ret.lateralTuning.pid.kiBP = [0., 9.]
      ret.lateralTuning.pid.kiV = [0.01, PidKi]
    elif int(params.get('LateralControlMethod')) == 1:
      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.innerLoopGain = InnerLoopGain
      ret.lateralTuning.indi.outerLoopGain = OuterLoopGain
      ret.lateralTuning.indi.timeConstant = TimeConstant
      ret.lateralTuning.indi.actuatorEffectiveness = ActuatorEffectiveness
    elif int(params.get('LateralControlMethod')) == 2:
      ret.lateralTuning.init('lqr')
      ret.lateralTuning.lqr.scale = Scale
      ret.lateralTuning.lqr.ki = LqrKi
      ret.lateralTuning.lqr.a = [0., 1., -0.22619643, 1.21822268]
      ret.lateralTuning.lqr.b = [-1.92006585e-04, 3.95603032e-05]
      ret.lateralTuning.lqr.c = [1., 0.]
      ret.lateralTuning.lqr.k = [-110., 451.]
      ret.lateralTuning.lqr.l = [0.33, 0.318]
      ret.lateralTuning.lqr.dcGain = DcGain


    ret.longitudinalTuning.kpBP = [0., 1., 10., 35.]
    ret.longitudinalTuning.kpV = [0.8, 0.7, 0.6, 0.55]
    ret.longitudinalTuning.kiBP = [0., 1., 15., 35.]
    ret.longitudinalTuning.kiV = [0.4, 0.3, 0.2, 0.1]
    #ret.longitudinalTuning.kfBP = [0., 5.]
    #ret.longitudinalTuning.kfV = [1., 1.]
    ret.longitudinalTuning.deadzoneBP = [0.0, 0.5]
    ret.longitudinalTuning.deadzoneV = [0.00, 0.00]
  
    ret.gasMaxBP = [0., 1., 1.1, 15., 40.]
    ret.gasMaxV = [2., 2., 2., 2., 2.]
    ret.brakeMaxBP = [0., 5.]
    ret.brakeMaxV = [4.0, 4.0]

    ret.steerMaxV = [LqrSteerMaxV]
    ret.steerMaxBP = [0.]


    # these cars require a special panda safety mode due to missing counters and checksums in the messages

    ret.mdpsHarness = params.get('CommunityFeaturesToggle') == b'0'
    ret.sasBus = 0 if (688 in fingerprint[0] or not ret.mdpsHarness) else 1
    ret.fcaBus = 0 if 909 in fingerprint[0] else 2 if 909 in fingerprint[2] else -1
    ret.bsmAvailable = True if 1419 in fingerprint[0] else False
    ret.lfaAvailable = True if 1157 in fingerprint[2] else False
    ret.lvrAvailable = True if 871 in fingerprint[0] else False
    ret.evgearAvailable = True if 882 in fingerprint[0] else False
    ret.emsAvailable = True if 608 and 809 in fingerprint[0] else False

    if True:
      ret.sccBus = 2 if 1057 in fingerprint[2] and False else 0 if 1057 in fingerprint[0] else -1
    else:
      ret.sccBus = -1

    ret.radarOffCan = (ret.sccBus == -1)
    #ret.radarTimeStep = 0.1

    ret.openpilotLongitudinalControl = True and not (ret.sccBus == 0)

    ret.safetyModel = car.CarParams.SafetyModel.hyundaiLegacy

    if ret.mdpsHarness:
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiCommunity
    if ret.radarOffCan or (ret.sccBus == 2) or True:
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiCommunityNonscc
    if ret.mdpsHarness:
      ret.minSteerSpeed = 0.

    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableCamera = True

    ret.radarDisablePossible = True

    ret.enableCruise = False and ret.sccBus == 0

    if ret.radarDisablePossible:
      ret.openpilotLongitudinalControl = True
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiCommunityNonscc # todo based on toggle
      ret.sccBus = -1
      ret.enableCruise = False
      ret.radarOffCan = True
      if ret.fcaBus == 0:
        ret.fcaBus = -1

    ret.standStill = False
    return ret

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp2.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)
    ret = self.CS.update(self.cp, self.cp2, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp2.can_valid and self.cp_cam.can_valid
    
    events = self.create_common_events(ret)

    # speeds
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo > (self.CP.minSteerSpeed + .84) or not self.CC.enabled:
      self.low_speed_alert = False
      self.belowspeeddingtimer = 0.

    if self.CP.sccBus == 2:
      self.CP.enableCruise = self.CC.usestockscc

    #if self.CS.brakeHold and not self.CC.usestockscc:
    #  events.add(EventName.brakeHold)
    if self.CS.parkBrake and not self.CC.usestockscc:
      events.add(EventName.parkBrake)
    if self.CS.brakeUnavailable and not self.CC.usestockscc:
      events.add(EventName.brakeUnavailable)
    if self.CC.lanechange_manual_timer and ret.vEgo > 0.3:
      events.add(EventName.laneChangeManual)
    if self.CC.emergency_manual_timer:
      events.add(EventName.emgButtonManual)
    if self.CC.acc_standstill_timer >= 200:
      #events.add(EventName.standStill)
      self.CP.standStill = True
    else:
      self.CP.standStill = False

    buttonEvents = []
    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.pressed = self.CS.cruise_buttons != 0 
      but = self.CS.cruise_buttons
      if but == Buttons.RES_ACCEL:
        be.type = ButtonType.accelCruise
      elif but == Buttons.SET_DECEL:
        be.type = ButtonType.decelCruise
      elif but == Buttons.GAP_DIST:
        be.type = ButtonType.gapAdjustCruise
      elif but == Buttons.CANCEL:
        be.type = ButtonType.cancel
      else:
        be.type = ButtonType.unknown
      buttonEvents.append(be)
      self.buttonEvents = buttonEvents

    if self.CS.cruise_main_button != self.CS.prev_cruise_main_button:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.altButton3
      be.pressed = bool(self.CS.cruise_main_button)
      buttonEvents.append(be)
      self.buttonEvents = buttonEvents

    ret.buttonEvents = self.buttonEvents

    # handle button press
    if not self.CP.enableCruise:
      for b in self.buttonEvents:
        if b.type == ButtonType.decelCruise and b.pressed \
                and (not ret.brakePressed or ret.standstill):
          events.add(EventName.buttonEnable)
          events.add(EventName.pcmEnable)
        if b.type == ButtonType.accelCruise and b.pressed \
                and ((self.CC.setspeed > self.CC.clu11_speed - 2) or ret.standstill or self.CC.usestockscc):
          events.add(EventName.buttonEnable)
          events.add(EventName.pcmEnable)
        if b.type == ButtonType.cancel and b.pressed:
          events.add(EventName.buttonCancel)
          events.add(EventName.pcmDisable)
        #if b.type == ButtonType.altButton3 and b.pressed:
          #events.add(EventName.buttonCancel)
          #events.add(EventName.pcmDisable)

    ret.events = events.to_msg()
    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c, sm):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators,
                               c.cruiseControl.cancel, c.hudControl.visualAlert, c.hudControl.leftLaneVisible,
                               c.hudControl.rightLaneVisible, c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart,
                               c.hudControl.setSpeed, c.hudControl.leadVisible, c.hudControl.leadDistance,
                               c.hudControl.leadvRel, c.hudControl.leadyRel, sm)
    self.frame += 1
    return can_sends
