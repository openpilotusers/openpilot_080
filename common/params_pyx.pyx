# distutils: language = c++
# cython: language_level = 3
from libcpp cimport bool
from libcpp.string cimport string
from common.params_pxd cimport Params as c_Params

import os
import threading
from common.basedir import BASEDIR

cdef enum TxType:
  PERSISTENT = 1
  CLEAR_ON_MANAGER_START = 2
  CLEAR_ON_PANDA_DISCONNECT = 3

keys = {
  b"AccessToken": [TxType.CLEAR_ON_MANAGER_START],
  b"AthenadPid": [TxType.PERSISTENT],
  b"CalibrationParams": [TxType.PERSISTENT],
  b"CarBatteryCapacity": [TxType.PERSISTENT],
  b"CarParams": [TxType.CLEAR_ON_MANAGER_START, TxType.CLEAR_ON_PANDA_DISCONNECT],
  b"CarParamsCache": [TxType.CLEAR_ON_MANAGER_START, TxType.CLEAR_ON_PANDA_DISCONNECT],
  b"CarVin": [TxType.CLEAR_ON_MANAGER_START, TxType.CLEAR_ON_PANDA_DISCONNECT],
  b"CommunityFeaturesToggle": [TxType.PERSISTENT],
  b"CompletedTrainingVersion": [TxType.PERSISTENT],
  b"DisablePowerDown": [TxType.PERSISTENT],
  b"DisableUpdates": [TxType.PERSISTENT],
  b"DoUninstall": [TxType.CLEAR_ON_MANAGER_START],
  b"DongleId": [TxType.PERSISTENT],
  b"GitBranch": [TxType.PERSISTENT],
  b"GitCommit": [TxType.PERSISTENT],
  b"GitRemote": [TxType.PERSISTENT],
  b"GithubSshKeys": [TxType.PERSISTENT],
  b"HasAcceptedTerms": [TxType.PERSISTENT],
  b"HasCompletedSetup": [TxType.PERSISTENT],
  b"IsDriverViewEnabled": [TxType.CLEAR_ON_MANAGER_START],
  b"IsLdwEnabled": [TxType.PERSISTENT],
  b"IsGeofenceEnabled": [TxType.PERSISTENT],
  b"IsMetric": [TxType.PERSISTENT],
  b"IsOffroad": [TxType.CLEAR_ON_MANAGER_START],
  b"IsRHD": [TxType.PERSISTENT],
  b"IsTakingSnapshot": [TxType.CLEAR_ON_MANAGER_START],
  b"IsUpdateAvailable": [TxType.CLEAR_ON_MANAGER_START],
  b"IsUploadRawEnabled": [TxType.PERSISTENT],
  b"LastAthenaPingTime": [TxType.PERSISTENT],
  b"LastUpdateTime": [TxType.PERSISTENT],
  b"LastUpdateException": [TxType.PERSISTENT],
  b"LimitSetSpeed": [TxType.PERSISTENT],
  b"LimitSetSpeedNeural": [TxType.PERSISTENT],
  b"LiveParameters": [TxType.PERSISTENT],
  b"OpenpilotEnabledToggle": [TxType.PERSISTENT],
  b"LaneChangeEnabled": [TxType.PERSISTENT],
  b"PandaFirmware": [TxType.CLEAR_ON_MANAGER_START, TxType.CLEAR_ON_PANDA_DISCONNECT],
  b"PandaFirmwareHex": [TxType.CLEAR_ON_MANAGER_START, TxType.CLEAR_ON_PANDA_DISCONNECT],
  b"PandaDongleId": [TxType.CLEAR_ON_MANAGER_START, TxType.CLEAR_ON_PANDA_DISCONNECT],
  b"Passive": [TxType.PERSISTENT],
  b"RecordFront": [TxType.PERSISTENT],
  b"ReleaseNotes": [TxType.PERSISTENT],
  b"ShouldDoUpdate": [TxType.CLEAR_ON_MANAGER_START],
  b"SubscriberInfo": [TxType.PERSISTENT],
  b"TermsVersion": [TxType.PERSISTENT],
  b"TrainingVersion": [TxType.PERSISTENT],
  b"UpdateAvailable": [TxType.CLEAR_ON_MANAGER_START],
  b"UpdateFailedCount": [TxType.CLEAR_ON_MANAGER_START],
  b"Version": [TxType.PERSISTENT],
  b"Offroad_ChargeDisabled": [TxType.CLEAR_ON_MANAGER_START, TxType.CLEAR_ON_PANDA_DISCONNECT],
  b"Offroad_ConnectivityNeeded": [TxType.CLEAR_ON_MANAGER_START],
  b"Offroad_ConnectivityNeededPrompt": [TxType.CLEAR_ON_MANAGER_START],
  b"Offroad_TemperatureTooHigh": [TxType.CLEAR_ON_MANAGER_START],
  b"Offroad_PandaFirmwareMismatch": [TxType.CLEAR_ON_MANAGER_START, TxType.CLEAR_ON_PANDA_DISCONNECT],
  b"Offroad_InvalidTime": [TxType.CLEAR_ON_MANAGER_START],
  b"Offroad_IsTakingSnapshot": [TxType.CLEAR_ON_MANAGER_START],
  b"Offroad_NeosUpdate": [TxType.CLEAR_ON_MANAGER_START],
  b"Offroad_UpdateFailed": [TxType.CLEAR_ON_MANAGER_START],
  b"IsOpenpilotViewEnabled": [TxType.CLEAR_ON_MANAGER_START],
  b"OpkrAutoShutdown": [TxType.PERSISTENT],
  b"OpkrAutoScreenOff": [TxType.PERSISTENT],
  b"OpkrUIBrightness": [TxType.PERSISTENT],
  b"OpkrEnableDriverMonitoring": [TxType.PERSISTENT],
  b"OpkrEnableLogger": [TxType.PERSISTENT],
  b"OpkrEnableGetoffAlert": [TxType.PERSISTENT],
  b"OpkrAutoResume": [TxType.PERSISTENT],
  b"OpkrVariableCruise": [TxType.PERSISTENT],
  b"OpkrLaneChangeSpeed": [TxType.PERSISTENT],
  b"OpkrAutoLaneChangeDelay": [TxType.PERSISTENT],
  b"OpkrSteerAngleCorrection": [TxType.PERSISTENT],
  b"PutPrebuiltOn": [TxType.PERSISTENT],
  b"FingerprintIssuedFix": [TxType.PERSISTENT],
  b"LdwsCarFix": [TxType.PERSISTENT],
  b"LateralControlMethod": [TxType.PERSISTENT],
  b"CruiseStatemodeSelInit": [TxType.PERSISTENT],
  b"OuterLoopGain": [TxType.PERSISTENT],
  b"InnerLoopGain": [TxType.PERSISTENT],
  b"TimeConstant": [TxType.PERSISTENT],
  b"ActuatorEffectiveness": [TxType.PERSISTENT],
  b"Scale": [TxType.PERSISTENT],
  b"LqrKi": [TxType.PERSISTENT],
  b"DcGain": [TxType.PERSISTENT],
  b"IgnoreZone": [TxType.PERSISTENT],
  b"PidKp": [TxType.PERSISTENT],
  b"PidKi": [TxType.PERSISTENT],
  b"PidKf": [TxType.PERSISTENT],
  b"CameraOffsetAdj": [TxType.PERSISTENT],
  b"SteerRatioAdj": [TxType.PERSISTENT],
  b"SteerActuatorDelayAdj": [TxType.PERSISTENT],
  b"SteerRateCostAdj": [TxType.PERSISTENT],
  b"SteerLimitTimerAdj": [TxType.PERSISTENT],
  b"TireStiffnessFactorAdj": [TxType.PERSISTENT],
  b"SteerMaxAdj": [TxType.PERSISTENT],
  b"SteerMaxBaseAdj": [TxType.PERSISTENT],
  b"SteerDeltaUpAdj": [TxType.PERSISTENT],
  b"SteerDeltaDownAdj": [TxType.PERSISTENT],
  b"SteerMaxvAdj": [TxType.PERSISTENT],
  b"OpkrBatteryChargingControl": [TxType.PERSISTENT],
  b"OpkrBatteryChargingMin": [TxType.PERSISTENT],
  b"OpkrBatteryChargingMax": [TxType.PERSISTENT],
  b"OpkrUiOpen": [TxType.PERSISTENT],
  b"OpkrDriveOpen": [TxType.PERSISTENT],
  b"OpkrTuneOpen": [TxType.PERSISTENT],
  b"OpkrControlOpen": [TxType.PERSISTENT],
  b"LeftCurvOffsetAdj": [TxType.PERSISTENT],
  b"RightCurvOffsetAdj": [TxType.PERSISTENT],
  b"DebugUi1": [TxType.PERSISTENT],
  b"DebugUi2": [TxType.PERSISTENT],
  b"OpkrBlindSpotDetect": [TxType.PERSISTENT],
  b"OpkrMaxAngleLimit": [TxType.PERSISTENT],
  b"OpkrAutoResumeOption": [TxType.PERSISTENT],
  b"OpkrAngleOffsetSelect": [TxType.PERSISTENT],
  b"OpkrSpeedLimitOffset": [TxType.PERSISTENT],
  b"LimitSetSpeedCamera": [TxType.PERSISTENT],
  b"OpkrLiveSteerRatio": [TxType.PERSISTENT],
  b"OpkrVariableSteerMax": [TxType.PERSISTENT],
  b"OpkrVariableSteerDelta": [TxType.PERSISTENT],
  b"FingerprintTwoSet": [TxType.PERSISTENT],
  b"OpkrVariableCruiseProfile": [TxType.PERSISTENT],
}

def ensure_bytes(v):
  if isinstance(v, str):
    return v.encode()
  else:
    return v


class UnknownKeyName(Exception):
  pass

cdef class Params:
  cdef c_Params* p

  def __cinit__(self, d=None, bool persistent_params=False):
    if d is None:
      self.p = new c_Params(persistent_params)
    else:
      self.p = new c_Params(<string>d.encode())

  def __dealloc__(self):
    del self.p

  def clear_all(self, tx_type=None):
    for key in keys:
      if tx_type is None or tx_type in keys[key]:
        self.delete(key)

  def manager_start(self):
    self.clear_all(TxType.CLEAR_ON_MANAGER_START)

  def panda_disconnect(self):
    self.clear_all(TxType.CLEAR_ON_PANDA_DISCONNECT)

  def get(self, key, block=False, encoding=None):
    key = ensure_bytes(key)

    if key not in keys:
      raise UnknownKeyName(key)

    cdef string k = key
    cdef bool b = block

    cdef string val
    with nogil:
      val = self.p.get(k, b)

    if val == b"":
      if block:
        # If we got no value while running in blocked mode
        # it means we got an interrupt while waiting
        raise KeyboardInterrupt
      else:
        return None

    if encoding is not None:
      return val.decode(encoding)
    else:
      return val

  def put(self, key, dat):
    """
    Warning: This function blocks until the param is written to disk!
    In very rare cases this can take over a second, and your code will hang.
    Use the put_nonblocking helper function in time sensitive code, but
    in general try to avoid writing params as much as possible.
    """
    key = ensure_bytes(key)
    dat = ensure_bytes(dat)

    if key not in keys:
      raise UnknownKeyName(key)

    self.p.write_db_value(key, dat)

  def delete(self, key):
    key = ensure_bytes(key)
    self.p.delete_db_value(key)


def put_nonblocking(key, val, d=None):
  def f(key, val):
    params = Params(d)
    params.put(key, val)

  t = threading.Thread(target=f, args=(key, val))
  t.start()
  return t
