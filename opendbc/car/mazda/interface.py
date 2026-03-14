#!/usr/bin/env python3
from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.radar_interface import RadarInterface
from opendbc.car.mazda.values import CAR, DBC, LKAS_LIMITS, MAZDA_RADAR_SESSION_ADDR, MazdaFlags


PROGRAMMING_SESSION_REQUEST = b"\x10\x02"
PROGRAMMING_SESSION_RESPONSE = b"\x50\x02"
DEFAULT_SESSION_REQUEST = b"\x10\x01"
DEFAULT_SESSION_RESPONSE = b"\x50\x01"


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]
    ret.radarUnavailable = Bus.radar not in DBC[candidate]

    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021)

    ret.enableBsm = 0x477 in fingerprint[0]

    ret.steerActuatorDelay = 0.1
    if candidate in (CAR.MAZDA_CX5_2022,):
      ret.steerActuatorDelay = 0.07
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate not in (CAR.MAZDA_CX5_2022,):
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    ret.centerToFront = ret.wheelbase * 0.41

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    ret.intelligentCruiseButtonManagementAvailable = True

    return ret

  @staticmethod
  def init(CP: structs.CarParams, CP_SP: structs.CarParamsSP, can_recv, can_send):
    if CP.carFingerprint != CAR.MAZDA_CX5_2022 or not (CP.flags & MazdaFlags.DEBUG_LONG.value):
      return

    try:
      query = IsoTpParallelQuery(can_send, can_recv, 0, [(MAZDA_RADAR_SESSION_ADDR, None)],
                                 [PROGRAMMING_SESSION_REQUEST], [PROGRAMMING_SESSION_RESPONSE])
      response = query.get_data(0.5)
      if response:
        carlog.warning(f"entered Mazda radar programming session at {hex(MAZDA_RADAR_SESSION_ADDR)}")
      else:
        carlog.error("Mazda radar programming session request returned no response")
    except Exception:
      carlog.exception("failed to enter Mazda radar programming session")

  @staticmethod
  def deinit(CP: structs.CarParams, can_recv, can_send):
    if CP.carFingerprint != CAR.MAZDA_CX5_2022 or not (CP.flags & MazdaFlags.DEBUG_LONG.value):
      return

    try:
      query = IsoTpParallelQuery(can_send, can_recv, 0, [(MAZDA_RADAR_SESSION_ADDR, None)],
                                 [DEFAULT_SESSION_REQUEST], [DEFAULT_SESSION_RESPONSE])
      query.get_data(0.5)
      carlog.warning("requested default session on Mazda radar")
    except Exception:
      carlog.exception("failed to return Mazda radar to default session")
