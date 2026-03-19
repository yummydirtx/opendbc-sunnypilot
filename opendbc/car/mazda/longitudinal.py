from __future__ import annotations

from enum import Enum

from opendbc.can.dbc import DBC
from opendbc.can.packer import set_value
from opendbc.car import make_tester_present_msg, uds
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery


MAZDA_LONG_DBC = DBC("mazda_2017")

RADAR_ADDR = 0x764
RADAR_BUS = 0

CRZ_INFO_ADDR = 0x21B
CRZ_CTRL_ADDR = 0x21C

CRZ_INFO_TEMPLATE = bytes.fromhex("01ffe20006800000")

LONG_COMMAND_STEP = 2
TESTER_PRESENT_STEP = 50


class MazdaLongitudinalProfile(str, Enum):
  STANDBY = "standby"
  ENGAGED_CRUISE = "engaged_cruise"
  ENGAGED_FOLLOW = "engaged_follow"
  STOP_GO_HOLD = "stop_go_hold"


CRZ_CTRL_TEMPLATES: dict[MazdaLongitudinalProfile, bytes] = {
  MazdaLongitudinalProfile.STANDBY: bytes.fromhex("02010b0000000000"),
  MazdaLongitudinalProfile.ENGAGED_CRUISE: bytes.fromhex("0a018b2000001000"),
  MazdaLongitudinalProfile.ENGAGED_FOLLOW: bytes.fromhex("0a018b4000001000"),
  MazdaLongitudinalProfile.STOP_GO_HOLD: bytes.fromhex("0a018b6000001000"),
}


def _get_signal(message_name: str, signal_name: str):
  return MAZDA_LONG_DBC.name_to_msg[message_name].sigs[signal_name]


def _patch_signal(message_name: str, raw: bytes, signal_name: str, value: float) -> bytes:
  sig = _get_signal(message_name, signal_name)
  encoded = int(round((value - sig.offset) / sig.factor))
  if encoded < 0:
    encoded = (1 << sig.size) + encoded

  dat = bytearray(raw)
  set_value(dat, sig, encoded)
  return bytes(dat)


def _compute_inverted_sum_checksum(raw: bytes, checksum_index: int = 7) -> int:
  return (0xFF - (sum(raw[i] for i in range(len(raw)) if i != checksum_index) & 0xFF)) & 0xFF


def _update_crz_info_checksum(raw: bytes) -> bytes:
  dat = bytearray(raw)
  dat[7] = _compute_inverted_sum_checksum(dat)
  return bytes(dat)


def clip(value: float, lower: float, upper: float) -> float:
  return min(max(value, lower), upper)


def accel_to_accel_cmd(accel: float) -> int:
  return int(round(clip(accel * 100.0, -350.0, 200.0)))


def build_crz_info(accel: float, counter: int) -> bytes:
  raw = _patch_signal("CRZ_INFO", CRZ_INFO_TEMPLATE, "ACCEL_CMD", accel_to_accel_cmd(accel))
  raw = _patch_signal("CRZ_INFO", raw, "CTR1", counter % 16)
  return _update_crz_info_checksum(raw)


def select_profile(long_active: bool, lead_visible: bool, standstill: bool) -> MazdaLongitudinalProfile:
  if not long_active:
    return MazdaLongitudinalProfile.STANDBY
  if standstill:
    return MazdaLongitudinalProfile.STOP_GO_HOLD
  if lead_visible:
    return MazdaLongitudinalProfile.ENGAGED_FOLLOW
  return MazdaLongitudinalProfile.ENGAGED_CRUISE


def build_crz_ctrl(long_active: bool, lead_visible: bool, standstill: bool) -> bytes:
  return CRZ_CTRL_TEMPLATES[select_profile(long_active, lead_visible, standstill)]


def create_longitudinal_messages(bus: int, accel: float, counter: int, long_active: bool,
                                 lead_visible: bool, standstill: bool) -> list[CanData]:
  return [
    CanData(CRZ_INFO_ADDR, build_crz_info(accel, counter), bus),
    CanData(CRZ_CTRL_ADDR, build_crz_ctrl(long_active, lead_visible, standstill), bus),
  ]


def create_radar_tester_present(bus: int = RADAR_BUS) -> CanData:
  return make_tester_present_msg(RADAR_ADDR, bus, suppress_response=True)


def _uds_request(can_recv, can_send, bus: int, addr: int, request: bytes, response: bytes,
                 *, timeout: float = 0.1) -> bool:
  query = IsoTpParallelQuery(can_send, can_recv, bus, [(addr, None)], [request], [response])
  return len(query.get_data(timeout)) > 0


def enter_radar_programming_session(can_recv, can_send, bus: int = RADAR_BUS, addr: int = RADAR_ADDR,
                                    retry: int = 5) -> bool:
  request = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.PROGRAMMING])
  response = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, uds.SESSION_TYPE.PROGRAMMING])

  for attempt in range(retry):
    try:
      if _uds_request(can_recv, can_send, bus, addr, request, response):
        carlog.warning(f"mazda radar programming session enabled on {hex(addr)}")
        return True
    except Exception:
      carlog.exception("mazda radar programming session exception")
    carlog.error(f"mazda radar programming session retry ({attempt + 1})")

  carlog.error("mazda radar programming session failed")
  return False


def request_radar_default_session(can_recv, can_send, bus: int = RADAR_BUS, addr: int = RADAR_ADDR) -> bool:
  request = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.DEFAULT])
  response = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, uds.SESSION_TYPE.DEFAULT])

  try:
    return _uds_request(can_recv, can_send, bus, addr, request, response)
  except Exception:
    carlog.exception("mazda radar default session exception")
    return False
