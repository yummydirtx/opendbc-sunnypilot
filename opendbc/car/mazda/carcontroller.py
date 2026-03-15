from opendbc.can import CANPacker
from opendbc.car import Bus, make_tester_present_msg, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.longitudinal_experimental import (
  CRZ_CTRL_ADDR,
  CRZ_EVENTS_ADDR,
  CRZ_INFO_ADDR,
  MazdaLongitudinalReplayMutator,
  accel_to_info_accel_cmd,
  make_default_debug_long_stream,
)
from opendbc.car.mazda.values import CarControllerParams, Buttons, MAZDA_RADAR_SESSION_ADDR, MazdaFlags

from opendbc.sunnypilot.car.mazda.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.params = CarControllerParams(CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.debug_long_enabled = bool(CP.flags & MazdaFlags.DEBUG_LONG.value)
    self.debug_long_mutator = MazdaLongitudinalReplayMutator(make_default_debug_long_stream()) if self.debug_long_enabled else None
    self.debug_long_pending = None

  def _get_debug_long_command_set(self, CC):
    if self.debug_long_mutator is None:
      return None

    info_accel_cmd = accel_to_info_accel_cmd(CC.actuators.accel) if CC.longActive else None
    return self.debug_long_mutator.next_command_set(
      info_accel_cmd=info_accel_cmd,
      unsafe_patch_events=CC.longActive,
    )

  def _append_debug_long_can(self, can_sends, CC):
    if not self.debug_long_enabled or self.debug_long_mutator is None:
      return

    if self.frame % 50 == 0:
      can_sends.append(make_tester_present_msg(MAZDA_RADAR_SESSION_ADDR, 0, suppress_response=True))

    if self.frame % 2 == 0:
      self.debug_long_pending = self._get_debug_long_command_set(CC)
      if self.debug_long_pending is not None:
        can_sends.append(CanData(CRZ_EVENTS_ADDR, self.debug_long_pending.raw_21f, 0))
    else:
      if self.debug_long_pending is None:
        self.debug_long_pending = self._get_debug_long_command_set(CC)
      if self.debug_long_pending is not None:
        can_sends.append(CanData(CRZ_INFO_ADDR, self.debug_long_pending.raw_21b, 0))
        can_sends.append(CanData(CRZ_CTRL_ADDR, self.debug_long_pending.raw_21c, 0))
        self.debug_long_pending = None

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    apply_torque = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, self.params)

    if not self.debug_long_enabled and CC.cruiseControl.cancel:
      # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
      # a race condition with the stock system, where the second cancel from openpilot
      # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
      # read 3 messages and most likely sync state before we attempt cancel.
      self.brake_counter = self.brake_counter + 1
      if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
        # Cancel Stock ACC if it's enabled while OP is disengaged
        # Send at a rate of 10hz until we sync with stock ACC state
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    elif not self.debug_long_enabled:
      self.brake_counter = 0
      if CC.cruiseControl.resume and self.frame % 5 == 0:
        # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
        # Send Resume button when planner wants car to move
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))
    else:
      self.brake_counter = 0

    self.apply_torque_last = apply_torque

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      # TODO: find a way to silence audible warnings so we can add more hud alerts
      steer_required = steer_required and CS.lkas_allowed_speed
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))

    self._append_debug_long_can(can_sends, CC)

    if not self.debug_long_enabled:
      # Intelligent Cruise Button Management
      can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer, self.frame, self.last_button_frame))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
