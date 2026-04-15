from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda.longitudinal import LONG_COMMAND_STEP, NEAR_STOP_ENTRY_SPEED, RADAR_BUS, TESTER_PRESENT_STEP, \
                                           create_longitudinal_messages, create_radar_tester_present, hold_brake_accel, \
                                           hold_latched_accel, near_stop_brake_accel
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import CarControllerParams, Buttons

from opendbc.sunnypilot.car.mazda.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

CRZ_CTRL_LATCH_FRAMES = int(round(2.0 / DT_CTRL))
CRZ_CTRL_PASSIVE_FRAMES = int(round(9.6 / DT_CTRL))
HOLD_REQUEST_FRAMES = int(round(6.0 / DT_CTRL))
RESUME_RELEASE_FRAMES = int(round(0.5 / DT_CTRL))


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.params = CarControllerParams(CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.long_counter = 0
    self.standstill_hold_frames = 0
    self.resume_release_frames = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    apply_torque = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, self.params)

    if not self.CP.openpilotLongitudinalControl:
      if CC.cruiseControl.cancel:
        # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
        # a race condition with the stock system, where the second cancel from openpilot
        # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
        # read 3 messages and most likely sync state before we attempt cancel.
        self.brake_counter = self.brake_counter + 1
        if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
          # Cancel Stock ACC if it's enabled while OP is disengaged
          # Send at a rate of 10hz until we sync with stock ACC state
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
      else:
        self.brake_counter = 0
        if CC.cruiseControl.resume and self.frame % 5 == 0:
          # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
          # Send Resume button when planner wants car to move
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))
    else:
      self.brake_counter = 0
      if CS.out.standstill and CC.cruiseControl.resume and self.frame % 5 == 0:
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

    self.apply_torque_last = apply_torque

    if self.CP.openpilotLongitudinalControl:
      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      starting = CC.actuators.longControlState == LongCtrlState.starting
      resume_button_requested = CC.cruiseControl.resume
      release_hold_requested = CC.cruiseControl.override or CS.out.gasPressed or starting

      if not CC.longActive:
        self.standstill_hold_frames = 0
        self.resume_release_frames = 0
      else:
        if CS.out.standstill and not release_hold_requested:
          self.standstill_hold_frames += 1
        else:
          self.standstill_hold_frames = 0

      # Stock MRCC enters its stop-go state before the standstill bit flips.
      # Mirror that near-stop transition on the synthesized CRZ frames while
      # keeping the later latch timing anchored to true standstill.
      # A virtual RES press should happen while Mazda still sees the passive
      # stop-go hold state. Only release that synthetic hold once the car is
      # actually starting to move or the driver overrides with gas.
      stop_go_request = CC.longActive and not release_hold_requested and (CS.out.standstill or stopping or CS.out.vEgo < NEAR_STOP_ENTRY_SPEED)
      standstill_hold_request = CC.longActive and CS.out.standstill and not release_hold_requested
      hold_latched = standstill_hold_request and self.standstill_hold_frames > HOLD_REQUEST_FRAMES
      brake_release_requested = release_hold_requested or (resume_button_requested and (not stopping or hold_latched))

      if CS.out.standstill and brake_release_requested:
        self.resume_release_frames = RESUME_RELEASE_FRAMES
      elif self.resume_release_frames > 0:
        self.resume_release_frames -= 1

      crz_info_hold_request = stop_go_request and not brake_release_requested
      crz_hold_latched = standstill_hold_request and self.standstill_hold_frames >= CRZ_CTRL_LATCH_FRAMES
      # Stock resumes from passive hold by re-enabling ACC while the RES press
      # is active, instead of staying indefinitely in the passive-hold substate.
      crz_hold_passive = standstill_hold_request and self.standstill_hold_frames >= CRZ_CTRL_PASSIVE_FRAMES and not resume_button_requested
      release_brake = self.resume_release_frames > 0

      accel = 0.0
      if CC.longActive:
        accel = CC.actuators.accel
        if release_brake:
          accel = max(accel, 0.0)
        elif CS.out.standstill:
          accel = hold_latched_accel() if hold_latched else hold_brake_accel()
        elif stopping or CS.out.vEgo < NEAR_STOP_ENTRY_SPEED:
          accel = min(accel, near_stop_brake_accel(CS.out.vEgo))

      if self.frame % TESTER_PRESENT_STEP == 0:
        can_sends.append(create_radar_tester_present(RADAR_BUS))

      if self.frame % LONG_COMMAND_STEP == 0:
        long_active = CC.longActive
        lead_visible = CC.hudControl.leadVisible
        can_sends.extend(create_longitudinal_messages(RADAR_BUS, accel, self.long_counter,
                                                      long_active, lead_visible, CS.out.standstill,
                                                      hold_request=crz_info_hold_request,
                                                      crz_ctrl_hold_request=stop_go_request,
                                                      hold_latched=hold_latched,
                                                      crz_hold_latched=crz_hold_latched,
                                                      crz_hold_passive=crz_hold_passive,
                                                      v_ego=CS.out.vEgo))
        self.long_counter = (self.long_counter + 1) % 16

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

    # Intelligent Cruise Button Management
    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer, self.frame, self.last_button_frame))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
