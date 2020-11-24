from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import clip
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import *
from selfdrive.car.hyundai.values import Buttons, SteerLimitParams, CAR, FEATURES
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV

VisualAlert = car.CarControl.HUDControl.VisualAlert
min_set_speed = 60 * CV.KPH_TO_MS

# Accel limits
ACCEL_HYST_GAP = 0.02  # don't change accel command for small oscilalitons within this value
# ACCEL_MAX = 1.5  # 1.5 m/s2
# ACCEL_MIN = -3.0 # 3   m/s2
ACCEL_MAX = 10.0  # 1.5 m/s2
ACCEL_MIN = -10.0 # 3   m/s2
ACCEL_SCALE = max(ACCEL_MAX, -ACCEL_MIN)
# SPAS steering limits
STEER_ANG_MAX = 360          # SPAS Max Angle
STEER_ANG_MAX_RATE = 3.6    # SPAS Degrees per ms

def accel_hysteresis(accel, accel_steady):

  # for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if accel > accel_steady + ACCEL_HYST_GAP:
    accel_steady = accel - ACCEL_HYST_GAP
  elif accel < accel_steady - ACCEL_HYST_GAP:
    accel_steady = accel + ACCEL_HYST_GAP
  accel = accel_steady

  return accel, accel_steady

def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart, button_on):
  sys_warning = (visual_alert == VisualAlert.steerRequired)

  # initialize to no line visible
  sys_state = 1
  if not button_on:
    lane_visible = 0
  if left_lane and right_lane or sys_warning:  #HUD alert only display when LKAS status is active
    if enabled or sys_warning:
      sys_state = 3
    else:
      sys_state = 4
  elif left_lane:
    sys_state = 5
  elif right_lane:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if left_lane_depart:
    left_lane_warning = 1 if fingerprint in [CAR.GENESIS, CAR.GENESIS_G70, CAR.GENESIS_G80, CAR.GENESIS_G90] else 2
  if right_lane_depart:
    right_lane_warning = 1 if fingerprint in [CAR.GENESIS, CAR.GENESIS_G70, CAR.GENESIS_G80, CAR.GENESIS_G90] else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(dbc_name)
    self.accel_steady = 0
    self.apply_steer_last = 0
    self.steer_rate_limited = False
    self.lkas11_cnt = 0
    self.scc12_cnt = 0
    self.last_resume_frame = 0
    self.last_lead_distance = 5
    self.turning_signal_timer = 0
    self.lkas_button_on = True
    self.longcontrol = CP.openpilotLongitudinalControl
    self.scc_live = not CP.radarOffCan
    self.active = False
    if CP.spasEnabled:
      self.en_cnt = 0
      self.apply_steer_ang = 0.0
      self.en_spas = 3
      self.mdps11_stat_last = 0
      # self.spas_always = False
      # 3secondz
      self.spas_always = True

    self.p = SteerLimitParams(CP)

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert,
             left_lane, right_lane, left_lane_depart, right_lane_depart, set_speed, lead_visible, lead_dist=150.0):
    # self.active = enabled
    # self.active = True
    # *** compute control surfaces ***

    # gas and brake
    apply_accel = actuators.gas - actuators.brake

    apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady)
    apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX)

    # Steering Torque
    new_steer = actuators.steer * self.p.STEER_MAX
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
    self.steer_rate_limited = new_steer != apply_steer

    # SPAS limit angle extremes for safety
    if CS.spas_enabled:
      apply_steer_ang_req = clip(actuators.steerAngle, -1*(STEER_ANG_MAX), STEER_ANG_MAX)
      # SPAS limit angle rate for safety
      if abs(self.apply_steer_ang - apply_steer_ang_req) > STEER_ANG_MAX_RATE:
        if apply_steer_ang_req > self.apply_steer_ang:
          self.apply_steer_ang += STEER_ANG_MAX_RATE
        else:
          self.apply_steer_ang -= STEER_ANG_MAX_RATE
      else:
        self.apply_steer_ang = apply_steer_ang_req
    spas_active = CS.spas_enabled and enabled and (self.spas_always or CS.out.vEgo < 7.0) # 25km/h

    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    # temporarily disable steering when LKAS button off 
    lkas_active = enabled and abs(CS.out.steeringAngle) < 90. and self.lkas_button_on and not spas_active

    # fix for Genesis hard fault at low speed
    # if CS.out.vEgo < 60 * CV.KPH_TO_MS and self.car_fingerprint == CAR.GENESIS and not CS.mdps_bus:
    #   lkas_active = False

    # Optima has blinker flash signal only
    if self.car_fingerprint in [CAR.OPTIMA, CAR.OPTIMA_HEV]:
      if CS.left_blinker_flash or CS.right_blinker_flash: 
        self.turning_signal_timer = 100
    # Disable steering while turning blinker on and speed below 60 kph
    elif CS.out.leftBlinker or CS.out.rightBlinker:
      self.turning_signal_timer = 100  # Disable for 1.0 Seconds after blinker turned off
    if self.turning_indicator_alert: # set and clear by interface
      lkas_active = 0
    if self.turning_signal_timer > 0:
      self.turning_signal_timer -= 1

    if not lkas_active and not self.spas_always:
      apply_steer = 0

    self.apply_accel_last = apply_accel
    self.apply_steer_last = apply_steer

    sys_warning, sys_state, left_lane_warning, right_lane_warning =\
      process_hud_alert(lkas_active, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart,
                        self.lkas_button_on)

    clu11_speed = CS.clu11["CF_Clu_Vanz"]
    enabled_speed = 38 if CS.is_set_speed_in_mph  else 60
    if clu11_speed > enabled_speed:
      enabled_speed = clu11_speed

    if not(min_set_speed < set_speed < 255 * CV.KPH_TO_MS):
      set_speed = min_set_speed 
    set_speed *= CV.MS_TO_MPH if CS.is_set_speed_in_mph else CV.MS_TO_KPH

    if frame == 0: # initialize counts from last received count signals
      self.lkas11_cnt = CS.lkas11["CF_Lkas_MsgCount"]
      self.scc12_cnt = CS.scc12["CR_VSM_Alive"] + 1 if not CS.no_radar else 0

      #TODO: fix this
      # self.prev_scc_cnt = CS.scc11["AliveCounterACC"]
      # self.scc_update_frame = frame

    # check if SCC is alive
    # if frame % 7 == 0:
    #   if CS.scc11["AliveCounterACC"] == self.prev_scc_cnt:
    #     if frame - self.scc_update_frame > 20 and self.scc_live:
    #       self.scc_live = False
    #   else:
    #     self.scc_live = True
    #     self.prev_scc_cnt = CS.scc11["AliveCounterACC"]
    #     self.scc_update_frame = frame

    
    self.prev_scc_cnt = CS.scc11["AliveCounterACC"]

    self.lkas11_cnt = (self.lkas11_cnt + 1) % 0x10
    self.scc12_cnt %= 0xF


    can_sends = []

    # if (frame % 10) == 0:
    #     can_sends.append(create_lkas12(CS.sas_bus))
    #     can_sends.append(create_lkas12(CS.mdps_bus))
    # if (frame % 50) == 0:
    #   can_sends.append(create_1191(CS.sas_bus))
    # if (frame % 7) == 0:
    #   can_sends.append(create_1156(CS.sas_bus))

    # Send LKAS dummy to car
    # can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
    #                                CS.lkas11, sys_warning, sys_state, enabled, left_lane, right_lane,
    #                                left_lane_warning, right_lane_warning, CS.sas_bus))

    # # Send LKAS dummy to mdps1
    # can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
    #                               CS.lkas11, sys_warning, sys_state, enabled, left_lane, right_lane,
    #                               left_lane_warning, right_lane_warning, CS.mdps_bus))

    if frame % 2 and CS.mdps_bus: # send clu11 to mdps if it is not on bus 0
      can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.NONE, enabled_speed, CS.mdps_bus))

    if pcm_cancel_cmd and self.longcontrol:
      can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.CANCEL, clu11_speed, CS.scc_bus))

    if CS.out.cruiseState.standstill:
      # run only first time when the car stopped
      if self.last_lead_distance == 0:
        # get the lead distance from the Radar
        self.last_lead_distance = CS.lead_distance
        self.resume_cnt = 0
      # when lead car starts moving, create 6 RES msgs
      elif CS.lead_distance != self.last_lead_distance and (frame - self.last_resume_frame) > 5:
        can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL, clu11_speed, CS.scc_bus))
        self.resume_cnt += 1
        # interval after 6 msgs
        if self.resume_cnt > 5:
          self.last_resume_frame = frame
          self.clu11_cnt = 0
    # reset lead distnce after the car starts moving
    elif self.last_lead_distance != 0:
      self.last_lead_distance = 0 
    
    if CS.mdps_bus: # send mdps12 to LKAS to prevent LKAS error
      can_sends.append(create_mdps12(self.packer, frame, CS.mdps12, CS.scc_bus))
    # ###############TEST#################
    if frame % 2 == 0: 
      can_sends.append(create_scc11(self.packer, frame, enabled, set_speed, lead_dist, lead_visible, self.scc_live, CS.scc11, CS.sas_bus))
      can_sends.append(create_scc12(self.packer, apply_accel, enabled, self.scc12_cnt, self.scc_live, CS.scc12, CS.sas_bus, CS.out.stockAeb, CS.out.gasPressed, CS.out.cruiseState.standstill))
    #   # if frame % 20 == 0:
    #   #   can_sends.append(create_scc13(self.packer, CS.scc13, CS.sas_bus))
      # if True:
      can_sends.append(create_scc14(self.packer, enabled,apply_accel, CS.scc14, CS.sas_bus, CS.out.stockAeb, CS.out.gasPressed, CS.out.cruiseState.standstill,CS.out.vEgo))
    #   self.prev_scc_cnt = self.scc12_cnt
      self.scc12_cnt += 1
      
      ###########################

    # send scc to car if longcontrol enabled and SCC not on bus 0 or ont live
    # if self.longcontrol and (CS.scc_bus or not self.scc_live) and frame % 2 == 0: 
    #   can_sends.append(create_scc12(self.packer, apply_accel, enabled, self.scc12_cnt, self.scc_live, CS.scc12, CS.sas_bus))
    #   can_sends.append(create_scc11(self.packer, frame, enabled, set_speed, lead_visible, self.scc_live, CS.scc11, CS.sas_bus))
    #   if CS.has_scc13 and frame % 20 == 0:
    #     can_sends.append(create_scc13(self.packer, CS.scc13, CS.sas_bus))
    #   if CS.has_scc14:
    #     can_sends.append(create_scc14(self.packer, enabled, CS.scc14, CS.sas_bus))
    #   self.scc12_cnt += 1

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.car_fingerprint in FEATURES["send_lfa_mfa"]:
      can_sends.append(create_lfa_mfa(self.packer, frame, lkas_active, CS.sas_bus))

    if CS.spas_enabled:
    # if True:
      if CS.mdps_bus:
      # can_sends.append(create_ems11(self.packer, CS.ems11, spas_active, CS.mdps_bus))
        can_sends.append(create_ems366(self.packer, CS.ems366, spas_active, CS.mdps_bus))
      # SPAS11 50hz
      if (frame % 2) == 0:
        if CS.mdps11_stat == 7 and not self.mdps11_stat_last == 7:
          self.en_spas = 7
          self.en_cnt = 0

        if self.en_spas == 7 and self.en_cnt >= 8:
          self.en_spas = 3
          self.en_cnt = 0
  
        if self.en_cnt < 8 and spas_active:
          self.en_spas = 4
        elif self.en_cnt >= 8 and spas_active:
          self.en_spas = 5

        if not spas_active:
          self.apply_steer_ang = CS.mdps11_strang
          self.en_spas = 3
          self.en_cnt = 0

        self.mdps11_stat_last = CS.mdps11_stat
        self.en_cnt += 1
        can_sends.append(create_spas11(self.packer, self.car_fingerprint, (frame // 2), self.en_spas, self.apply_steer_ang, CS.mdps_bus))

      # SPAS12 20Hz
      if (frame % 5) == 0:
        can_sends.append(create_spas12(CS.mdps_bus))

    return can_sends