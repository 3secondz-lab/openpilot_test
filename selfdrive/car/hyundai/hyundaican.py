import crcmod
from selfdrive.car.hyundai.values import CAR, CHECKSUM

hyundai_checksum = crcmod.mkCrcFun(0x11D, initCrc=0xFD, rev=False, xorOut=0xdf)


def create_lkas11(packer, frame, car_fingerprint, apply_steer, steer_req,
                  lkas11, sys_warning, sys_state, enabled,
                  left_lane, right_lane,
                  left_lane_depart, right_lane_depart, bus):
  values = lkas11
  values["CF_Lkas_LdwsSysState"] = sys_state
  values["CF_Lkas_SysWarning"] = 3 if sys_warning else 0
  values["CF_Lkas_LdwsLHWarning"] = left_lane_depart
  values["CF_Lkas_LdwsRHWarning"] = right_lane_depart
  values["CR_Lkas_StrToqReq"] = apply_steer
  values["CF_Lkas_ActToi"] = steer_req
  values["CF_Lkas_ToiFlt"] = 0
  values["CF_Lkas_MsgCount"] = frame % 0x10
  values["CF_Lkas_Chksum"] = 0

  if car_fingerprint in [CAR.SONATA, CAR.PALISADE, CAR.SONATA_HEV, CAR.SANTA_FE, CAR.KONA_EV, CAR.NIRO_EV]:
    values["CF_Lkas_LdwsActivemode"] = int(left_lane) + (int(right_lane) << 1)
    values["CF_Lkas_LdwsOpt_USM"] = 2

    # FcwOpt_USM 5 = Orange blinking car + lanes
    # FcwOpt_USM 4 = Orange car + lanes
    # FcwOpt_USM 3 = Green blinking car + lanes
    # FcwOpt_USM 2 = Green car + lanes
    # FcwOpt_USM 1 = White car + lanes
    # FcwOpt_USM 0 = No car + lanes
    values["CF_Lkas_FcwOpt_USM"] = 2 if enabled else 1

    # SysWarning 4 = keep hands on wheel
    # SysWarning 5 = keep hands on wheel (red)
    # SysWarning 6 = keep hands on wheel (red) + beep
    # Note: the warning is hidden while the blinkers are on
    values["CF_Lkas_SysWarning"] = 4 if sys_warning else 0

  elif car_fingerprint == CAR.GENESIS:
    # This field is actually LdwsActivemode
    # Genesis and Optima fault when forwarding while engaged
    values["CF_Lkas_LdwsActivemode"] = 2
  elif car_fingerprint == CAR.OPTIMA:
    values["CF_Lkas_LdwsActivemode"] = 0

  dat = packer.make_can_msg("LKAS11", bus, values)[2]

  if car_fingerprint in CHECKSUM["crc8"]:
    # CRC Checksum as seen on 2019 Hyundai Santa Fe
    dat = dat[:6] + dat[7:8]
    checksum = hyundai_checksum(dat)
  elif car_fingerprint in CHECKSUM["6B"]:
    # Checksum of first 6 Bytes, as seen on 2018 Kia Sorento
    checksum = sum(dat[:6]) % 256
  else:
    # Checksum of first 6 Bytes and last Byte as seen on 2018 Kia Stinger
    checksum = (sum(dat[:6]) + dat[7]) % 256

  values["CF_Lkas_Chksum"] = checksum

  return packer.make_can_msg("LKAS11", bus, values)

def create_clu11(packer, frame, clu11, button, speed, bus):
  values = clu11
  values["CF_Clu_CruiseSwState"] = button
  values["CF_Clu_Vanz"] = speed
  values["CF_Clu_AliveCnt1"] = frame // 2 % 0x10
  return packer.make_can_msg("CLU11", bus, values)

def create_lfa_mfa(packer, frame, enabled, bus):
  values = {
    "ACTIVE": enabled,
    "HDA_USM": 2,
  }

  # ACTIVE 1 = Green steering wheel icon

  # LFA_USM 2 & 3 = LFA cancelled, fast loud beeping
  # LFA_USM 0 & 1 = No mesage

  # LFA_SysWarning 1 = "Switching to HDA", short beep
  # LFA_SysWarning 2 = "Switching to Smart Cruise control", short beep
  # LFA_SysWarning 3 =  LFA error

  # ACTIVE2: nothing
  # HDA_USM: nothing

  return packer.make_can_msg("LFAHDA_MFC", bus, values)

def create_mdps12(packer, frame, mdps12, bus):
  values = mdps12
  values["CF_Mdps_ToiActive"] = 0
  values["CF_Mdps_ToiUnavail"] = 0
  values["CF_Mdps_MsgCount2"] = frame % 0x100
  values["CF_Mdps_Chksum2"] = 0

  dat = packer.make_can_msg("MDPS12", bus, values)[2]
  checksum = sum(dat) % 256
  values["CF_Mdps_Chksum2"] = checksum

  return packer.make_can_msg("MDPS12", bus, values)

def create_scc11(packer, frame, enabled, aebcmdact, set_speed, lead_dist, lead_visible, scc_live, scc11, bus):
  values = scc11

  # ## TEST ##
  # values["MainMode_ACC"] = 1
  # values["SCCInfoDisplay"] = 0
  # values["DriverAlertDisplay"] = 0
  # values["TauGapSet"] = 4
  # values["ACC_ObjLatPos"] = 0
  # values["ACC_ObjRelSpd"] = 0
  # values["VSetDis"] = set_speed
  # ## END ##

  values["AliveCounterACC"] = frame // 2 % 0x10
  if not scc_live:
    values["MainMode_ACC"] = 0
    values["VSetDis"] = set_speed

  values["ObjValid"] = 1 if enabled else 0
  values["ACC_ObjDist"] = lead_dist
  values["ACC_ObjStatus"] = lead_visible

  if aebcmdact:
    values["ObjValid"] = 1
    values["ACC_ObjDist"] = 20
    values["ACC_ObjStatus"] = 1
    values["ACC_ObjLatPos"] = 0
    values["ACC_ObjRelSpd"] = -20
    values["DriverAlertDisplay"] = 2
  print(values)
  return packer.make_can_msg("SCC11", bus, values)

def create_scc12(packer, apply_accel, enabled, cnt, scc_live, scc12, bus, aebcmdact, aeb_cnt, gaspressed, standstill):
  values = scc12
  if not scc_live and enabled and not aebcmdact:
    values["ACCMode"] = 2 if gaspressed and (apply_accel > -0.2) else 1
    # if apply_accel < 0.0 and standstill:
    #   values["StopReq"] = 1
    values["aReqRaw"] = apply_accel #aReqMax
    values["aReqValue"] = apply_accel  #aReqMin
  values["CR_VSM_Alive"] = cnt
  values["CR_VSM_ChkSum"] = 0

  dat = packer.make_can_msg("SCC12", bus, values)[2]
  values["CR_VSM_ChkSum"] = 16 - sum([sum(divmod(i, 16)) for i in dat]) % 16

  if aebcmdact:
    values["AEB_Status"]=0
    values["CR_VSM_ChkSum"] = 0
    values["CR_VSM_Alive"] = 0
    if aeb_cnt < 5:
      values["AEB_StopReq"] = 0
      values["CF_VSM_Stat"] = 1
    elif aeb_cnt < 12:
      values["AEB_CmdAct"] = 1
      values["AEB_StopReq"] = 1
      values["CF_VSM_Stat"] = 1 
      values["StopReq"] = 1
    elif aeb_cnt < 18:
      values["AEB_StopReq"] = 0
      values["CF_VSM_Stat"] = 1
    elif aeb_cnt < 27:
      values["ACCMode"] = 1
      values["AEB_CmdAct"] = 1
      values["AEB_StopReq"] = 1
      values["CR_VSM_DecCmd"] = 0.2
      values["CF_VSM_Stat"] = 2
    else:
      values["ACCMode"] = 0
      values["AEB_CmdAct"] = 1
      values["StopReq"] = 1
      values["CR_VSM_DecCmd"] = 1.0
      # values["CF_VSM_ConfMode"] = 1
      values["CF_VSM_Stat"] = 2

  print(values)
  return packer.make_can_msg("SCC12", bus, values)

def create_scc13(packer, scc13, bus):
  values = scc13
  return packer.make_can_msg("SCC13", bus, values)

def create_scc14(packer, enabled, apply_accel, scc14, bus, aebcmdact, gaspressed, standstill, e_vgo):
  values = scc14
  if enabled:
      values["SCCMode"] = 2 if gaspressed and (apply_accel > -0.2) else 1
      # values["ObjGap"] = objgap
      if standstill:
        values["JerkUpperLimit"] = 50.
        values["JerkLowerLimit"] = 10.
        values["ComfortBandUpper"] = 10.
        values["ComfortBandLower"] = 10.
        if e_vgo > 0.27:
          values["ComfortBandUpper"] = 2.
          values["ComfortBandLower"] = 0.
      else:
        values["JerkUpperLimit"] = 50.
        values["JerkLowerLimit"] = 50.
        values["ComfortBandUpper"] = 50.
        values["ComfortBandLower"] = 50.

  return packer.make_can_msg("SCC14", bus, values)

def create_spas11(packer, car_fingerprint, frame, en_spas, apply_steer, bus):
  values = {
    "CF_Spas_Stat": en_spas,
    "CF_Spas_TestMode": 0,
    "CR_Spas_StrAngCmd": apply_steer,
    "CF_Spas_BeepAlarm": 0,
    "CF_Spas_Mode_Seq": 2,
    "CF_Spas_AliveCnt": frame % 0x200,
    "CF_Spas_Chksum": 0,
    "CF_Spas_PasVol": 0,
  }
  dat = packer.make_can_msg("SPAS11", bus, values)[2]
  if car_fingerprint in CHECKSUM["crc8"]:
    dat = dat[:6]
    values["CF_Spas_Chksum"] = hyundai_checksum(dat)
  else:
    values["CF_Spas_Chksum"] = sum(dat[:6]) % 256
  return packer.make_can_msg("SPAS11", bus, values)

def create_spas12(bus):
  return [1268, 0, b"\x00\x00\x00\x00\x00\x00\x00\x00", bus]

def create_ems11(packer, ems11, enabled, bus):
  values = ems11
  if enabled:
    values["VS"] = 0
  return packer.make_can_msg("EMS11", bus, values)

def create_ems366(packer, ems366, enabled, bus):
  values = ems366
  if enabled:
    values["VS"] = 0
  return packer.make_can_msg("EMS366", bus, values)

def create_1191(bus):
  return [1191, 0, b"\x01\x00", bus]

def create_1156(bus):
  return [1156, 0, b"\x08\x20\xfe\x3f\x00\xe0\xfd\x3f", bus]

def create_lkas12(bus):
  return [1342, 0, b"\x00\x00\x00\x00\x60\x05", bus]
