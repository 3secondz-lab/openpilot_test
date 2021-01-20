#!/usr/bin/env python
from common.numpy_fast import clip
from common.params import Params
from copy import copy
from cereal import car, log
import cereal.messaging as messaging
from selfdrive.car.car_helpers import get_car, get_one_can
from selfdrive.boardd.boardd import can_list_to_can_capnp
import csv
import numpy as np

HwType = log.HealthData.HwType

freq = 0.8
def steer_thread():
  poller = messaging.Poller()

  logcan = messaging.sub_sock('can')
  health = messaging.sub_sock('health')
  joystick_sock = messaging.sub_sock('testJoystick', conflate=True, poller=poller)

  carstate = messaging.pub_sock('carState')
  carcontrol = messaging.pub_sock('carControl')
  sendcan = messaging.pub_sock('sendcan')

  button_1_last = 0
  enabled = False
  Params().put("LongControlEnabled", str(1))
  # Params().put("MadModeEnabled", str(1))  

  # wait for health and CAN packets
  hw_type = messaging.recv_one(health).health.hwType
  has_relay = hw_type in [HwType.blackPanda, HwType.uno, HwType.dos]
  print("Waiting for CAN messages...")
  get_one_can(logcan)

  CI, CP = get_car(logcan, sendcan, has_relay)
  Params().put("CarParams", CP.to_bytes())

  CC = car.CarControl.new_message()

  p_cnt = 0
  i_cnt = 0
  d_cnt = 0


  k_p = 3.7
  k_i = 0.005
  k_d = 0.2
  c_i = 0.99

  ang_des = 0
  err = 0
  err_prev = 0

  aebenabled = False
  can_strs = messaging.drain_sock_raw(logcan, wait_for_one=True)
  CS = CI.update(CC, can_strs)
  log_dict = {"frame":0, "cmd_gas":0, "cmd_brake":0, "cmd_steering":0}
  log_msgs = [CI.CS.scc11, CI.CS.scc12, CI.CS.scc13, CI.CS.lkas11, CI.CS.lkas12, CI.CS.mdps11, CI.CS.mdps12]
  with open('log.csv','w') as f:
    w = csv.DictWriter(f, log_dict.keys())
    for msg in log_msgs:
        log_dict.update(msg)
    w.writeheader()
    while True:

      # send
      joystick = messaging.recv_one(joystick_sock)
      can_strs = messaging.drain_sock_raw(logcan, wait_for_one=True)
      CS = CI.update(CC, can_strs)

      # Usually axis run in pairs, up/down for one, and left/right for
      # the other.
      actuators = car.CarControl.Actuators.new_message()

      if joystick is not None:
        axis_0 = clip(-joystick.testJoystick.axes[0] * 1.0, -1., 1.)          # -1 to 1
        actuators.steer = axis_0
        actuators.steerAngle = axis_0 * 180.   # deg
        axis_3 = clip(-joystick.testJoystick.axes[5] * 1.0, -1., 1.)          # -1 to 1
        actuators.gas = max(axis_3, 0.)
        actuators.brake = max(-axis_3, 0.)

        pcm_cancel_cmd = joystick.testJoystick.buttons[0]
        button_1 = joystick.testJoystick.buttons[1]
        if not button_1 and button_1_last:
          enabled = not enabled

        button_1_last = button_1

        # print(" Enable:{}, Accel:{:.4f} , Steer:{:.4f}, steering Angle:{:.0f}°    ".format(enabled, \
        #      (actuators.gas - actuators.brake), actuators.steer, CS.steeringAngle))#, end='\r')
        hud_alert = 0
        if joystick.testJoystick.buttons[3]:
          hud_alert = "steerRequired"
          try:
            strtime +=1
          except Exception:
            strtime = 0
          actuators.steer = float(np.sin(freq*strtime/100*2*3.14))
          actuators.steerAngle = actuators.steer*90
        if joystick.testJoystick.buttons[2]:
          aebenabled = True
        else:
          aebenabled = False

      # CC.actuators.gas = actuators.gas
      # CC.actuators.brake = actuators.brake
      # CC.actuators.steer = actuators.steer
      # CC.actuators.steerAngle = actuators.steerAngle
      # CC.hudControl.visualAlert = hud_alert
      # CC.hudControl.setSpeed = 26.67
      # CC.hudControl.leadVisible = True
      # CC.cruiseControl.cancel = pcm_cancel_cmd
      # CC.enabled = enabled
      # CC.hudControl.leftLaneVisible = True
      # CC.hudControl.rightLaneVisible = True
      # CC.aebenabled = aebenabled
      can_sends = CI.apply(CC)

      ## Steering PID Controller

      ang_pre = ang_des
      ang_des = actuators.steerAngle

      err_prev = err    
      err = ang_des - CS.steeringAngle

      if (ang_des * CS.steeringAngle < 0 and abs(err)>90):
        if CS.steeringAngle < -90:
          ang_des = CS.steeringAngle + 90
        elif CS.steeringAngle > 90:
          ang_des = CS.steeringAngle - 90

      p_cnt = err*k_p
      i_cnt = err*k_i + c_i*i_cnt
      d_cnt = (err-err_prev)*k_d

      
      if (i_cnt*err<0):
        i_cnt = 0

      # print("Destination : {:.2f},  Err : {:.4f},  p_cnt : {:.4f},  i_cnt : {:.4f},  d_cnt : {:.4f} ".format(ang_des, err, p_cnt, i_cnt, d_cnt), end='\n')
      # print(CC)
      CC.actuators.steerAngle = ang_des + p_cnt + i_cnt + d_cnt
      # CC.actuators.steerAngle = ang_des
      ##
      sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan'))

      # broadcast carState
      cs_send = messaging.new_message('carState')
      cs_send.carState = copy(CS)
      carstate.send(cs_send.to_bytes())
      
      # broadcast carControl
      cc_send = messaging.new_message('carControl')
      cc_send.carControl = copy(CC)
      carcontrol.send(cc_send.to_bytes())
      # print(CS)
      log_dict["frame"] = CI.frame
      log_dict["cmd_gas"] = actuators.gas
      log_dict["cmd_brake"] = actuators.brake
      log_dict["cmd_steering"] = actuators.steerAngle
      log_dict.update(CI.CS.scc11)
      log_dict.update(CI.CS.scc12)
      log_dict.update(CI.CS.scc13)
      log_dict.update(CI.CS.lkas11)
      log_dict.update(CI.CS.lkas12)
      log_dict.update(CI.CS.mdps11) 
      log_dict.update(CI.CS.mdps12)
      print(log_dict)
      w.writerow(log_dict)
if __name__ == "__main__":
  steer_thread()
