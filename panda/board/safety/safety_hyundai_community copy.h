bool hyundai_has_scc = true;
int OP_LKAS_live = 0;
int OP_MDPS_live = 0;
int OP_CLU_live = 0;
int OP_SCC_live = 0;
int car_SCC_live = 0;
int OP_EMS_live = 0;
int HKG_mdps_bus = 0;

const CanMsg HYUNDAI_COMMUNITY_TX_MSGS[] = {
  {832, 0, 8},  // LKAS11
  {1265, 0, 4},  {1265, 2, 4}, // CLU11 Bus
  {1157, 2, 4}, // LFAHDA_MFC
  {593, 1, 8},  // MDPS12
  {1056, 2, 8}, //   SCC11
  {1057, 2, 8}, //   SCC12
  {1290, 2, 8}, //   SCC13
  {905, 2, 8},  //   SCC14
  {1186, 2, 8},  //   4a2SCC 
  {790, 0, 8}, // EMS11
  {870, 0, 7}, // EMS366
  {912, 2, 7}, {912, 0, 7}, // SPAS11
  {1268, 2, 8}, {1268, 0, 8}, // SPAS12
 };

// older hyundai models have less checks due to missing counters and checksums
AddrCheckStruct hyundai_community_rx_checks[] = {
  {.msg = {{608, 2, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}}},
  {.msg = {{902, 2, 8, .expected_timestep = 20000U}}},
  // {.msg = {{916, 0, 8, .expected_timestep = 20000U}}}, some Santa Fe does not have this msg, need to find alternative
};
const int HYUNDAI_COMMUNITY_RX_CHECK_LEN = sizeof(hyundai_community_rx_checks) / sizeof(hyundai_community_rx_checks[0]);

static int hyundai_community_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  bool valid;
  int addr = GET_ADDR(to_push);
  int bus = GET_BUS(to_push);

  valid = addr_safety_check(to_push, hyundai_community_rx_checks, HYUNDAI_COMMUNITY_RX_CHECK_LEN,
                            hyundai_get_checksum, hyundai_compute_checksum,
                            hyundai_get_counter);


  if (valid) {
    if (addr == 593 && bus == mdps_bus) {
      int torque_driver_new = ((GET_BYTES_04(to_push) & 0x7ff) * 0.79) - 808; // scale down new driver torque signal to match previous one
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (addr == 1057 && OP_SCC_live) { // for cars with long control
      hyundai_has_scc = true;
      car_SCC_live = 50;
      // 2 bits: 13-14
      int cruise_engaged = (GET_BYTES_04(to_push) >> 13) & 0x3;
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }
    if (addr == 1056 && !OP_SCC_live) { // for cars without long control
      hyundai_has_scc = true;
      // 2 bits: 13-14
      int cruise_engaged = GET_BYTES_04(to_push) & 0x1; // ACC main_on signal
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }
    // cruise control for car without SCC
    if (addr == 608 && bus == car_bus && !hyundai_has_scc && !OP_SCC_live) {
      // bit 25
      int cruise_engaged = (GET_BYTES_04(to_push) >> 25 & 0x1); // ACC main_on signal
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }
    // engage for Cruise control disabled car
    if (addr == 1265 && bus == car_bus && OP_SCC_live && !car_SCC_live) {
      // first byte
      int cruise_button = (GET_BYTES_04(to_push) & 0x7);
      // enable on both accel and decel buttons falling edge
      if (!cruise_button && (cruise_engaged_prev == 1 || cruise_engaged_prev == 1)) {
        controls_allowed = 1;
      }
      // disable on cancel rising edge
      if (cruise_button == 4) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_button;
    }
    // exit controls on rising edge of gas press for cars with long control
    if (addr == 608 && OP_SCC_live && bus == car_bus) {
      gas_pressed = (GET_BYTE(to_push, 7) >> 6) != 0;
    }
    if (addr == 881 && OP_SCC_live && bus == car_bus) {
      gas_pressed = (((GET_BYTE(to_push, 4) & 0x7F) << 1) | GET_BYTE(to_push, 3) >> 7) > 5;
    }
    // sample wheel speed, averaging opposite corners
    if (addr == 902 && bus == car_bus) {
      int hyundai_speed = GET_BYTES_04(to_push) & 0x3FFF;  // FL
      hyundai_speed += (GET_BYTES_48(to_push) >> 16) & 0x3FFF;  // RL
      hyundai_speed /= 2;
      vehicle_moving = hyundai_speed > HYUNDAI_STANDSTILL_THRSLD;
    }
    // exit controls on rising edge of brake press for cars with long control
    if (addr == 916 && OP_SCC_live && bus == car_bus) {
      brake_pressed = (GET_BYTE(to_push, 6) >> 7) != 0;
    }

    //////////// for test ////////////

    

    generic_rx_checks(true);

    controls_allowed = true;
  }
  return valid;
}

static int hyundai_community_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  // if (!msg_allowed(to_send, HYUNDAI_COMMUNITY_TX_MSGS, sizeof(HYUNDAI_COMMUNITY_TX_MSGS)/sizeof(HYUNDAI_COMMUNITY_TX_MSGS[0]))) {
  //   tx = 0;
  // }

  if (relay_malfunction) {
    tx = 0;
  }

  // LKA STEER: safety check
  if (addr == 832) {
    OP_LKAS_live = 20;
    int desired_torque = ((GET_BYTES_04(to_send) >> 16) & 0x7ff) - 1024;
    uint32_t ts = TIM2->CNT;
    bool violation = 0;

    if (controls_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, HYUNDAI_MAX_STEER, -HYUNDAI_MAX_STEER);

      // *** torque rate limit check ***
      violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
        HYUNDAI_MAX_STEER, HYUNDAI_MAX_RATE_UP, HYUNDAI_MAX_RATE_DOWN,
        HYUNDAI_DRIVER_TORQUE_ALLOWANCE, HYUNDAI_DRIVER_TORQUE_FACTOR);

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, rt_torque_last, HYUNDAI_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > HYUNDAI_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (!controls_allowed) { // a reset worsen the issue of Panda blocking some valid LKAS messages
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }
  }

  // FORCE CANCEL: safety check only relevant when spamming the cancel button.
  // ensuring that only the cancel button press is sent (VAL 4) when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  //allow clu11 to be sent to MDPS if MDPS is not on bus0
  if (addr == 1265 && !controls_allowed && (bus != mdps_bus)) { 
    if ((GET_BYTES_04(to_send) & 0x7) != 4) {
      tx = 0;
    }
  }

  if (addr == 593) {OP_MDPS_live = 20;}
  if (addr == 1265 && bus == mdps_bus) {OP_CLU_live = 20;} // only count mesage created for MDPS
  if (addr == 1057) {OP_SCC_live = 20; if (car_SCC_live > 0) {car_SCC_live -= 1;}}
  // if (addr == 790) {OP_EMS_live = 20;}
  if (addr == 870) {OP_EMS_live = 20;}

  // 1 allows the message through
  return tx;
}

static int hyundai_community_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);

  // forward cam to ccan and viceversa, except lkas cmd
  if (!relay_malfunction) {
    if (bus_num == car_bus) {
      if (!OP_CLU_live || addr != 1265 ) {
        if (!OP_MDPS_live || addr != 593) {
          if (!OP_EMS_live || addr != 870) { // if (!OP_EMS_live || addr != 790) {
            bus_fwd = 10*scc_bus + mdps_bus;
          } else {
            bus_fwd = scc_bus;  // EON create EMS11 for MDPS
            OP_EMS_live -= 1;
          }
        } else {
          bus_fwd = -1;  // EON create MDPS for LKAS
          OP_MDPS_live -= 1;
        }
      } else {
        bus_fwd = -1; // EON create CLU12 for MDPS
        OP_CLU_live -= 1;
      }
    }
    if (bus_num == mdps_bus) {
      if (!OP_MDPS_live || addr != 593) {
        if (!OP_SCC_live || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
          bus_fwd = 10*car_bus + scc_bus;
        } else {
          bus_fwd = scc_bus;  // EON create SCC11 SCC12 SCC13 SCC14 for Car
          OP_SCC_live -= 1;
        }
      } else {
        bus_fwd = car_bus;  // EON create MDPS for LKAS
        OP_MDPS_live -= 1;
      }
    }
    if (bus_num == scc_bus) {
      if (!OP_LKAS_live || (addr != 832 && addr != 1157)) {
        if (!OP_SCC_live || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
          bus_fwd = 10*car_bus + mdps_bus;
        } else {
          bus_fwd = mdps_bus;
          OP_SCC_live -= 1;
        }
      } else {
        OP_LKAS_live -= 1;
      }
    }
  } else {
    if (bus_num == car_bus) {
      bus_fwd = 10*mdps_bus + scc_bus;
    }
    if (bus_num == mdps_bus) {
      bus_fwd = 10*car_bus + scc_bus;
    }
    if (bus_num == scc_bus) {
      bus_fwd = 10*car_bus + mdps_bus;
    }
  }
  return bus_fwd;
}

static void hyundai_community_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();

  if (board_has_obd() && HKG_forward_obd) {
    current_board->set_can_mode(CAN_MODE_NORMAL);
    }
}

const safety_hooks hyundai_community_hooks = {
  .init = hyundai_community_init,
  .rx = hyundai_community_rx_hook,
  .tx = hyundai_community_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_community_fwd_hook,
  .addr_check = hyundai_community_rx_checks,
  .addr_check_len = sizeof(hyundai_community_rx_checks) / sizeof(hyundai_community_rx_checks[0]),
};
