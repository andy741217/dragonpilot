
const int HYUNDAI_COMMUNITY_MAX_ACCEL = 150;        // 1.5 m/s2
const int HYUNDAI_COMMUNITY_MIN_ACCEL = -300;       // -3.0 m/s2

const int HYUNDAI_COMMUNITY_ISO_MAX_ACCEL = 200;        // 2.0 m/s2
const int HYUNDAI_COMMUNITY_ISO_MIN_ACCEL = -350;       // -3.5 m/s2

//bool hyundai_community_non_scc_car = false;
//bool aeb_cmd_act = false;
//int prev_desired_accel = 0;
//int decel_not_ramping = 0;



const CanMsg HYUNDAI_COMMUNITY_TX_MSGS[] = {
  {832, 0, 8}, {832, 1, 8}, // LKAS11 Bus 0, 1
  {1265, 0, 4}, {1265, 1, 4}, {1265, 2, 4},// CLU11 Bus 0, 1, 2
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
  {1056, 0, 8}, //   SCC11,  Bus 0
  {1057, 0, 8}, //   SCC12,  Bus 0
  {1290, 0, 8}, //   SCC13,  Bus 0
  {905, 0, 8},  //   SCC14,  Bus 0
  {1186, 0, 8}, //  4a2SCC,  Bus 0
  {1155, 0, 8}, //   FCA12,  Bus 0
  {909, 0, 8},  //   FCA11,  Bus 0
  {2000, 0, 8}, // SCC_DIAG, Bus 0
  {593, 2, 8}   //  MDPS12,  Bus 2
 };

// TODO: missing checksum for wheel speeds message,worst failure case is
//       wheel speeds stuck at 0 and we don't disengage on brake press
AddrCheckStruct hyundai_community_rx_checks[] = {
  {.msg = {{902, 0, 8, .expected_timestep = 10000U}}},
  {.msg = {{916, 0, 8, .expected_timestep = 10000U}}},
  {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}}},
};
const int HYUNDAI_COMMUNITY_RX_CHECK_LEN = sizeof(hyundai_community_rx_checks) / sizeof(hyundai_community_rx_checks[0]);

// for non SCC hyundai vehicles
//AddrCheckStruct hyundai_community_nonscc_rx_checks[] = {
//  {.msg = {{902, 0, 8, .expected_timestep = 10000U}}},
//  {.msg = {{916, 0, 8, .expected_timestep = 10000U}}},
//};

//const int HYUNDAI_COMMUNITY_NONSCC_RX_CHECK_LEN = sizeof(hyundai_community_nonscc_rx_checks) / sizeof(hyundai_community_nonscc_rx_checks[0]);

//static uint8_t hyundai_community_get_counter(CAN_FIFOMailBox_TypeDef *to_push) {
  //int addr = GET_ADDR(to_push);

//  uint8_t cnt;
//  if (addr == 902) {
//    cnt = ((GET_BYTE(to_push, 3) >> 6) << 2) | (GET_BYTE(to_push, 1) >> 6);
//  } else if (addr == 916) {
//    cnt = (GET_BYTE(to_push, 1) >> 5) & 0x7;
//  } else if (addr == 1057) {
//    cnt = GET_BYTE(to_push, 7) & 0xF;
//  } else {
//    cnt = 0;
//  }
//  return cnt;
//}
//static uint8_t hyundai_community_get_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
//  int addr = GET_ADDR(to_push);

//  uint8_t chksum;
//  if (addr == 916) {
//    chksum = GET_BYTE(to_push, 6) & 0xF;
//  } else if (addr == 1057) {
//    chksum = GET_BYTE(to_push, 7) >> 4;
//  } else {
//    chksum = 0;
//  }
//  return chksum;
//}
//static uint8_t hyundai_community_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
//  int addr = GET_ADDR(to_push);

//  uint8_t chksum = 0;
  // same algorithm, but checksum is in a different place
//  for (int i = 0; i < 8; i++) {
//    uint8_t b = GET_BYTE(to_push, i);
//    if (((addr == 916) && (i == 6)) || ((addr == 1057) && (i == 7))) {
//      b &= (addr == 1057) ? 0x0FU : 0xF0U; // remove checksum
//    }
//    chksum += (b % 16U) + (b / 16U);
//  }
//  return (16U - (chksum %  16U)) % 16U;
//}

static int hyundai_community_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  
  UNUSED(to_push);
  return 1;
}
  
static int hyundai_community_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  
  UNUSED(to_send);
  controls_allowed = 1;
  return true;
}
  
static int hyundai_community_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);
  // forward cam to ccan and viceversa, except lkas cmd
  if (!relay_malfunction) {
    if ((bus_num == 0) && (addr != 593)){
      bus_fwd = 2;
    }
    if ((bus_num == 2) && (addr != 832) && (addr != 1157)) {
      if ((addr != 1056) && (addr != 1057) && (addr != 905) && (addr != 1290)) {
        bus_fwd = 0;
      }
    }
  }
  return bus_fwd;
}

//static void hyundai_community_init(int16_t param) {
//  UNUSED(param);
//  controls_allowed = false;
//  relay_malfunction_reset();
//  hyundai_community_non_scc_car = false;
//}

//static void hyundai_community_nonscc_init(int16_t param) {
//  UNUSED(param);
//  controls_allowed = false;
//  relay_malfunction_reset();
//  hyundai_community_non_scc_car = true;
//}

const safety_hooks hyundai_community_hooks = {
//  .init = hyundai_community_init,
  .rx = hyundai_community_rx_hook,
  .tx = hyundai_community_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_community_fwd_hook,
//  .addr_check = hyundai_community_rx_checks,
//  .addr_check_len = sizeof(hyundai_community_rx_checks) / sizeof(hyundai_community_rx_checks[0]),
};

//const safety_hooks hyundai_community_nonscc_hooks = {
// .init = hyundai_community_nonscc_init,
//  .rx = hyundai_community_rx_hook,
//  .tx = hyundai_community_tx_hook,
//  .tx_lin = nooutput_tx_lin_hook,
//  .fwd = hyundai_community_fwd_hook,
//  .addr_check = hyundai_community_nonscc_rx_checks,
//  .addr_check_len = sizeof(hyundai_community_nonscc_rx_checks) / sizeof(hyundai_community_nonscc_rx_checks[0]),
//};
