void default_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {}

int default_ign_hook() {
  return -1; // use GPIO to determine ignition
}

// *** all output safety mode ***

static void alloutput_init(int16_t param) {
  controls_allowed = 1;
}

static int alloutput_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  return true;
}

static int alloutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  return true;
}

static int alloutput_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  return -1;
}

const safety_hooks alloutput_hooks = {
  .init = alloutput_init,
  .rx = default_rx_hook,
  .tx = alloutput_tx_hook,
  .tx_lin = alloutput_tx_lin_hook,
  .ignition = default_ign_hook,
  .fwd = alloutput_fwd_hook,
};