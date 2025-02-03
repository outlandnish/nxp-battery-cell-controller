#include "BatteryCellController.h"
#include "bcc_defs.h"

BatteryCellController::BatteryCellController(SPIClass *spi, bcc_device_t device, uint8_t cell_count, uint8_t enable_pin, uint8_t intb_pin, uint8_t cs_pin, bool loopback) {
  this->spi = spi;

  this->comm_mode = BCC_MODE_TPL;
  this->devices[0] = device;
  this->device_count = 1;
  this->cell_count = cell_count;
  this->loopback = loopback;

  this->enable_pin = enable_pin;
  this->intb_pin = intb_pin;
  this->cs_tx_pin = cs_pin;

  pinMode(this->enable_pin, OUTPUT);
  pinMode(this->intb_pin, INPUT);
  int_state = digitalRead(this->intb_pin);

  attachInterrupt(this->intb_pin, [this]() {
    this->on_interrupt();
  }, CHANGE);
}

BatteryCellController::BatteryCellController(HalfDuplexSPI *bcc_tx, HalfDuplexSPI *bcc_rx, bcc_device_t devices[], uint8_t device_count, uint8_t cell_count, uint8_t enable_pin, uint8_t intb_pin, uint8_t cs_tx_pin, bool loopback) {
  this->bcc_tx = bcc_tx;
  this->bcc_rx = bcc_rx;

  for (int i = 0; i < device_count; i++) {
    this->devices[i] = devices[i];
  }

  this->device_count = device_count;

  this->cell_count = cell_count;
  this->comm_mode = BCC_MODE_TPL;
  this->loopback = loopback;

  this->enable_pin = enable_pin;
  this->intb_pin = intb_pin;
  this->cs_tx_pin = cs_tx_pin;

  pinMode(this->enable_pin, OUTPUT);
  pinMode(this->intb_pin, INPUT);
  int_state = digitalRead(this->intb_pin);

  attachInterrupt(this->intb_pin, [this]() {
    this->on_interrupt();
  }, CHANGE);
}

void BatteryCellController::on_interrupt() {
  int_state = int_state == HIGH ? LOW : HIGH;
  interrupt_events.push(int_state);
}

void BatteryCellController::reset_interrupt_events() {
  while (!interrupt_events.empty()) {
    interrupt_events.pop();
  }
}

bcc_status_t BatteryCellController::begin() {
  uint8_t device;
  bcc_status_t status;

  if ((this->comm_mode != BCC_MODE_SPI) && (this->comm_mode != BCC_MODE_TPL)) {
    return BCC_STATUS_PARAM_RANGE;
  }

  if ((this->device_count == 0) || (this->device_count > ((this->comm_mode == BCC_MODE_SPI) ? BCC_DEVICE_CNT_MAX_SPI : BCC_DEVICE_CNT_MAX_TPL))) {
    return BCC_STATUS_PARAM_RANGE;
  }

  for (device = 0; device < this->device_count; device++) {
    this->message_counter[device] = 0U;
    if (this->devices[device] == BCC_DEVICE_MC33771C) {
      if (!BCC_IS_IN_RANGE(this->cell_count, MC33771C_MIN_CELLS, MC33771C_MAX_CELLS)) {
        return BCC_STATUS_PARAM_RANGE;
      }
      this->cell_map[device] = s_cellMap33771c[this->cells[device]];
    } else if (this->devices[device] == BCC_DEVICE_MC33772C) {
      if (!BCC_IS_IN_RANGE(this->cell_count, MC33772C_MIN_CELLS, MC33772C_MAX_CELLS)) {
        return BCC_STATUS_PARAM_RANGE;
      }
      this->cell_map[device] = s_cellMap33772c[this->cells[device]];
    } else {
      return BCC_STATUS_PARAM_RANGE;
    }
  }
  this->message_counter[this->cell_count] = 0U;

  /* Enable MC33664 device in TPL mode. */
  if (this->comm_mode == BCC_MODE_TPL) {
    if ((status = this->enable_tpl()) != BCC_STATUS_SUCCESS) {
      return status;
    }
  }

  Serial.println("Initializing devices...");

  return init_devices();
}

bcc_status_t BatteryCellController::enable_tpl() {
 int32_t timeout;

  /* Set normal state (transition from low to high). */
  digitalWrite(this->enable_pin, LOW);
  /* Wait at least 100 us. */
  delayMicroseconds(150);
  digitalWrite(this->enable_pin, HIGH);

  /* Note: MC33664 has time t_Ready/t_INTB_PULSE_DELAY (max. 100 us) to take effect.
  * Wait for INTB transition from high to low (max. 100 us). */
  delayMicroseconds(BCC_T_INTB_PULSE_DELAY_US);
  if (interrupt_events.empty()) {
    return BCC_STATUS_COM_TIMEOUT;
  }
  if (interrupt_events.front() != LOW) {
    return BCC_STATUS_COM_TIMEOUT;
  }
  interrupt_events.pop();

  /* Wait for INTB transition from low to high (typ. 100 us).
  * Wait for at most 200 us. */
  delay(BCC_T_INTB_PULSE_US * 2);
  if (interrupt_events.empty()) {
    return BCC_STATUS_COM_TIMEOUT;
  }
  if (interrupt_events.front() != HIGH) {
    return BCC_STATUS_COM_TIMEOUT;
  }
  interrupt_events.pop();

  /* Now the device should be in normal mode (i.e. after INTB low to high
  * transition). For sure wait for 150 us. */
  delayMicroseconds(150);

  return BCC_STATUS_SUCCESS;
}

void BatteryCellController::disable_tpl() {
  digitalWrite(this->enable_pin, LOW);
}

bcc_status_t BatteryCellController::init_devices() {
  uint8_t device;
  bcc_status_t status;

  Serial.println("Waking up devices...");
  wake_up();

  Serial.println("Running software reset...");
  status = software_reset((comm_mode == BCC_MODE_TPL) ? BCC_CID_UNASSIG : BCC_CID_DEV1);
  // if (status != BCC_STATUS_SUCCESS) {
  //   return status;
  // }

  delay(BCC_T_VPWR_READY_MS);

  Serial.println("Assigning CIDs...");
  status = assign_cid(BCC_CID_DEV1);
  if (status != BCC_STATUS_SUCCESS) {
    return status;
  }

  for (device = 1; device < cell_count; device++) {
    delay(2);

    wake_up_pattern_tpl();

    status = assign_cid((bcc_cid_t)(device + 1));
    if (status != BCC_STATUS_SUCCESS) {
      return status;
    }
  }

  return status;
}

bcc_status_t BatteryCellController::software_reset(bcc_cid_t cid) {
  if ((((uint8_t)cid) > device_count) ||
    ((cid == BCC_CID_UNASSIG) && (comm_mode == BCC_MODE_SPI)))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  if (cid == BCC_CID_UNASSIG) {
    return write_register_global(MC33771C_INIT_OFFSET, MC33771C_SYS_CFG1_SOFT_RST(MC33771C_SYS_CFG1_SOFT_RST_ACTIVE_ENUM_VAL));
  }
  else {
    return write_register(cid, MC33771C_INIT_OFFSET, MC33771C_SYS_CFG1_SOFT_RST(MC33771C_SYS_CFG1_SOFT_RST_ACTIVE_ENUM_VAL));
  }
}

void BatteryCellController::wake_up_pattern_spi() {
  digitalWrite(this->cs_tx_pin, LOW);
  delayMicroseconds(BCC_CSB_WU_FLT_US);

  digitalWrite(this->cs_tx_pin, HIGH);
  delayMicroseconds(BCC_T_WAKE_UP_US);
}

void BatteryCellController::wake_up_pattern_tpl() {
  digitalWrite(this->cs_tx_pin, LOW);
  delayMicroseconds(BCC_WAKE_PULSE_US);

  digitalWrite(this->cs_tx_pin, HIGH);
  delayMicroseconds(BCC_T_WAKE_DELAY_US);

  digitalWrite(this->cs_tx_pin, LOW);
  delayMicroseconds(BCC_WAKE_PULSE_US);

  digitalWrite(this->cs_tx_pin, HIGH);
  delayMicroseconds(BCC_T_WU_WAIT_US * this->cell_count);
}

void BatteryCellController::wake_up() {
  if (this->comm_mode == BCC_MODE_SPI) {
    wake_up_pattern_spi();
  } else {
    wake_up_pattern_tpl();
  }
}

bcc_status_t BatteryCellController::assign_cid(bcc_cid_t cid) {
  uint16_t writeVal, readVal;
  bcc_status_t status;

  /* Check if unassigned node replies. This is the first reading after device
     * reset. */
  /* Note: In SPI communication mode, the device responds with all bit filed
    * set to zero except message counter and the correct CRC to the very first
    * MCU <-> MC33771C/772C message. */
  status = read_register(BCC_CID_UNASSIG, MC33771C_INIT_OFFSET, 1U, &readVal);
  if ((status != BCC_STATUS_SUCCESS) && (status != BCC_STATUS_COM_NULL)) {
    return status;
  }

  /* Assign CID;
  * Terminate RDTX_OUT of the last node in TPL setup without loop-back.
  * Stop forwarding only for MC33772C in TPL setup with one node and no
  * loop-back. RDTX_OUT should not be terminated in this case. */
  writeVal = MC33771C_INIT_CID(cid) |
    MC33771C_INIT_RDTX_OUT(MC33771C_INIT_RDTX_OUT_DISABLED_ENUM_VAL);
  if ((comm_mode == BCC_MODE_TPL) &&
    (device_count == 1U) &&
    (!loopback) &&
    (devices[(uint8_t)cid - 1] == BCC_DEVICE_MC33772C))
  {
    writeVal |= MC33772C_INIT_TPL2_TX_TERM(MC33772C_INIT_TPL2_TX_TERM_DISABLED_ENUM_VAL) |
      MC33772C_INIT_BUS_FW(MC33772C_INIT_BUS_FW_DISABLED_ENUM_VAL);
  }
  else if ((comm_mode == BCC_MODE_TPL) &&
    ((uint8_t)cid == device_count) &&
    (!loopback))
  {
    writeVal |= MC33771C_INIT_RDTX_OUT(MC33771C_INIT_RDTX_OUT_ENABLED_ENUM_VAL);

    if (devices[(uint8_t)cid - 1] == BCC_DEVICE_MC33772C)
    {
      writeVal |= MC33772C_INIT_BUS_FW(MC33772C_INIT_BUS_FW_ENABLED_ENUM_VAL);
    }
  }
  else
  {
    writeVal |= MC33771C_INIT_RDTX_OUT(MC33771C_INIT_RDTX_OUT_DISABLED_ENUM_VAL);

    if (devices[(uint8_t)cid - 1] == BCC_DEVICE_MC33772C)
    {
      writeVal |= MC33772C_INIT_BUS_FW(MC33772C_INIT_BUS_FW_ENABLED_ENUM_VAL);
    }
  }

  status = write_register(cid, MC33771C_INIT_OFFSET, writeVal);
  if (status == BCC_STATUS_SUCCESS) {
    message_counter[(uint8_t)cid] = message_counter[0];

    status = read_register(cid, MC33771C_INIT_OFFSET, 1U, &readVal);

    if ((status == BCC_STATUS_SUCCESS) && (writeVal != readVal)) {
      status = BCC_STATUS_SPI_FAIL;
    }
  }

  if (status != BCC_STATUS_SUCCESS) {
    delayMicroseconds(750U);

    status = write_register(BCC_CID_UNASSIG, MC33771C_INIT_OFFSET, writeVal);
    if (status == BCC_STATUS_SUCCESS) {
      message_counter[(uint8_t)cid] = message_counter[0];

      status = read_register(cid, MC33771C_INIT_OFFSET, 1U, &readVal);

      if ((status == BCC_STATUS_SUCCESS) && (writeVal != readVal)) {
        status = BCC_STATUS_SPI_FAIL;
      }
    }
  }

  return status;
}

bcc_status_t BatteryCellController::read_register(bcc_cid_t cid, uint8_t reg_addr, uint8_t reg_cnt, uint16_t *reg_val) {
  bcc_status_t status;

  switch (comm_mode) {
    case BCC_MODE_SPI:
      status = read_register_spi(cid, reg_addr, reg_cnt, reg_val);
      break;
    case BCC_MODE_TPL:
      status = read_register_tpl(cid, reg_addr, reg_cnt, reg_val);
      break;
  }

  return status;
}

bcc_status_t BatteryCellController::write_register(bcc_cid_t cid, uint8_t reg_addr, uint16_t reg_val) {
  bcc_status_t status;

  switch (comm_mode) {
    case BCC_MODE_SPI:
      status = write_register_spi(cid, reg_addr, reg_val);
      break;
    case BCC_MODE_TPL:
      status = write_register_tpl(cid, reg_addr, reg_val);
      break;
  }

  return status;
}

bcc_status_t BatteryCellController::write_register_global(uint8_t reg_addr, uint16_t reg_val) {
  bcc_status_t status;

  assert_param(comm_mode == BCC_MODE_TPL);

  return write_global_tpl(reg_addr, reg_val);
}

bcc_status_t BatteryCellController::update_register(bcc_cid_t cid, uint8_t reg_addr, uint16_t mask, uint16_t val) {
  uint16_t temp_reg_value;
  bcc_status_t status;

  if (((uint8_t)cid) > device_count)
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, reg_addr, 1U, &temp_reg_value);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Update register value. */
  temp_reg_value = temp_reg_value & ~(mask);
  temp_reg_value = temp_reg_value | (val & mask);

  return write_register(cid, reg_addr, temp_reg_value);
}

void BatteryCellController::pack_frame(uint16_t data, uint8_t addr, bcc_cid_t cid, uint8_t cmd_count, uint8_t *frame) {
  assert_param(frame != NULL);

   /* Register Data field. */
  frame[BCC_MSG_IDX_DATA_H] = (uint8_t)(data >> 8U);
  frame[BCC_MSG_IDX_DATA_L] = (uint8_t)(data & 0xFFU);

  /* Register Address field. Master/Slave field is always 0 for sending. */
  frame[BCC_MSG_IDX_ADDR] = (addr & BCC_MSG_ADDR_MASK);

  /* Device address (Cluster ID) field. */
  frame[BCC_MSG_IDX_CID] = ((uint8_t)cid & 0x3FU);

  /* Message counter and Command fields. */
  frame[BCC_MSG_IDX_CNT_CMD] = (cmd_count & 0xF3U);

  /* CRC field. */
  frame[BCC_MSG_IDX_CRC] = calculate_crc(frame);
}

uint8_t BatteryCellController::calculate_crc(uint8_t *data) {
  uint8_t crc;      /* Result. */
  uint8_t tableIdx; /* Index to the CRC table. */

  crc = 0x42U;

  tableIdx = crc ^ data[BCC_MSG_IDX_DATA_H];
  crc = s_crcTable[tableIdx];
  tableIdx = crc ^ data[BCC_MSG_IDX_DATA_L];
  crc = s_crcTable[tableIdx];
  tableIdx = crc ^ data[BCC_MSG_IDX_ADDR];
  crc = s_crcTable[tableIdx];
  tableIdx = crc ^ data[BCC_MSG_IDX_CID];
  crc = s_crcTable[tableIdx];
  tableIdx = crc ^ data[BCC_MSG_IDX_CNT_CMD];
  crc = s_crcTable[tableIdx];

  return crc;
}

bcc_status_t BatteryCellController::check_crc(uint8_t *data) {
  uint8_t crc; /* Calculated CRC. */
  crc = calculate_crc(data);

  return (crc == data[BCC_MSG_IDX_CRC]) ? BCC_STATUS_SUCCESS : BCC_STATUS_COM_CRC;
}

bcc_status_t BatteryCellController::check_message_counter(bcc_cid_t cid, uint8_t *data) {
  uint8_t msg_count_previous; /* Previously received message counter value. */
  uint8_t msg_count_received; /* Currently received message counter value. */

  assert_param(data != NULL);

  msg_count_previous = message_counter[(uint8_t)cid];
  msg_count_received = (data[BCC_MSG_IDX_CNT_CMD] & BCC_MSG_MSG_CNT_MASK) >> BCC_MSG_MSG_CNT_SHIFT;

  /* Store the Message counter value. */
  message_counter[(uint8_t)cid] = BCC_INC_MSG_CNTR(msg_count_received);

  /* Check the Message counter value.
  * Note: Do not perform a check for CID=0. */
  if ((cid != BCC_CID_UNASSIG) && (msg_count_received != msg_count_previous)) {
    return BCC_STATUS_COM_MSG_CNT;
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::check_echo_frame(uint8_t tx[], uint8_t rx[]) {
  assert_param(tx != NULL);
  assert_param(rx != NULL);

  if ((tx[BCC_MSG_IDX_DATA_H] == rx[BCC_MSG_IDX_DATA_H]) &&
    (tx[BCC_MSG_IDX_DATA_L] == rx[BCC_MSG_IDX_DATA_L]) &&
    (tx[BCC_MSG_IDX_ADDR] == rx[BCC_MSG_IDX_ADDR]) &&
    (tx[BCC_MSG_IDX_CID] == rx[BCC_MSG_IDX_CID]) &&
    (tx[BCC_MSG_IDX_CNT_CMD] == rx[BCC_MSG_IDX_CNT_CMD]) &&
    (tx[BCC_MSG_IDX_CRC] == rx[BCC_MSG_IDX_CRC]))
  {
      return BCC_STATUS_SUCCESS;
  }
  else
  {
      return BCC_STATUS_COM_ECHO;
  }
}

bcc_status_t BatteryCellController::read_register_spi(bcc_cid_t cid, uint8_t reg_addr, uint8_t reg_count, uint16_t *reg_val) {
  bcc_status_t status;
  uint8_t tx_buffer[BCC_MSG_SIZE];
  uint8_t rx_buffer[BCC_MSG_SIZE];
  uint8_t reg_index;

  if (((uint8_t)cid > device_count) || (reg_addr > BCC_MAX_REG_ADDR) ||
    (reg_count == 0U) || ((reg_addr + reg_count - 1U) > BCC_MAX_REG_ADDR))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Create frame for request. */
  pack_frame(0x0001U, reg_addr, cid, BCC_CMD_READ, tx_buffer);

  /* Send request for data. Required data are returned with the following transfer. */
  status = transfer_spi(tx_buffer, rx_buffer);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Check CRC of the response. */
  if ((status = check_crc(rx_buffer)) != BCC_STATUS_SUCCESS) {
    return status;
  }

  if ((status = check_message_counter(cid, rx_buffer)) != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  if (BCC_IS_NULL_RESP(rx_buffer))
  {
    return BCC_STATUS_COM_NULL;
  }

  /* Read required data. */
  for (reg_index = 0U; reg_index < reg_count; reg_index++)
  {
    /* Increment address of the register to be read. */
    reg_addr++;
    if (reg_addr > 0x7FU)
    {
      reg_addr = 0x00U;
    }

    pack_frame(0x0001U, reg_addr, cid, BCC_CMD_READ, tx_buffer);

    /* Send request for data. Required data are returned with the following transfer. */
    status = transfer_spi(tx_buffer, rx_buffer);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }

    /* Check CRC. */
    if ((status = check_crc(rx_buffer)) != BCC_STATUS_SUCCESS)
    {
      return status;
    }

    /* Check the Message counter value. */
    if ((status = check_message_counter(cid, rx_buffer)) != BCC_STATUS_SUCCESS)
    {
      return status;
    }

    if (BCC_IS_NULL_RESP(rx_buffer))
    {
      return BCC_STATUS_COM_NULL;
    }

    /* Store data. */
    *reg_val++ = BCC_GET_MSG_DATA(rx_buffer);
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::write_register_spi(bcc_cid_t cid, uint8_t reg_addr, uint16_t reg_val) {
  bcc_status_t status;
  uint8_t tx_buffer[BCC_MSG_SIZE];
  uint8_t rx_buffer[BCC_MSG_SIZE];
  
  if (((uint8_t)cid > device_count) || (reg_addr > BCC_MAX_REG_ADDR))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Create frame for writing. */
  pack_frame(reg_val, reg_addr, cid, BCC_CMD_WRITE, tx_buffer);
  status = transfer_spi(tx_buffer, rx_buffer);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Check CRC */
  if ((status = check_crc(rx_buffer)) != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Check message counter */
  if ((status = check_message_counter(cid, rx_buffer)) != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Check whether all fields except CRC and message counter are zero */
  if (BCC_IS_NULL_RESP(rx_buffer))
  {
    return BCC_STATUS_COM_NULL;
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::read_register_tpl(bcc_cid_t cid, uint8_t reg_addr, uint8_t reg_count, uint16_t *reg_val) {
  uint8_t tx_buffer[BCC_MSG_SIZE];
  uint8_t *rx_buffer;
  uint8_t reg_index;
  bcc_status_t status;

  if (((uint8_t)cid > device_count) || (reg_addr > BCC_MAX_REG_ADDR) ||
    (reg_count == 0U) || ((reg_addr + reg_count - 1U) > BCC_MAX_REG_ADDR))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Create frame for request. */
  pack_frame(0x0001U, reg_addr, cid, BCC_CMD_READ, tx_buffer);

  /* Pointer to begnning of the received frame */
  rx_buffer = this->rx_buffer;

  status = transfer_tpl(tx_buffer, rx_buffer, reg_count);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Check the echo frame */
  status = check_echo_frame(tx_buffer, rx_buffer);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Check and store responses */
  for (reg_index = 0U; reg_index < reg_count; reg_index++)
  {
    rx_buffer += BCC_MSG_SIZE;

    /* Check CRC. */
    if ((status = check_crc(rx_buffer)) != BCC_STATUS_SUCCESS)
    {
        return status;
    }

    /* Check the Message counter value. */
    if ((status = check_message_counter(cid, rx_buffer)) != BCC_STATUS_SUCCESS)
    {
        return status;
    }

    /* Store data. */
    *reg_val++ = BCC_GET_MSG_DATA(rx_buffer);
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::write_register_tpl(bcc_cid_t cid, uint8_t reg_addr, uint16_t reg_val) {
  uint8_t tx_buffer[BCC_MSG_SIZE];
  bcc_status_t status;

  if (((uint8_t)cid > device_count) || (reg_addr > BCC_MAX_REG_ADDR))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Create frame for writing. */
  pack_frame(reg_val, reg_addr, cid, BCC_CMD_WRITE, tx_buffer);

  status = transfer_tpl(tx_buffer, this->rx_buffer, 1);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  return check_echo_frame(tx_buffer, this->rx_buffer);
}

bcc_status_t BatteryCellController::write_global_tpl(uint8_t reg_addr, uint16_t reg_val) {
  uint8_t tx_buffer[BCC_MSG_SIZE]; /* Transmission buffer. */
  bcc_status_t status;

  /* Check input parameters. */
  if (reg_addr > BCC_MAX_REG_ADDR)
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Create frame for writing. */
  pack_frame(reg_val, reg_addr, BCC_CID_UNASSIG, BCC_CMD_GLOB_WRITE, tx_buffer);

  // print out packed frame
  Serial.print("PACK: ");
  for (int i = 0; i < BCC_MSG_SIZE; i++) {
    Serial.print(tx_buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  status = transfer_tpl(tx_buffer, this->rx_buffer, 1);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Check the echo frame. */
  return check_echo_frame(tx_buffer, this->rx_buffer);
}

bcc_status_t BatteryCellController::transfer_spi(uint8_t tx[], uint8_t rx[]) {
  assert_param(tx != NULL);
  assert_param(rx != NULL);

  auto error = spi->transfer(tx, rx, SPI_ALIGNMENT);
  if (error != SPI_OK)
  {
    return (error == SPI_TIMEOUT) ? BCC_STATUS_COM_TIMEOUT : BCC_STATUS_SPI_FAIL;
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::transfer_tpl(uint8_t tx[], uint8_t rx[], uint32_t rx_transfer_count) {
  bcc_status_t status;
  uint32_t rxTimeout = BCC_RX_COM_TIMEOUT_MS * 1000;

  assert_param(tx != NULL);
  assert_param(rx != NULL);
  assert_param(rx_transfer_count > 0);

  /* Transmissions at RX and TX SPI occur almost at the same time. Start
    * reading (response) at RX SPI first. */
  auto error = bcc_rx->receiveAsync(rx, rx_transfer_count * SPI_ALIGNMENT);
  if (error != HAL_OK)
  {
    Serial.println("Error in starting RX transmission");
    return (error == HAL_TIMEOUT) ? BCC_STATUS_COM_TIMEOUT : BCC_STATUS_SPI_FAIL;
  }
  
  // print out data frame
  Serial.print("TX: ");
  for (int i = 0; i < BCC_MSG_SIZE; i++) {
    Serial.print(tx[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  /* Send data via TX SPI. */
  digitalWrite(this->cs_tx_pin, LOW);
  error = bcc_tx->transmit(tx, SPI_ALIGNMENT, BCC_TX_COM_TIMEOUT_MS);
  digitalWrite(this->cs_tx_pin, HIGH);
  if (error != HAL_OK)
  {
    Serial.println("Error in TX transmission");
    /* Cancel reception of data. */
    bcc_rx->abortTransfer();
    return (error == HAL_TIMEOUT) ? BCC_STATUS_COM_TIMEOUT : BCC_STATUS_SPI_FAIL;
  } 

  /* Wait until RX transmission finished. */
  while (bcc_rx->getState() != HAL_SPI_STATE_READY && rxTimeout > 0) {
    HAL_Delay(1); // Small delay to avoid busy-waiting
    rxTimeout -= 1000; // Decrease timeout (microseconds -> milliseconds)
  }

  if (rxTimeout <= 0 && bcc_rx->getState() != HAL_SPI_STATE_READY) {
    Serial.println("Timeout in waiting for RX transmission");
    bcc_rx->abortTransfer();
    return BCC_STATUS_COM_TIMEOUT;
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::send_nop_tpl(bcc_cid_t cid) {
  uint8_t txBuf[BCC_MSG_SIZE]; /* Transmission buffer. */
  bcc_status_t status;

  if ((cid == BCC_CID_UNASSIG) || ((uint8_t)cid > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Create frame for writing.
  * Note: Register Data, Register Address and Message counter fields can
  * contain any value. */
  pack_frame(0x0000U, 0x00U, cid, BCC_CMD_NOOP, txBuf);

  status = transfer_tpl(txBuf, rx_buffer, 1);
  if (status != BCC_STATUS_SUCCESS)
  {
      return status;
  }

  /* Check the echo frame. */
  return check_echo_frame(txBuf, rx_buffer);
}

bcc_status_t BatteryCellController::send_nop_spi(bcc_cid_t cid) {
  uint8_t tx[BCC_MSG_SIZE]; /* Transmission buffer. */
  uint8_t rx[BCC_MSG_SIZE]; /* Buffer for receiving. */
  bcc_status_t status;

  if ((cid == BCC_CID_UNASSIG) || ((uint8_t)cid > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Create frame for writing.
  * Note: Register Data, Register Address and Message counter fields can
  * contain any value. */
  pack_frame(0x0000U, 0x00U, cid, BCC_CMD_NOOP, tx);

  status = transfer_spi(tx, rx);
  if (status != BCC_STATUS_SUCCESS)
  {
      return status;
  }

  /* Check CRC. */
  if ((status = check_crc(rx)) != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Check message counter. */
  if ((status = check_message_counter(cid, rx)) != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Check whether all field except CRC and message counter are zero. */
  if (BCC_IS_NULL_RESP(rx))
  {
    return BCC_STATUS_COM_NULL;
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::send_nop(bcc_cid_t cid) {
  uint8_t tx_buffer[BCC_MSG_SIZE];
  uint8_t rx_buffer[BCC_MSG_SIZE];
  bcc_status_t status;
  

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::sleep() {
  if (comm_mode == BCC_MODE_SPI) {
    return write_register(BCC_CID_DEV1, MC33771C_SYS_CFG_GLOBAL_OFFSET,
      MC33771C_SYS_CFG_GLOBAL_GO2SLEEP(MC33771C_SYS_CFG_GLOBAL_GO2SLEEP_ENABLED_ENUM_VAL));
  }
  else {
    return write_register_global(MC33771C_SYS_CFG_GLOBAL_OFFSET,
      MC33771C_SYS_CFG_GLOBAL_GO2SLEEP(MC33771C_SYS_CFG_GLOBAL_GO2SLEEP_ENABLED_ENUM_VAL));
  }
}

bcc_status_t BatteryCellController::start_conversion_async(bcc_cid_t cid, bcc_avg_t average) {
  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) || (average > BCC_AVG_256))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  return update_register(cid, MC33771C_ADC_CFG_OFFSET,
    MC33771C_ADC_CFG_SOC_MASK | MC33771C_ADC_CFG_AVG_MASK,
    MC33771C_ADC_CFG_SOC(MC33771C_ADC_CFG_SOC_ENABLED_ENUM_VAL) |
    MC33771C_ADC_CFG_AVG(average));
}

bcc_status_t BatteryCellController::start_conversion_global_async(uint16_t adc_config_value) {
  assert_param(comm_mode == BCC_MODE_TPL);

  /* Set Start of Conversion bit in case it is not. */
  adc_config_value |= MC33771C_ADC_CFG_SOC(MC33771C_ADC_CFG_SOC_ENABLED_ENUM_VAL);

  return write_register_global(MC33771C_ADC_CFG_OFFSET, adc_config_value);
}

bcc_status_t BatteryCellController::check_conversion_completed(bcc_cid_t cid, bool *completed) {
  uint16_t adc_config_value;
  bcc_status_t status;

  assert_param(completed != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, MC33771C_ADC_CFG_OFFSET, 1U, &adc_config_value);

  *(completed) = ((adc_config_value & MC33771C_ADC_CFG_EOC_N_MASK) ==
          MC33771C_ADC_CFG_EOC_N(MC33771C_ADC_CFG_EOC_N_COMPLETED_ENUM_VAL));

  return status;
}

bcc_status_t BatteryCellController::start_conversion(bcc_cid_t cid, bcc_avg_t average) {
  bool complete;           /* Conversion complete flag. */
  bcc_status_t status;

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) || (average > BCC_AVG_256))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  status = start_conversion_async(cid, average);
  if (status != BCC_STATUS_SUCCESS)
  {
      return status;
  }

  /* Wait for at least 520 us (16-bit conversion) before polling bit EOC_N
    * to avoid any traffic on the communication bus during conversion. */
  delayMicroseconds(((uint32_t)BCC_T_EOC_TYP_US) << ((uint8_t)average));

  status = start_timeout(
    (((uint32_t)BCC_T_EOC_TIMEOUT_US) << ((uint8_t)average)) -
    (((uint32_t)BCC_T_EOC_TYP_US) << ((uint8_t)average)));
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  do
  {
    status = check_conversion_completed(cid, &complete);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }
  } while ((!complete) && (!has_timer_expired()));

  /* Check once more after timeout expiration because the read command takes
    * several tens/hundreds of microseconds (depends on user code efficiency)
    * and the last read command could be done relatively long before the
    * timeout expiration. */
  if (!complete)
  {
    status = check_conversion_completed(cid, &complete);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }
  }

  return (complete) ? BCC_STATUS_SUCCESS : BCC_STATUS_COM_TIMEOUT;
}

bcc_status_t BatteryCellController::get_raw_values(bcc_cid_t cid, uint16_t *measurements) {
  bcc_status_t status;
  uint8_t i;

  assert_param(measurements != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Read all the measurement registers.
    * Note: the order and number of registers conforms to the order of measured
    * values in Measurements array, see enumeration bcc_measurements_t. */
  if (devices[(uint8_t)cid - 1] == BCC_DEVICE_MC33771C)
  {
    status = read_register(cid, MC33771C_CC_NB_SAMPLES_OFFSET,
      BCC_MEAS_CNT, measurements);
  }
  else
  {
    status = read_register(cid, MC33772C_CC_NB_SAMPLES_OFFSET,
      (MC33772C_MEAS_STACK_OFFSET - MC33772C_CC_NB_SAMPLES_OFFSET) + 1, measurements);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }

    /* Skip the reserved registers to speed-up this function. */
    measurements[BCC_MSR_CELL_VOLT14] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT13] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT12] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT11] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT10] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT9] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT8] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT7] = 0x0000;

    status = read_register(cid, MC33772C_MEAS_CELL6_OFFSET,
      (MC33772C_MEAS_VBG_DIAG_ADC1B_OFFSET - MC33772C_MEAS_CELL6_OFFSET) + 1,
      (uint16_t *)(measurements + ((uint8_t)BCC_MSR_CELL_VOLT6)));
  }

  /* Mask the read registers.
    * Note: Nothing to mask in CC_NB_SAMPLES, COULOMB_CNT1 and COULOMB_CNT2
    * registers. */
  measurements[BCC_MSR_ISENSE1] &= MC33771C_MEAS_ISENSE1_MEAS_I_MSB_MASK;
  measurements[BCC_MSR_ISENSE2] &= MC33771C_MEAS_ISENSE2_MEAS_I_LSB_MASK;

  for (i = (uint8_t)BCC_MSR_STACK_VOLT; i < BCC_MEAS_CNT; i++)
  {
    measurements[i] &= MC33771C_MEAS_STACK_MEAS_STACK_MASK;
  }

  return status;
}

bcc_status_t BatteryCellController::get_raw_values(bcc_cid_t cid, uint16_t *measurements) {
  bcc_status_t status;

  uint8_t i;

  assert_param(measurements != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Read all the measurement registers.
    * Note: the order and number of registers conforms to the order of measured
    * values in Measurements array, see enumeration bcc_measurements_t. */
  if (devices[(uint8_t)cid - 1] == BCC_DEVICE_MC33771C)
  {
    status = read_register(cid, MC33771C_CC_NB_SAMPLES_OFFSET,
      BCC_MEAS_CNT, measurements);
  }
  else
  {
    status = read_register(cid, MC33772C_CC_NB_SAMPLES_OFFSET,
      (MC33772C_MEAS_STACK_OFFSET - MC33772C_CC_NB_SAMPLES_OFFSET) + 1, measurements);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }

    /* Skip the reserved registers to speed-up this function. */
    measurements[BCC_MSR_CELL_VOLT14] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT13] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT12] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT11] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT10] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT9] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT8] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT7] = 0x0000;

    status = read_register(cid, MC33772C_MEAS_CELL6_OFFSET,
      (MC33772C_MEAS_VBG_DIAG_ADC1B_OFFSET - MC33772C_MEAS_CELL6_OFFSET) + 1,
      (uint16_t *)(measurements + ((uint8_t)BCC_MSR_CELL_VOLT6)));
  }

  /* Mask the read registers.
    * Note: Nothing to mask in CC_NB_SAMPLES, COULOMB_CNT1 and COULOMB_CNT2
    * registers. */
  measurements[BCC_MSR_ISENSE1] &= MC33771C_MEAS_ISENSE1_MEAS_I_MSB_MASK;
  measurements[BCC_MSR_ISENSE2] &= MC33771C_MEAS_ISENSE2_MEAS_I_LSB_MASK;

  for (i = (uint8_t)BCC_MSR_STACK_VOLT; i < BCC_MEAS_CNT; i++)
  {
    measurements[i] &= MC33771C_MEAS_STACK_MEAS_STACK_MASK;
  }

  return status;
}

bcc_status_t BatteryCellController::get_coulomb_counter(bcc_cid_t cid, bcc_cc_data_t* coulomb_counter) {
  bcc_status_t status;
  uint16_t readVal[3];

  assert_param(coulomb_counter != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, MC33771C_CC_NB_SAMPLES_OFFSET, 3U, readVal);
  if (status != BCC_STATUS_SUCCESS)
  {
      return status;
  }

  coulomb_counter->nbSamples = readVal[0];
  coulomb_counter->ccAccumulator = BCC_GET_COULOMB_CNT(readVal[1], readVal[2]);

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_current_sense_voltage(bcc_cid_t cid, int32_t *current_sense_voltage) {
  bcc_status_t status;
  uint16_t readVal[2];

  assert_param(current_sense_voltage != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, MC33771C_MEAS_ISENSE1_OFFSET, 2U, readVal);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  if ((readVal[0] & readVal[1] & MC33771C_MEAS_ISENSE1_DATA_RDY_MASK) == 0U)
  {
    return BCC_STATUS_DATA_RDY;
  }

  *current_sense_voltage = BCC_GET_ISENSE_VOLT(readVal[0], readVal[1]);

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_stack_voltage(bcc_cid_t cid, uint32_t *stack_voltage) {
  bcc_status_t status;
  uint16_t readVal;

  assert_param(stack_voltage != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, MC33771C_MEAS_STACK_OFFSET, 1U, &readVal);
  if (status != BCC_STATUS_SUCCESS)
  {
      return status;
  }

  if ((readVal & MC33771C_MEAS_STACK_DATA_RDY_MASK) == 0U)
  {
      return BCC_STATUS_DATA_RDY;
  }

  *stack_voltage = BCC_GET_STACK_VOLT(readVal);

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_cell_voltages(bcc_cid_t cid, uint32_t *cell_voltages) {
  bcc_status_t status;
  uint16_t read_values[BCC_MAX_CELLS];
  uint8_t i, cell_count;

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  cell_count = BCC_MAX_CELLS_DEV(devices[(uint8_t)cid - 1]);

  /* Read the measurement registers. */
  status = read_register(cid,
                        (devices[(uint8_t)cid - 1] == BCC_DEVICE_MC33771C) ?
                              MC33771C_MEAS_CELL14_OFFSET : MC33771C_MEAS_CELL6_OFFSET,
                        cell_count, read_values);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Convert measurements to [uV], change the cell order and check the data-ready flag. */
  for (i = 0; i < cell_count; i++)
  {
    cell_voltages[cell_count - (i + 1)] = BCC_GET_VOLT(read_values[i]);
    read_values[0] &= read_values[i];
  }

  if ((read_values[0] & MC33771C_MEAS_CELL1_DATA_RDY_MASK) == 0U)
  {
    return BCC_STATUS_DATA_RDY;
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_cell_voltage(bcc_cid_t cid, uint8_t cell_index, uint32_t *cell_voltage) {
  bcc_status_t status;
  uint16_t read_value;

  assert_param(cell_voltage != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
      (cell_index >= BCC_MAX_CELLS_DEV(devices[(uint8_t)cid - 1])))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, MC33771C_MEAS_CELL1_OFFSET - cell_index, 1U, &read_value);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  if ((read_value & MC33771C_MEAS_CELL1_DATA_RDY_MASK) == 0U)
  {
      return BCC_STATUS_DATA_RDY;
  }

  *cell_voltage = BCC_GET_VOLT(read_value);

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_an_voltages(bcc_cid_t cid, uint32_t *an_voltages) {
  bcc_status_t status;
  uint16_t read_values[BCC_GPIO_INPUT_CNT];
  uint8_t i;

  assert_param(an_voltages != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Read the measurement registers. */
  status = read_register(cid, MC33771C_MEAS_AN6_OFFSET,
    BCC_GPIO_INPUT_CNT, read_values);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Convert measurements to [uV] and check the data-ready flag. */
  for (i = 0; i < BCC_GPIO_INPUT_CNT; i++)
  {
    an_voltages[i] = BCC_GET_VOLT(read_values[i]);
    read_values[0] &= read_values[i];
  }

  if ((read_values[0] & MC33771C_MEAS_AN0_DATA_RDY_MASK) == 0U)
  {
    return BCC_STATUS_DATA_RDY;
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_an_voltage(bcc_cid_t cid, uint8_t an_index, uint32_t *an_voltage) {
  bcc_status_t status;
  uint16_t read_value;

  assert_param(an_voltage != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
    (an_index >= BCC_GPIO_INPUT_CNT))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, MC33771C_MEAS_AN0_OFFSET - an_index, 1U, &read_value);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  if ((read_value & MC33771C_MEAS_AN0_DATA_RDY_MASK) == 0U)
  {
    return BCC_STATUS_DATA_RDY;
  }

  *an_voltage = BCC_GET_VOLT(read_value);

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_ic_temperature(bcc_cid_t cid, bcc_temp_unit_t unit, int16_t *ic_temperature) {
  bcc_status_t status;
  uint16_t read_value;

  assert_param(ic_temperature != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
    (unit > BCC_TEMP_FAHRENHEIT))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, MC33771C_MEAS_IC_TEMP_OFFSET, 1U, &read_value);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  if ((read_value & MC33771C_MEAS_IC_TEMP_DATA_RDY_MASK) == 0U)
  {
    return BCC_STATUS_DATA_RDY;
  }

  if (unit == BCC_TEMP_CELSIUS)
  {
    *ic_temperature = BCC_GET_IC_TEMP_C(read_value);
  }
  else if (unit == BCC_TEMP_FAHRENHEIT)
  {
    *ic_temperature = BCC_GET_IC_TEMP_F(read_value);
  }
  else
  {
    *ic_temperature = BCC_GET_IC_TEMP_K(read_value);
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_fault_status(bcc_cid_t cid, uint16_t *fault_status) {
  bcc_status_t status;

  assert_param(fault_status != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  /* Read CELL_OV_FLT and CELL_UV_FLT. */
  status = read_register(cid, MC33771C_CELL_OV_FLT_OFFSET, 2U, &fault_status[BCC_FS_CELL_OV]);
  if (status != BCC_STATUS_SUCCESS)
  {
      return status;
  }

  /* Read CB_OPEN_FLT, CB_SHORT_FLT. */
  status = read_register(cid, MC33771C_CB_OPEN_FLT_OFFSET, 2U, &fault_status[BCC_FS_CB_OPEN]);
  if (status != BCC_STATUS_SUCCESS)
  {
      return status;
  }

  /* Read GPIO_STS, AN_OT_UT_FLT, GPIO_SHORT_Anx_OPEN_STS. */
  status = read_register(cid, MC33771C_GPIO_STS_OFFSET, 3U, &fault_status[BCC_FS_GPIO_STATUS]);
  if (status != BCC_STATUS_SUCCESS)
  {
      return status;
  }

  /* Read COM_STATUS, FAULT1_STATUS, FAULT2_STATUS and FAULT3_STATUS. */
  return read_register(cid,MC33771C_COM_STATUS_OFFSET, 4U, &fault_status[BCC_FS_COMM]);
}

bcc_status_t BatteryCellController::clear_fault_status(bcc_cid_t cid, bcc_fault_status_t fault_status) {
  /* This array is intended for conversion of bcc_fault_status_t value to
    * a BCC register address. */
  const uint8_t regAddrMap[BCC_STAT_CNT] = {
    MC33771C_CELL_OV_FLT_OFFSET, MC33771C_CELL_UV_FLT_OFFSET,
    MC33771C_CB_OPEN_FLT_OFFSET, MC33771C_CB_SHORT_FLT_OFFSET,
    MC33771C_GPIO_STS_OFFSET, MC33771C_AN_OT_UT_FLT_OFFSET,
    MC33771C_GPIO_SHORT_ANX_OPEN_STS_OFFSET, MC33771C_COM_STATUS_OFFSET,
    MC33771C_FAULT1_STATUS_OFFSET, MC33771C_FAULT2_STATUS_OFFSET,
    MC33771C_FAULT3_STATUS_OFFSET
  };

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
    ((uint32_t)fault_status >= BCC_STAT_CNT))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  return write_register(cid, regAddrMap[fault_status], 0x0000U);
}

bcc_status_t BatteryCellController::set_gpio_mode(bcc_cid_t cid, uint8_t gpio_selection, bcc_pin_mode_t mode) {
  bcc_status_t status = BCC_STATUS_PARAM_RANGE;

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
    (gpio_selection >= BCC_GPIO_INPUT_CNT))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  if ((mode == BCC_PIN_WAKE_UP_IN) && (gpio_selection == 0U))
  {
    /* Set GPIO0 to digital input and enable the wake-up capability. */
    status = set_gpio_config(cid, 0U, BCC_PIN_DIGITAL_IN);
    if (status == BCC_STATUS_SUCCESS)
    {
      status = update_register(cid,
        MC33771C_GPIO_CFG2_OFFSET,
        MC33771C_GPIO_CFG2_GPIO0_WU_MASK,
        MC33771C_GPIO_CFG2_GPIO0_WU(MC33771C_GPIO_CFG2_GPIO0_WU_WAKEUP_ENUM_VAL));
    }
  }
  else if ((mode == BCC_PIN_CONVERT_TR_IN) && (gpio_selection == 2U))
  {
    /* Set GPIO2 to digital input serving as a conversion trigger. */
    status = set_gpio_config(cid, 2U, BCC_PIN_DIGITAL_IN);
    if (status == BCC_STATUS_SUCCESS)
    {
      status = update_register(cid,
        MC33771C_GPIO_CFG2_OFFSET,
        MC33771C_GPIO_CFG2_GPIO2_SOC_MASK,
        MC33771C_GPIO_CFG2_GPIO2_SOC(MC33771C_GPIO_CFG2_GPIO2_SOC_ADC_TRG_ENABLED_ENUM_VAL));
    }
  }
  else if (mode <= BCC_PIN_DIGITAL_OUT)
  {
    status = BCC_STATUS_SUCCESS;
    if (gpio_selection == 0U)
    {
      /* Disable the wake-up capability. */
      status = update_register(cid,
        MC33771C_GPIO_CFG2_OFFSET,
        MC33771C_GPIO_CFG2_GPIO0_WU_MASK,
        MC33771C_GPIO_CFG2_GPIO0_WU(MC33771C_GPIO_CFG2_GPIO0_WU_NO_WAKEUP_ENUM_VAL));
    }
    else if (gpio_selection == 2U)
    {
      /* Disable the conversion trigger. */
      status = update_register(cid,
        MC33771C_GPIO_CFG2_OFFSET,
        MC33771C_GPIO_CFG2_GPIO2_SOC_MASK,
        MC33771C_GPIO_CFG2_GPIO2_SOC(MC33771C_GPIO_CFG2_GPIO2_SOC_ADC_TRG_DISABLED_ENUM_VAL));
    }

    if (status == BCC_STATUS_SUCCESS)
    {
      status = set_gpio_config(cid, gpio_selection, mode);
    }
  }

  return status;
}

bcc_status_t BatteryCellController::read_gpio(bcc_cid_t cid, uint8_t gpio_selection, bool *value) {
  bcc_status_t status;
  uint16_t gpio_sts_value;

  assert_param(value != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
    (gpio_selection >= BCC_GPIO_INPUT_CNT))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Read and update content of GPIO_CFG2 register. */
  status = read_register(cid, MC33771C_GPIO_STS_OFFSET, 1U, &gpio_sts_value);
  *value = (gpio_sts_value & (1U << gpio_selection)) > 0U;

  return status;
}

bcc_status_t BatteryCellController::write_gpio(bcc_cid_t cid, uint8_t gpio_selection, bool value) {
  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
    (gpio_selection >= BCC_GPIO_INPUT_CNT))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Update the content of GPIO_CFG2 register. */
  return update_register(cid, MC33771C_GPIO_CFG2_OFFSET,
    (uint16_t)(1U << gpio_selection),
    (uint16_t)((value ? 1U : 0U) << gpio_selection));
}

bcc_status_t BatteryCellController::enable_cell_balancing(bcc_cid_t cid, bool enable) {
  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  return update_register(cid, MC33771C_SYS_CFG1_OFFSET, MC33771C_SYS_CFG1_CB_DRVEN_MASK,
    enable ? MC33771C_SYS_CFG1_CB_DRVEN(MC33771C_SYS_CFG1_CB_DRVEN_ENABLED_ENUM_VAL) 
    : MC33771C_SYS_CFG1_CB_DRVEN(MC33771C_SYS_CFG1_CB_DRVEN_DISABLED_ENUM_VAL));
}

bcc_status_t BatteryCellController::set_cell_balancing(bcc_cid_t cid, uint8_t cell_index, bool enable, uint16_t timer) {
  uint16_t config_value;

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  if (cell_index >= BCC_MAX_CELLS_DEV(devices[(uint8_t)cid - 1]))
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  if (timer > MC33771C_CB1_CFG_CB_TIMER_MASK)
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  config_value = enable ? MC33771C_CB1_CFG_CB_EN(MC33771C_CB1_CFG_CB_EN_ENABLED_ENUM_VAL) 
                      : MC33771C_CB1_CFG_CB_EN(MC33771C_CB1_CFG_CB_EN_DISABLED_ENUM_VAL);
  config_value |= MC33771C_CB1_CFG_CB_TIMER(timer);

  return write_register(cid, MC33771C_CB1_CFG_OFFSET + cell_index, config_value);
}

bcc_status_t BatteryCellController::pause_cell_balancing(bcc_cid_t cid, bool pause) {
  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  return update_register(cid, MC33771C_SYS_CFG1_OFFSET, MC33771C_SYS_CFG1_CB_MANUAL_PAUSE_MASK,
    (pause) ? MC33771C_SYS_CFG1_CB_MANUAL_PAUSE(MC33771C_SYS_CFG1_CB_MANUAL_PAUSE_ENABLED_ENUM_VAL) 
            : MC33771C_SYS_CFG1_CB_MANUAL_PAUSE(MC33771C_SYS_CFG1_CB_MANUAL_PAUSE_DISABLED_ENUM_VAL));
}

bcc_status_t BatteryCellController::read_fuse_mirror(bcc_cid_t cid, uint8_t fuse_address, uint16_t* value) {
  bcc_status_t status;

  assert_param(value != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  if (fuse_address > ((devices[(uint8_t)cid - 1U] == BCC_DEVICE_MC33771C) ?
          MC33771C_MAX_FUSE_READ_ADDR : MC33772C_MAX_FUSE_READ_ADDR))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  status = write_register(cid, MC33771C_FUSE_MIRROR_CNTL_OFFSET,
    MC33771C_FUSE_MIRROR_CNTL_FMR_ADDR(fuse_address) |
    MC33771C_FUSE_MIRROR_CNTL_FSTM(MC33771C_FUSE_MIRROR_CNTL_FSTM_LOCKED_ENUM_VAL) |
    MC33771C_FUSE_MIRROR_CNTL_FST(MC33771C_FUSE_MIRROR_CNTL_FST_SPI_WRITE_ENABLE_ENUM_VAL));
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  return read_register(cid, MC33771C_FUSE_MIRROR_DATA_OFFSET, 1U, value);
}

bcc_status_t BatteryCellController::write_fuse_mirror(bcc_cid_t cid, uint8_t fuse_address, uint16_t value) {
  bcc_status_t status;

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  if (fuse_address > ((devices[(uint8_t)cid - 1U] == BCC_DEVICE_MC33771C) ?
          MC33771C_MAX_FUSE_WRITE_ADDR : MC33772C_MAX_FUSE_WRITE_ADDR))
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  /* FUSE_MIRROR_CNTL to enable writing. */
  status = write_register(cid, MC33771C_FUSE_MIRROR_CNTL_OFFSET,
    MC33771C_FUSE_MIRROR_CNTL_FMR_ADDR(0U) |
    MC33771C_FUSE_MIRROR_CNTL_FSTM(MC33771C_FUSE_MIRROR_CNTL_FSTM_UNLOCKED_ENUM_VAL) |
    MC33771C_FUSE_MIRROR_CNTL_FST(MC33771C_FUSE_MIRROR_CNTL_FST_SPI_WRITE_ENABLE_ENUM_VAL));
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Send the fuse address. */
  status = write_register(cid, MC33771C_FUSE_MIRROR_CNTL_OFFSET,
    MC33771C_FUSE_MIRROR_CNTL_FMR_ADDR(fuse_address) |
    MC33771C_FUSE_MIRROR_CNTL_FSTM(MC33771C_FUSE_MIRROR_CNTL_FSTM_UNLOCKED_ENUM_VAL) |
    MC33771C_FUSE_MIRROR_CNTL_FST(MC33771C_FUSE_MIRROR_CNTL_FST_SPI_WRITE_ENABLE_ENUM_VAL));
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  status = write_register(cid, MC33771C_FUSE_MIRROR_DATA_OFFSET, value);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* FUSE_MIRROR_CNTL to low power. */
  return write_register(cid, MC33771C_FUSE_MIRROR_CNTL_OFFSET,
    MC33771C_FUSE_MIRROR_CNTL_FMR_ADDR(0U) |
    MC33771C_FUSE_MIRROR_CNTL_FSTM(MC33771C_FUSE_MIRROR_CNTL_FSTM_UNLOCKED_ENUM_VAL) | 
    MC33771C_FUSE_MIRROR_CNTL_FST(MC33771C_FUSE_MIRROR_CNTL_FST_LP_ENUM_VAL));
}

bcc_status_t BatteryCellController::read_guid(bcc_cid_t cid, uint64_t *guid) {
  const uint8_t addr771c[3] = {
    MC33771C_FUSE_TR_0_OFFSET,
    MC33771C_FUSE_TR_1_OFFSET,
    MC33771C_FUSE_TR_2_OFFSET
  };
  const uint8_t addr772c[3] = {
    MC33772C_FUSE_TR_0_OFFSET,
    MC33772C_FUSE_TR_1_OFFSET,
    MC33772C_FUSE_TR_2_OFFSET
  };
  uint8_t const *readAddr;
  uint16_t readData[3];
  uint8_t i;
  bcc_status_t status;

  assert_param(guid != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  readAddr = (devices[(uint8_t)cid - 1] == BCC_DEVICE_MC33771C) ? addr771c : addr772c;

  for (i = 0; i < 3; i++)
  {
    status = read_fuse_mirror(cid, readAddr[i], &(readData[i]));
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }
  }

  *guid = (((uint64_t)(readData[0] & BCC_FUSE_TR_0_MASK)) << 21) |
    (((uint64_t)(readData[1] & BCC_FUSE_TR_1_MASK)) << 5) |
    ((uint64_t)(readData[2] & BCC_FUSE_TR_2_MASK));

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::read_eeprom(bcc_cid_t cid, uint8_t address, uint8_t *data) {
  bcc_status_t status;
  uint16_t reg_value;

  assert_param(data != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  if (address > BCC_MAX_EEPROM_ADDR)
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* EEPROM Read command. */
  reg_value = MC33771C_EEPROM_CTRL_R_W(MC33771C_EEPROM_CTRL_R_W_READ_ENUM_VAL) |
    MC33771C_EEPROM_CTRL_EEPROM_ADD(address);
  status = write_register(cid, MC33771C_EEPROM_CTRL_OFFSET, reg_value);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Wait while data is read from EEPROM. */
  status = start_timeout(BCC_EEPROM_READ_TIMEOUT_US);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  do
  {
    status = read_register(cid, MC33771C_EEPROM_CTRL_OFFSET, 1U, &reg_value);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }
  } while ((reg_value & MC33771C_EEPROM_CTRL_BUSY_MASK) && (!has_timer_expired()));

  /* Check once more after timeout expiration because the read command takes
    * several tens/hundreds of microseconds (depends on user code efficiency)
    * and the last read command could be done relatively long before the
    * timeout expiration. */
  if (reg_value & MC33771C_EEPROM_CTRL_BUSY_MASK)
  {
    status = read_register(cid, MC33771C_EEPROM_CTRL_OFFSET, 1U, &reg_value);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }
  }

  if (reg_value & MC33771C_EEPROM_CTRL_BUSY_MASK)
  {
    return BCC_STATUS_COM_TIMEOUT;
  }

  if (reg_value & MC33771C_EEPROM_CTRL_EE_PRESENT_MASK)
  {
    return BCC_STATUS_EEPROM_PRESENT;
  }

  if (reg_value & MC33771C_EEPROM_CTRL_ERROR_MASK)
  {
    return BCC_STATUS_EEPROM_ERROR;
  }

  /* Store read data to memory space defined by the pointer. */
  *data = (uint8_t)(reg_value & MC33771C_EEPROM_CTRL_READ_DATA_MASK);

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::write_eeprom(bcc_cid_t cid, uint8_t address, uint8_t data) {
  bcc_status_t status;
  uint16_t reg_value;

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  if (address > BCC_MAX_EEPROM_ADDR)
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* EEPROM Write command. */
  reg_value = MC33771C_EEPROM_CTRL_R_W(MC33771C_EEPROM_CTRL_R_W_WRITE_ENUM_VAL) |
    MC33771C_EEPROM_CTRL_EEPROM_ADD(address) |
    MC33771C_EEPROM_CTRL_DATA_TO_WRITE(data);
  status = write_register(cid, MC33771C_EEPROM_CTRL_OFFSET, reg_value);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Wait while BCC sends the write command to EEPROM. */
  status = start_timeout(BCC_EEPROM_WRITE_TIMEOUT_US);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  do
  {
    status = read_register(cid, MC33771C_EEPROM_CTRL_OFFSET, 1U, &reg_value);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }
  } while ((reg_value & MC33771C_EEPROM_CTRL_BUSY_MASK) && (!has_timer_expired()));

  /* Check once more after timeout expiration because the read command takes
    * several tens/hundreds of microseconds (depends on user code efficiency)
    * and the last read command could be done relatively long before the
    * timeout expiration. */
  if (reg_value & MC33771C_EEPROM_CTRL_BUSY_MASK)
  {
    status = read_register(cid, MC33771C_EEPROM_CTRL_OFFSET, 1U, &reg_value);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }
  }

  if (reg_value & MC33771C_EEPROM_CTRL_BUSY_MASK)
  {
    return BCC_STATUS_COM_TIMEOUT;
  }

  if (reg_value & MC33771C_EEPROM_CTRL_EE_PRESENT_MASK)
  {
    return BCC_STATUS_EEPROM_PRESENT;
  }

  if (reg_value & MC33771C_EEPROM_CTRL_ERROR_MASK)
  {
    return BCC_STATUS_EEPROM_ERROR;
  }

  return BCC_STATUS_SUCCESS;
}