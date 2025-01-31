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
    return write_global(MC33771C_INIT_OFFSET, MC33771C_SYS_CFG1_SOFT_RST(MC33771C_SYS_CFG1_SOFT_RST_ACTIVE_ENUM_VAL));
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

bcc_status_t BatteryCellController::write_global(uint8_t reg_addr, uint16_t reg_val) {
  bcc_status_t status;

  assert_param(comm_mode == BCC_MODE_TPL);

  return write_global_tpl(reg_addr, reg_val);
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