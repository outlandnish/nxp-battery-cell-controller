#include "HalfDuplexSPI.h"
#include "models.h"
#include <queue>

class BatteryCellController {
  SPIClass *spi;
  HalfDuplexSPI *bcc_tx;
  HalfDuplexSPI *bcc_rx;

  bcc_mode_t comm_mode;
  bcc_device_t devices[BCC_DEVICE_CNT_MAX];
  uint16_t cells[BCC_DEVICE_CNT_MAX];
  uint8_t device_count, cell_count;
  bool loopback;

  uint16_t cell_map[BCC_DEVICE_CNT_MAX];
  uint8_t message_counter[BCC_DEVICE_CNT_MAX + 1];
  uint8_t rx_buffer[BCC_RX_BUF_SIZE_TPL];

  uint8_t enable_pin, reset_pin, intb_pin, cs_tx_pin;
  uint8_t int_state;
  

  void pack_frame(uint16_t data, uint8_t addr, bcc_cid_t cid, uint8_t cmd_count, uint8_t *tx_buf);
  uint8_t calculate_crc(uint8_t *data);
  bcc_status_t check_crc(uint8_t *data);
  bcc_status_t check_message_counter(bcc_cid_t cid, uint8_t *data);
  bcc_status_t check_echo_frame(uint8_t tx[], uint8_t rx[]);

  bcc_status_t transfer_spi(uint8_t tx[], uint8_t rx[]);
  bcc_status_t transfer_tpl(uint8_t tx[], uint8_t rx_buf[], uint32_t rx_transfer_count);

  void wake_up_pattern_tpl();
  void wake_up_pattern_spi();

  bcc_status_t read_register(bcc_cid_t cid, uint8_t reg_addr, uint8_t reg_cnt, uint16_t *reg_val);
  bcc_status_t write_register(bcc_cid_t cid, uint8_t reg_addr, uint16_t reg_val);
  bcc_status_t write_global(uint8_t reg_addr, uint16_t reg_val);

  bcc_status_t read_register_spi(bcc_cid_t cid, uint8_t reg_addr, uint8_t reg_cnt, uint16_t *reg_val);
  bcc_status_t write_register_spi(bcc_cid_t cid, uint8_t reg_addr, uint16_t reg_val);

  bcc_status_t read_register_tpl(bcc_cid_t cid, uint8_t reg_addr, uint8_t reg_cnt, uint16_t *reg_val);
  bcc_status_t write_register_tpl(bcc_cid_t cid, uint8_t reg_addr, uint16_t reg_val);
  bcc_status_t write_global_tpl(uint8_t reg_addr, uint16_t reg_val);

  std::queue<uint8_t> interrupt_events;
  void on_interrupt();
  void reset_interrupt_events();

  public:
    BatteryCellController(SPIClass *spi, bcc_device_t device, uint8_t cell_count, uint8_t enable_pin, uint8_t intb_pin, uint8_t cs_pin, bool loopback = false);
    BatteryCellController(HalfDuplexSPI *bcc_tx, HalfDuplexSPI *bcc_rx, bcc_device_t devices[], uint8_t device_count, uint8_t cell_count, uint8_t enable_pin, uint8_t intb_pin, uint8_t cs_tx_pin, bool loopback = false);

    bcc_status_t begin();

    bcc_status_t enable_tpl();
    void disable_tpl();
    bcc_status_t init_devices();
    
    bcc_status_t assign_cid(bcc_cid_t cid);
    bcc_status_t send_nop(bcc_cid_t cid);
    bcc_status_t sleep();
    void wake_up();
    bcc_status_t software_reset(bcc_cid_t cid);
    bcc_status_t hardware_reset();
};