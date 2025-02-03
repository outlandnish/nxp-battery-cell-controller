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
  bcc_status_t write_register_global(uint8_t reg_addr, uint16_t reg_val);
  bcc_status_t update_register(bcc_cid_t cid, uint8_t reg_addr, uint16_t mask, uint16_t val);

  bcc_status_t read_register_spi(bcc_cid_t cid, uint8_t reg_addr, uint8_t reg_cnt, uint16_t *reg_val);
  bcc_status_t write_register_spi(bcc_cid_t cid, uint8_t reg_addr, uint16_t reg_val);

  bcc_status_t read_register_tpl(bcc_cid_t cid, uint8_t reg_addr, uint8_t reg_cnt, uint16_t *reg_val);
  bcc_status_t write_register_tpl(bcc_cid_t cid, uint8_t reg_addr, uint16_t reg_val);
  bcc_status_t write_global_tpl(uint8_t reg_addr, uint16_t reg_val);

  bcc_status_t send_nop_spi(bcc_cid_t cid);
  bcc_status_t send_nop_tpl(bcc_cid_t cid);

  bcc_status_t start_timeout(uint64_t microseconds);
  bool has_timer_expired();

  std::queue<uint8_t> interrupt_events;
  void on_interrupt();
  void reset_interrupt_events();

  public:
    BatteryCellController(SPIClass *spi, bcc_device_t device, uint8_t cell_count, uint8_t enable_pin, uint8_t intb_pin, uint8_t cs_pin, bool loopback = false);
    BatteryCellController(HalfDuplexSPI *bcc_tx, HalfDuplexSPI *bcc_rx, bcc_device_t devices[], uint8_t device_count, uint8_t cell_count, uint8_t enable_pin, uint8_t intb_pin, uint8_t cs_tx_pin, bool loopback = false);

    bcc_status_t begin();
    void wake_up();

    bcc_status_t enable_tpl();
    void disable_tpl();

    bcc_status_t init_devices();
    bcc_status_t assign_cid(bcc_cid_t cid);
    bcc_status_t send_nop(bcc_cid_t cid);
    bcc_status_t sleep();
    bcc_status_t software_reset(bcc_cid_t cid);
    // bcc_status_t hardware_reset();

    // measurements
    bcc_status_t start_conversion_async(bcc_cid_t cid, bcc_avg_t average);
    bcc_status_t start_conversion_global_async(uint16_t adc_config_value);
    bcc_status_t check_conversion_completed(bcc_cid_t cid, bool *completed);
    bcc_status_t start_conversion(bcc_cid_t cid, bcc_avg_t average);
    bcc_status_t get_raw_values(bcc_cid_t cid, uint16_t *measurements);

    // system measurements
    bcc_status_t get_coulomb_counter(bcc_cid_t cid, bcc_cc_data_t* coulomb_counter);
    bcc_status_t get_current_sense_voltage(bcc_cid_t cid, int32_t *current_sense_voltage);
    bcc_status_t get_stack_voltage(bcc_cid_t cid, uint32_t *stack_voltage);

    // cell measurements
    bcc_status_t get_cell_voltages(bcc_cid_t cid, uint32_t *cell_voltages);
    bcc_status_t get_cell_voltage(bcc_cid_t cid, uint8_t cell_index, uint32_t *cell_voltage);

    // ANx measurements
    bcc_status_t get_an_voltages(bcc_cid_t cid, uint32_t *an_voltages);
    bcc_status_t get_an_voltage(bcc_cid_t cid, uint8_t an_index, uint32_t *an_voltage);

    // IC temperature
    bcc_status_t get_ic_temperature(bcc_cid_t cid, bcc_temp_unit_t unit, int16_t *ic_temperature);

    // fault status
    bcc_status_t get_fault_status(bcc_cid_t cid, uint16_t *fault_status);
    bcc_status_t clear_fault_status(bcc_cid_t cid, bcc_fault_status_t fault_status);

    // GPIO
    bcc_status_t set_gpio_config(bcc_cid_t cid, uint8_t gpio_sel, bool val);
    bcc_status_t set_gpio_mode(bcc_cid_t cid, uint8_t gpio_sel, bcc_pin_mode_t mode);
    bcc_status_t read_gpio(bcc_cid_t cid, uint8_t gpio_sel, bool *val);
    bcc_status_t write_gpio(bcc_cid_t cid, uint8_t gpio_sel, bool val);

    // cell balancing
    bcc_status_t enable_cell_balancing(bcc_cid_t cid, bool enable);
    bcc_status_t set_cell_balancing(bcc_cid_t cid, uint8_t cell_index, bool enable, uint16_t timer);
    bcc_status_t pause_cell_balancing(bcc_cid_t cid, bool pause);

    // fuse mirror
    bcc_status_t read_fuse_mirror(bcc_cid_t cid, uint8_t fuse_addr, uint16_t *value);
    bcc_status_t write_fuse_mirror(bcc_cid_t cid, uint8_t fuse_addr, uint16_t value);

    bcc_status_t read_guid(bcc_cid_t cid, uint64_t *guid);

    // EEPROM
    bcc_status_t read_eeprom(bcc_cid_t cid, uint8_t address, uint8_t *data);
    bcc_status_t write_eeprom(bcc_cid_t cid, uint8_t address, uint8_t data);
};