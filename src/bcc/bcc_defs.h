#pragma once

/*! @brief Time after VPWR connection for the IC to be ready for initialization
 *  (t_VPWR(READY), max.) in [ms]. */
#define BCC_T_VPWR_READY_MS             5U

/*! @brief RESET de-glitch filter (t_RESETFLT, typ.) in [us]. */
#define BCC_T_RESETFLT_US               100U

/*! @brief CSB wake-up de-glitch filter, low to high transition
 * (CSB_WU_FLT, max.) in [us].
 * MC33771C: Max 80 us
 * MC33772C: Max 65 us */
#define BCC_CSB_WU_FLT_US               80U

/*! @brief Power up duration (t_WAKE-UP, max.) in [us]. */
#define BCC_T_WAKE_UP_US                440U

/*! @brief CSB_TX LOW period in CSB_TX wake-up pulse sequence in [us]. */
#define BCC_WAKE_PULSE_US               25U

/*! @brief Time between wake pulses (t_WAKE_DELAY, typ.) in [us]. */
#define BCC_T_WAKE_DELAY_US             600U

/*! @brief Time the MCU shall wait after sending first wake-up message
 * per 33771C/33772C IC (t_WU_Wait, min.) in [us]. */
#define BCC_T_WU_WAIT_US                750U

/*! @brief EN LOW to HIGH transition to INTB verification pulse
 * (t_INTB_PULSE_DELAY, max.) in [us]. */
#define BCC_T_INTB_PULSE_DELAY_US       100U

/*! @brief INTB verification pulse duration (t_INTB_PULSE, typ.) in [us]. */
#define BCC_T_INTB_PULSE_US             100U

/*! @brief SOC to data ready (includes post processing of data, ADC_CFG[AVG]=0)
 * (in [us]), typical value. */
#define BCC_T_EOC_TYP_US                520U

/*! @brief Timeout for SOC to data ready (ADC_CFG[AVG]=0) (in [us]).
 * Note: The typical value is 520 us, the maximal one 546 us. */
#define BCC_T_EOC_TIMEOUT_US            650U

/*! @brief Timeout (in [us]) for BCC device to perform a EEPROM read command
 * via I2C (START + 2*(8b + ACK) + START + 2*(8b + ACK) + STOP) and to clear
 * EEPROM_CTRL_BUSY flag.
 *
 * Note: The time measured for KIT33772CTPLEVB was around 420 us. */
#define BCC_EEPROM_READ_TIMEOUT_US      1000

/*! @brief Timeout (in [us]) for BCC device to perform a EEPROM write command
 * via I2C (START + 3*(8b + ACK) + STOP) and to clear EEPROM_CTRL_BUSY flag.
 *
 * Note: The time measured for KIT33772CTPLEVB was around 310 us. */
#define BCC_EEPROM_WRITE_TIMEOUT_US     800

/*! @brief Maximal address of EEPROM data. */
#define BCC_MAX_EEPROM_ADDR             0x7FU

/*! @brief Maximal MC33771C fuse mirror address for read access. */
#define MC33771C_MAX_FUSE_READ_ADDR     0x1AU

/*! @brief Maximal MC33771C fuse mirror address for read access. */
#define MC33772C_MAX_FUSE_READ_ADDR     0x12U

/*! @brief Maximal MC33771C fuse mirror address for read access. */
#define MC33771C_MAX_FUSE_WRITE_ADDR    0x17U

/*! @brief Maximal MC33771C fuse mirror address for read access. */
#define MC33772C_MAX_FUSE_WRITE_ADDR    0x0FU

/*! @brief Fuse address of Traceability 0 in MC33771C. */
#define MC33771C_FUSE_TR_0_OFFSET       0x18U

/*! @brief Fuse address of Traceability 1 in MC33771C. */
#define MC33771C_FUSE_TR_1_OFFSET       0x19U

/*! @brief Fuse address of Traceability 2 in MC33771C. */
#define MC33771C_FUSE_TR_2_OFFSET       0x1AU

/*! @brief Fuse address of Traceability 0 in MC33772C. */
#define MC33772C_FUSE_TR_0_OFFSET       0x10U

/*! @brief Fuse address of Traceability 1 in MC33772C. */
#define MC33772C_FUSE_TR_1_OFFSET       0x11U

/*! @brief Fuse address of Traceability 2 in MC33772C. */
#define MC33772C_FUSE_TR_2_OFFSET       0x12U

/*! @brief Mask of Traceability 0 data. */
#define BCC_FUSE_TR_0_MASK              0xFFFFU

/*! @brief Mask of Traceability 1 data. */
#define BCC_FUSE_TR_1_MASK              0xFFFFU

/*! @brief Mask of Traceability 2 data. */
#define BCC_FUSE_TR_2_MASK              0x001FU

/*! @brief Array containing cell maps for different numbers of cells connected
 * to the MC33771C. */
static const uint16_t s_cellMap33771c[MC33771C_MAX_CELLS + 1] = {
    0x0000,   /* Unsupported number of cells. */
    0x0000,   /* Unsupported number of cells. */
    0x0000,   /* Unsupported number of cells. */
    0x0000,   /* Unsupported number of cells. */
    0x0000,   /* Unsupported number of cells. */
    0x0000,   /* Unsupported number of cells. */
    0x0000,   /* Unsupported number of cells. */
    0x380F,   /* 7 cells. */
    0x3C0F,   /* 8 cells. */
    0x3E0F,   /* 9 cells. */
    0x3F0F,   /* 10 cells. */
    0x3F8F,   /* 11 cells. */
    0x3FCF,   /* 12 cells. */
    0x3FEF,   /* 13 cells. */
    0x3FFF    /* 14 cells. */
};

/*! @brief Array containing cell maps for different numbers of cells connected
 * to the MC33772C. */
static const uint16_t s_cellMap33772c[MC33772C_MAX_CELLS + 1] = {
    0x0000,   /* Unsupported number of cells. */
    0x0000,   /* Unsupported number of cells. */
    0x0000,   /* Unsupported number of cells. */
    0x0023,   /* 3 cells. */
    0x0033,   /* 4 cells. */
    0x003B,   /* 5 cells. */
    0x003F    /* 6 cells. */
};

#define MC33771C_INIT_CONF_REG_CNT     59U
#define MC33772C_INIT_CONF_REG_CNT     43U
#define SPI_ALIGNMENT 6

#define FAULT D6
#define INTB D7
#define EN D8

#define CSB_TX PA4
#define CSB_RX PA15

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Size of CRC table. */
#define BCC_CRC_TBL_SIZE          256U

/** BCC Commands. */
/*! @brief No operation command. */
#define BCC_CMD_NOOP              0x00U
/*! @brief Read command. */
#define BCC_CMD_READ              0x01U
/*! @brief Write command. */
#define BCC_CMD_WRITE             0x02U
/*! @brief Global write command. */
#define BCC_CMD_GLOB_WRITE        0x03U

/*!
 * @brief Returns data field of the communication frame.
 *
 * @param msg Pointer to the frame.
 * @return Data field.
 */
#define BCC_GET_MSG_DATA(msg) \
    (((uint16_t)*((msg) + BCC_MSG_IDX_DATA_H) << 8U) | \
      (uint16_t)*((msg) + BCC_MSG_IDX_DATA_L))

/*! @brief Mask for address field of frame. */
#define BCC_MSG_ADDR_MASK         0x7FU
/*! @brief Mask of the message counter bit field within the BCC_MSG_IDX_CNT_CMD
 *  byte. */
#define BCC_MSG_MSG_CNT_MASK      0xF0U
/*! @brief Shift of the message counter bit field within the BCC_MSG_IDX_CNT_CMD
 *  byte. */
#define BCC_MSG_MSG_CNT_SHIFT     4U

/*!
 * @brief Increments message counter value and executes modulo 16.
 *
 * @param msgCntr Message counter to be incremented.
 * @return Incremented value.
 */
#define BCC_INC_MSG_CNTR(msgCntr) \
    (((msgCntr) + 1U) & 0x0FU)

/*!
 * @brief Returns true if all bit fields except Message counter and CRC field
 * are zero.
 *
 * @param resp Response message to be checked.
 * @return True when the response is zero (except CRC and MSG_CNTR), false
 *         otherwise.
 */
#define BCC_IS_NULL_RESP(resp) \
    (((resp)[BCC_MSG_IDX_DATA_H] == 0U) && \
     ((resp)[BCC_MSG_IDX_DATA_L] == 0U) && \
     ((resp)[BCC_MSG_IDX_ADDR] == 0U) && \
     ((resp)[BCC_MSG_IDX_CID] == 0U) && \
     (((resp)[BCC_MSG_IDX_CNT_CMD] & (~BCC_MSG_MSG_CNT_MASK)) == 0U))

/*! @brief Address of the last register. */
#define BCC_MAX_REG_ADDR       0x7FU

/* Table with precalculated CRC values. */
static const uint8_t s_crcTable[BCC_CRC_TBL_SIZE] = {
    0x00U, 0x2fU, 0x5eU, 0x71U, 0xbcU, 0x93U, 0xe2U, 0xcdU,
    0x57U, 0x78U, 0x09U, 0x26U, 0xebU, 0xc4U, 0xb5U, 0x9aU,
    0xaeU, 0x81U, 0xf0U, 0xdfU, 0x12U, 0x3dU, 0x4cU, 0x63U,
    0xf9U, 0xd6U, 0xa7U, 0x88U, 0x45U, 0x6aU, 0x1bU, 0x34U,
    0x73U, 0x5cU, 0x2dU, 0x02U, 0xcfU, 0xe0U, 0x91U, 0xbeU,
    0x24U, 0x0bU, 0x7aU, 0x55U, 0x98U, 0xb7U, 0xc6U, 0xe9U,
    0xddU, 0xf2U, 0x83U, 0xacU, 0x61U, 0x4eU, 0x3fU, 0x10U,
    0x8aU, 0xa5U, 0xd4U, 0xfbU, 0x36U, 0x19U, 0x68U, 0x47U,
    0xe6U, 0xc9U, 0xb8U, 0x97U, 0x5aU, 0x75U, 0x04U, 0x2bU,
    0xb1U, 0x9eU, 0xefU, 0xc0U, 0x0dU, 0x22U, 0x53U, 0x7cU,
    0x48U, 0x67U, 0x16U, 0x39U, 0xf4U, 0xdbU, 0xaaU, 0x85U,
    0x1fU, 0x30U, 0x41U, 0x6eU, 0xa3U, 0x8cU, 0xfdU, 0xd2U,
    0x95U, 0xbaU, 0xcbU, 0xe4U, 0x29U, 0x06U, 0x77U, 0x58U,
    0xc2U, 0xedU, 0x9cU, 0xb3U, 0x7eU, 0x51U, 0x20U, 0x0fU,
    0x3bU, 0x14U, 0x65U, 0x4aU, 0x87U, 0xa8U, 0xd9U, 0xf6U,
    0x6cU, 0x43U, 0x32U, 0x1dU, 0xd0U, 0xffU, 0x8eU, 0xa1U,
    0xe3U, 0xccU, 0xbdU, 0x92U, 0x5fU, 0x70U, 0x01U, 0x2eU,
    0xb4U, 0x9bU, 0xeaU, 0xc5U, 0x08U, 0x27U, 0x56U, 0x79U,
    0x4dU, 0x62U, 0x13U, 0x3cU, 0xf1U, 0xdeU, 0xafU, 0x80U,
    0x1aU, 0x35U, 0x44U, 0x6bU, 0xa6U, 0x89U, 0xf8U, 0xd7U,
    0x90U, 0xbfU, 0xceU, 0xe1U, 0x2cU, 0x03U, 0x72U, 0x5dU,
    0xc7U, 0xe8U, 0x99U, 0xb6U, 0x7bU, 0x54U, 0x25U, 0x0aU,
    0x3eU, 0x11U, 0x60U, 0x4fU, 0x82U, 0xadU, 0xdcU, 0xf3U,
    0x69U, 0x46U, 0x37U, 0x18U, 0xd5U, 0xfaU, 0x8bU, 0xa4U,
    0x05U, 0x2aU, 0x5bU, 0x74U, 0xb9U, 0x96U, 0xe7U, 0xc8U,
    0x52U, 0x7dU, 0x0cU, 0x23U, 0xeeU, 0xc1U, 0xb0U, 0x9fU,
    0xabU, 0x84U, 0xf5U, 0xdaU, 0x17U, 0x38U, 0x49U, 0x66U,
    0xfcU, 0xd3U, 0xa2U, 0x8dU, 0x40U, 0x6fU, 0x1eU, 0x31U,
    0x76U, 0x59U, 0x28U, 0x07U, 0xcaU, 0xe5U, 0x94U, 0xbbU,
    0x21U, 0x0eU, 0x7fU, 0x50U, 0x9dU, 0xb2U, 0xc3U, 0xecU,
    0xd8U, 0xf7U, 0x86U, 0xa9U, 0x64U, 0x4bU, 0x3aU, 0x15U,
    0x8fU, 0xa0U, 0xd1U, 0xfeU, 0x33U, 0x1cU, 0x6dU, 0x42U
};

/* Timeout for sending one 48b frame via SPI TX in milliseconds. */
#define BCC_TX_COM_TIMEOUT_MS     1

/* Timeout for SPI TX communication in milliseconds. Note that the maximal
 * transfer (receiving of 127 registers) takes 4 ms. Another 0.95 us
 * (t_port_delay) is introduced by each repeater in 33771. */
#define BCC_RX_COM_TIMEOUT_MS     10

/* Timeout for sending one 48b frame via SPI in milliseconds. */
#define BCC_COM_TIMEOUT_MS        1