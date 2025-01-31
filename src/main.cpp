#include <Arduino.h>
// #include "bcc/bcc.h"
// #include "bcc/bcc_communication.h"
// #include "bcc/bcc_utils.h"
// #include "bcc/bcc_wait.h"
// #include "common.h"
// #include "monitoring.h"

// ntc_config_t ntcConfig;
// bcc_data_t g_bccData;
// bcc_status_t bccError;

// #define MC33771C_INIT_CONF_REG_CNT     59U
// #define MC33772C_INIT_CONF_REG_CNT     43U
// #define LPSPI_ALIGNMENT           8

// typedef struct
// {
//   const uint8_t address;
//   const uint16_t defaultVal;
//   const uint16_t value;
// } bcc_init_reg_t;

// static bcc_status_t initRegisters();
// static bcc_status_t clearFaultRegs();
// void onTimerCallback();

// static const bcc_init_reg_t s_initRegsMc33771c[MC33771C_INIT_CONF_REG_CNT] = {
//     {MC33771C_GPIO_CFG1_OFFSET, MC33771C_GPIO_CFG1_POR_VAL, MC33771C_GPIO_CFG1_VALUE},
//     {MC33771C_GPIO_CFG2_OFFSET, MC33771C_GPIO_CFG2_POR_VAL, MC33771C_GPIO_CFG2_VALUE},
//     {MC33771C_TH_ALL_CT_OFFSET, MC33771C_TH_ALL_CT_POR_VAL, MC33771C_TH_ALL_CT_VALUE},
//     {MC33771C_TH_CT14_OFFSET, MC33771C_TH_CT14_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_CT13_OFFSET, MC33771C_TH_CT13_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_CT12_OFFSET, MC33771C_TH_CT12_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_CT11_OFFSET, MC33771C_TH_CT11_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_CT10_OFFSET, MC33771C_TH_CT10_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_CT9_OFFSET, MC33771C_TH_CT9_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_CT8_OFFSET, MC33771C_TH_CT8_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_CT7_OFFSET, MC33771C_TH_CT7_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_CT6_OFFSET, MC33771C_TH_CT6_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_CT5_OFFSET, MC33771C_TH_CT5_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_CT4_OFFSET, MC33771C_TH_CT4_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_CT3_OFFSET, MC33771C_TH_CT3_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_CT2_OFFSET, MC33771C_TH_CT2_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_CT1_OFFSET, MC33771C_TH_CT1_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33771C_TH_AN6_OT_OFFSET, MC33771C_TH_AN6_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33771C_TH_AN5_OT_OFFSET, MC33771C_TH_AN5_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33771C_TH_AN4_OT_OFFSET, MC33771C_TH_AN4_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33771C_TH_AN3_OT_OFFSET, MC33771C_TH_AN3_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33771C_TH_AN2_OT_OFFSET, MC33771C_TH_AN2_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33771C_TH_AN1_OT_OFFSET, MC33771C_TH_AN1_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33771C_TH_AN0_OT_OFFSET, MC33771C_TH_AN0_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33771C_TH_AN6_UT_OFFSET, MC33771C_TH_AN6_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33771C_TH_AN5_UT_OFFSET, MC33771C_TH_AN5_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33771C_TH_AN4_UT_OFFSET, MC33771C_TH_AN4_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33771C_TH_AN3_UT_OFFSET, MC33771C_TH_AN3_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33771C_TH_AN2_UT_OFFSET, MC33771C_TH_AN2_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33771C_TH_AN1_UT_OFFSET, MC33771C_TH_AN1_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33771C_TH_AN0_UT_OFFSET, MC33771C_TH_AN0_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33771C_TH_ISENSE_OC_OFFSET, MC33771C_TH_ISENSE_OC_POR_VAL, MC33771C_TH_ISENSE_OC_VALUE},
//     {MC33771C_TH_COULOMB_CNT_MSB_OFFSET, MC33771C_TH_COULOMB_CNT_MSB_POR_VAL, MC33771C_TH_COULOMB_CNT_MSB_VALUE},
//     {MC33771C_TH_COULOMB_CNT_LSB_OFFSET, MC33771C_TH_COULOMB_CNT_LSB_POR_VAL, MC33771C_TH_COULOMB_CNT_LSB_VALUE},
//     {MC33771C_CB1_CFG_OFFSET, MC33771C_CB1_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_CB2_CFG_OFFSET, MC33771C_CB2_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_CB3_CFG_OFFSET, MC33771C_CB3_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_CB4_CFG_OFFSET, MC33771C_CB4_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_CB5_CFG_OFFSET, MC33771C_CB5_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_CB6_CFG_OFFSET, MC33771C_CB6_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_CB7_CFG_OFFSET, MC33771C_CB7_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_CB8_CFG_OFFSET, MC33771C_CB8_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_CB9_CFG_OFFSET, MC33771C_CB9_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_CB10_CFG_OFFSET, MC33771C_CB10_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_CB11_CFG_OFFSET, MC33771C_CB11_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_CB12_CFG_OFFSET, MC33771C_CB12_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_CB13_CFG_OFFSET, MC33771C_CB13_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_CB14_CFG_OFFSET, MC33771C_CB14_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33771C_OV_UV_EN_OFFSET, MC33771C_OV_UV_EN_POR_VAL, MC33771C_OV_UV_EN_VALUE},
//     {MC33771C_SYS_CFG1_OFFSET, MC33771C_SYS_CFG1_POR_VAL, MC33771C_SYS_CFG1_VALUE},
//     {MC33771C_SYS_CFG2_OFFSET, MC33771C_SYS_CFG2_POR_VAL, MC33771C_SYS_CFG2_VALUE},
//     {MC33771C_ADC_CFG_OFFSET, MC33771C_ADC_CFG_POR_VAL, MC33771C_ADC_CFG_VALUE},
//     {MC33771C_ADC2_OFFSET_COMP_OFFSET, MC33771C_ADC2_OFFSET_COMP_POR_VAL, MC33771C_ADC2_OFFSET_COMP_VALUE},
//     {MC33771C_FAULT_MASK1_OFFSET, MC33771C_FAULT_MASK1_POR_VAL, MC33771C_FAULT_MASK1_VALUE},
//     {MC33771C_FAULT_MASK2_OFFSET, MC33771C_FAULT_MASK2_POR_VAL, MC33771C_FAULT_MASK2_VALUE},
//     {MC33771C_FAULT_MASK3_OFFSET, MC33771C_FAULT_MASK3_POR_VAL, MC33771C_FAULT_MASK3_VALUE},
//     {MC33771C_WAKEUP_MASK1_OFFSET, MC33771C_WAKEUP_MASK1_POR_VAL, MC33771C_WAKEUP_MASK1_VALUE},
//     {MC33771C_WAKEUP_MASK2_OFFSET, MC33771C_WAKEUP_MASK2_POR_VAL, MC33771C_WAKEUP_MASK2_VALUE},
//     {MC33771C_WAKEUP_MASK3_OFFSET, MC33771C_WAKEUP_MASK3_POR_VAL, MC33771C_WAKEUP_MASK3_VALUE},
// };

// static const bcc_init_reg_t s_initRegsMc33772c[MC33772C_INIT_CONF_REG_CNT] = {
//     {MC33772C_GPIO_CFG1_OFFSET, MC33772C_GPIO_CFG1_POR_VAL, MC33772C_GPIO_CFG1_VALUE},
//     {MC33772C_GPIO_CFG2_OFFSET, MC33772C_GPIO_CFG2_POR_VAL, MC33771C_GPIO_CFG2_VALUE},
//     {MC33772C_TH_ALL_CT_OFFSET, MC33772C_TH_ALL_CT_POR_VAL, MC33771C_TH_ALL_CT_VALUE},
//     {MC33772C_TH_CT6_OFFSET, MC33772C_TH_CT6_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33772C_TH_CT5_OFFSET, MC33772C_TH_CT5_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33772C_TH_CT4_OFFSET, MC33772C_TH_CT4_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33772C_TH_CT3_OFFSET, MC33772C_TH_CT3_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33772C_TH_CT2_OFFSET, MC33772C_TH_CT2_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33772C_TH_CT1_OFFSET, MC33772C_TH_CT1_POR_VAL, MC33771C_TH_CTX_VALUE},
//     {MC33772C_TH_AN6_OT_OFFSET, MC33772C_TH_AN6_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33772C_TH_AN5_OT_OFFSET, MC33772C_TH_AN5_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33772C_TH_AN4_OT_OFFSET, MC33772C_TH_AN4_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33772C_TH_AN3_OT_OFFSET, MC33772C_TH_AN3_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33772C_TH_AN2_OT_OFFSET, MC33772C_TH_AN2_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33772C_TH_AN1_OT_OFFSET, MC33772C_TH_AN1_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33772C_TH_AN0_OT_OFFSET, MC33772C_TH_AN0_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
//     {MC33772C_TH_AN6_UT_OFFSET, MC33772C_TH_AN6_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33772C_TH_AN5_UT_OFFSET, MC33772C_TH_AN5_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33772C_TH_AN4_UT_OFFSET, MC33772C_TH_AN4_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33772C_TH_AN3_UT_OFFSET, MC33772C_TH_AN3_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33772C_TH_AN2_UT_OFFSET, MC33772C_TH_AN2_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33772C_TH_AN1_UT_OFFSET, MC33772C_TH_AN1_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33772C_TH_AN0_UT_OFFSET, MC33772C_TH_AN0_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
//     {MC33772C_TH_ISENSE_OC_OFFSET, MC33772C_TH_ISENSE_OC_POR_VAL, MC33771C_TH_ISENSE_OC_VALUE},
//     {MC33772C_TH_COULOMB_CNT_MSB_OFFSET, MC33772C_TH_COULOMB_CNT_MSB_POR_VAL, MC33771C_TH_COULOMB_CNT_MSB_VALUE},
//     {MC33772C_TH_COULOMB_CNT_LSB_OFFSET, MC33772C_TH_COULOMB_CNT_LSB_POR_VAL, MC33771C_TH_COULOMB_CNT_LSB_VALUE},
//     {MC33772C_CB1_CFG_OFFSET, MC33772C_CB1_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33772C_CB2_CFG_OFFSET, MC33772C_CB2_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33772C_CB3_CFG_OFFSET, MC33772C_CB3_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33772C_CB4_CFG_OFFSET, MC33772C_CB4_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33772C_CB5_CFG_OFFSET, MC33772C_CB5_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33772C_CB6_CFG_OFFSET, MC33772C_CB6_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
//     {MC33772C_OV_UV_EN_OFFSET, MC33772C_OV_UV_EN_POR_VAL, MC33772C_OV_UV_EN_VALUE},
//     {MC33772C_SYS_CFG1_OFFSET, MC33772C_SYS_CFG1_POR_VAL, MC33772C_SYS_CFG1_VALUE},
//     {MC33772C_SYS_CFG2_OFFSET, MC33772C_SYS_CFG2_POR_VAL, MC33771C_SYS_CFG2_VALUE},
//     {MC33772C_ADC_CFG_OFFSET, MC33772C_ADC_CFG_POR_VAL, MC33771C_ADC_CFG_VALUE},
//     {MC33772C_ADC2_OFFSET_COMP_OFFSET, MC33772C_ADC2_OFFSET_COMP_POR_VAL, MC33771C_ADC2_OFFSET_COMP_VALUE},
//     {MC33772C_FAULT_MASK1_OFFSET, MC33772C_FAULT_MASK1_POR_VAL, MC33771C_FAULT_MASK1_VALUE},
//     {MC33772C_FAULT_MASK2_OFFSET, MC33772C_FAULT_MASK2_POR_VAL, MC33771C_FAULT_MASK2_VALUE},
//     {MC33772C_FAULT_MASK3_OFFSET, MC33772C_FAULT_MASK3_POR_VAL, MC33772C_FAULT_MASK3_VALUE},
//     {MC33772C_WAKEUP_MASK1_OFFSET, MC33772C_WAKEUP_MASK1_POR_VAL, MC33771C_WAKEUP_MASK1_VALUE},
//     {MC33772C_WAKEUP_MASK2_OFFSET, MC33772C_WAKEUP_MASK2_POR_VAL, MC33771C_WAKEUP_MASK2_VALUE},
//     {MC33772C_WAKEUP_MASK3_OFFSET, MC33772C_WAKEUP_MASK3_POR_VAL, MC33772C_WAKEUP_MASK3_VALUE},
// };

#include "bcc/BatteryCellController.h"
#include "bcc/HalfDuplexSPI.h"

#define FAULT D6
#define INTB D7
#define EN D8

#define CELL_COUNT 6
#define DEVICE_COUNT 1

#define TX_DATA PA7
#define TX_SCLK PA5
#define TX_CS PA4

#define RX_DATA PB15
#define RX_SCLK PB13
#define RX_CS PB12

HalfDuplexSPI bcc_tx(true, TX_DATA, TX_SCLK, NC);
// HalfDuplexSPI bcc_rx(false, PB5, PB3, PA15);
HalfDuplexSPI bcc_rx(false, RX_DATA, RX_SCLK, NC);
bcc_device_t devices[DEVICE_COUNT] = {BCC_DEVICE_MC33772C};
BatteryCellController bcc(&bcc_tx, &bcc_rx, devices, DEVICE_COUNT, CELL_COUNT, EN, INTB, TX_CS, true);

SPISettings settings(4000000, MSBFIRST, SPI_MODE1);

void errorLoop() {
  bool enabled = false;
  while (1) {
    delay(100);
    digitalWrite(LED_BUILTIN, !enabled);
    enabled = !enabled;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(USER_BTN, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(TX_CS, OUTPUT);
  pinMode(RX_CS, INPUT_PULLUP);

  bcc_tx.begin();
  bcc_rx.begin();
  digitalWrite(TX_CS, HIGH);

  if (!bcc_tx.beginTransaction(settings)) {
    Serial.println("Error initializing BCC TX");
    errorLoop();
  }

  if (!bcc_rx.beginTransaction(settings)) {
    Serial.println("Error initializing BCC RX");  
    errorLoop();
  }

  // send dummy data on each SPI
  uint8_t dummy[2] = {0xFF, 0xFF};
  bcc_tx.transmit(dummy, 2, 1000);

startup:

  while (digitalRead(USER_BTN) == HIGH) {
    delay(100);
  }

  Serial.println("Starting battery cell controller...");
  auto error = bcc.begin();
  if (error != BCC_STATUS_SUCCESS) {
    Serial.print("Begin error: ");
    Serial.println(error);
    
    goto startup;
  }
}

void loop() {
  digitalWrite(TX_CS, LOW);
  delay(100);
  digitalWrite(TX_CS, HIGH);
}



// static bcc_status_t initRegisters()
// {
//     uint8_t cid, i;
//     bcc_status_t status;

//     for (cid = 1; cid <= g_bccData.drvConfig.devicesCnt; cid++)
//     {
//         if (g_bccData.drvConfig.device[cid - 1] == BCC_DEVICE_MC33771C)
//         {
//             for (i = 0; i < MC33771C_INIT_CONF_REG_CNT; i++)
//             {
//                 if (s_initRegsMc33771c[i].value != s_initRegsMc33771c[i].defaultVal)
//                 {
//                     status = BCC_Reg_Write(&g_bccData.drvConfig, (bcc_cid_t)cid,
//                             s_initRegsMc33771c[i].address, s_initRegsMc33771c[i].value);
//                     if (status != BCC_STATUS_SUCCESS)
//                     {
//                         return status;
//                     }
//                 }
//             }
//         }
//         else
//         {
//             for (i = 0; i < MC33772C_INIT_CONF_REG_CNT; i++)
//             {
//                 if (s_initRegsMc33772c[i].value != s_initRegsMc33772c[i].defaultVal)
//                 {
//                     status = BCC_Reg_Write(&g_bccData.drvConfig, (bcc_cid_t)cid,
//                             s_initRegsMc33772c[i].address, s_initRegsMc33772c[i].value);
//                     if (status != BCC_STATUS_SUCCESS)
//                     {
//                         return status;
//                     }
//                 }
//             }
//         }
//     }

//     return BCC_STATUS_SUCCESS;
// }

// /*!
//  * @brief Clears all fault registers of BCC devices.
//  */
// static bcc_status_t clearFaultRegs()
// {
//     uint8_t cid;
//     bcc_status_t status;

//     for (cid = 1; cid <= g_bccData.drvConfig.devicesCnt; cid++)
//     {
//         status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_CELL_OV);
//         if (status != BCC_STATUS_SUCCESS)
//         {
//             return status;
//         }

//         status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_CELL_UV);
//         if (status != BCC_STATUS_SUCCESS)
//         {
//             return status;
//         }

//         status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_CB_OPEN);
//         if (status != BCC_STATUS_SUCCESS)
//         {
//             return status;
//         }

//         status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_CB_SHORT);
//         if (status != BCC_STATUS_SUCCESS)
//         {
//             return status;
//         }

//         status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_GPIO_STATUS);
//         if (status != BCC_STATUS_SUCCESS)
//         {
//             return status;
//         }

//         status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_AN_OT_UT);
//         if (status != BCC_STATUS_SUCCESS)
//         {
//             return status;
//         }

//         status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_GPIO_SHORT);
//         if (status != BCC_STATUS_SUCCESS)
//         {
//             return status;
//         }

//         status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_COMM);
//         if (status != BCC_STATUS_SUCCESS)
//         {
//             return status;
//         }

//         status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_FAULT1);
//         if (status != BCC_STATUS_SUCCESS)
//         {
//             return status;
//         }

//         status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_FAULT2);
//         if (status != BCC_STATUS_SUCCESS)
//         {
//             return status;
//         }

//         status = BCC_Fault_ClearStatus(&g_bccData.drvConfig, (bcc_cid_t)cid, BCC_FS_FAULT3);
//         if (status != BCC_STATUS_SUCCESS)
//         {
//             return status;
//         }
//     }

//     return BCC_STATUS_SUCCESS;
// }
