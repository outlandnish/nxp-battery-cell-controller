// /*
//  * Copyright 2016 - 2020 NXP
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without modification,
//  * are permitted provided that the following conditions are met:
//  *
//  * o Redistributions of source code must retain the above copyright notice, this list
//  *   of conditions and the following disclaimer.
//  *
//  * o Redistributions in binary form must reproduce the above copyright notice, this
//  *   list of conditions and the following disclaimer in the documentation and/or
//  *   other materials provided with the distribution.
//  *
//  * o Neither the name of the copyright holder nor the names of its
//  *   contributors may be used to endorse or promote products derived from this
//  *   software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
//  * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//  * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  */

// #include "common.h"
// #include "bcc/MC33771C.h"

// /*******************************************************************************
//  * Definitions
//  ******************************************************************************/

// /* Value of CELL_OV_FLT register when no event occurred. */
// #define BCC_CELL_OV_FLT_NOEVENT   0x0000U

// /* Value of CELL_UV_FLT register when no event occurred. */
// #define BCC_CELL_UV_FLT_NOEVENT   0x0000U

// /* Value of CB_OPEN_FLT register when no event occurred. */
// #define BCC_CB_OPEN_FLT_NOEVENT   0x0000U

// /* Value of CB_SHORT_FLT register when no event occurred. */
// #define BCC_CB_SHORT_FLT_NOEVENT  0x0000U

// /* Value of AN_OT_UT_FLT register when no event occurred. */
// #define BCC_AN_OT_UT_FLT_NOEVENT  0x0000U

// /* Value of GPIO_SHORT register when no event occurred. */
// #define BCC_GPIO_SHORT_NOEVENT    0x0000U

// /* Value of FAULT1_STATUS register when no event occurred. */
// #define BCC_FAULT1_STATUS_NOEVENT 0x0000U

// /* Value of FAULT2_STATUS register when no event occurred. */
// #define BCC_FAULT2_STATUS_NOEVENT 0x0000U

// /* Value of FAULT3_STATUS register when no event occurred. */
// #define BCC_FAULT3_STATUS_NOEVENT 0x0000U

// /* Address of last printed fuse mirror register. */
// #define MC33771C_LAST_FUSE_ADDR   0x1AU
// #define MC33772C_LAST_FUSE_ADDR   0x12U

// /**
//  * Prints formated string with register name, value and whether an event
//  * occurred. It is intended for the status registers.
//  *
//  * @param regName Name of a status register.
//  * @param regVal Value of a status register.
//  * @param event "yes" when an event is signalized by register value.
//  */
// #define PRINT_STATUS_REG(regName, regVal, event) \
//     (Serial.printf("  | %s\t| 0x%02x %02x\t| %s\t\t     |\r\n", \
//             regName, regVal >> 8, regVal & 0x00FF, event))

// /**
//  * Prints formated string with register name, value and whether an event
//  * occurred. It is intended for the status registers.
//  *
//  * @param regName Name of a status register.
//  * @param regVal Value of a status register.
//  * @param defVal Value of a status register when has not occurred any event.
//  */
// #define PRINT_STATUS_REG_COMP(regName, regVal, defVal) \
//     (PRINT_STATUS_REG(regName, regVal, (regVal != defVal) ? "yes" : "no"))

// /* Number of configurable registers. */
// #define MC33771C_REG_CONF_CNT        61U
// #define MC33772C_REG_CONF_CNT        45U

// /*******************************************************************************
//  * Structure definition
//  ******************************************************************************/

// /* Structure containing a register name and its address. */
// typedef struct
// {
//     const char* name;
//     const uint8_t address;
// } bcc_drv_register_t;

// /*******************************************************************************
//  * Global variables (constants)
//  ******************************************************************************/

// /* Names and addresses of all configurable registers. */
// static const bcc_drv_register_t s_confRegsMc33771c[MC33771C_REG_CONF_CNT] = {
//     /* Note: INIT register is initialized automatically. */
//     /* Note: SYS_CFG_GLOBAL reg. contains only command GO2SLEEP
//      * (no initialization needed). */
//     { "SYS_CFG1", MC33771C_SYS_CFG1_OFFSET },
//     { "SYS_CFG2", MC33771C_SYS_CFG2_OFFSET },
//     { "SYS_DIAG", MC33771C_SYS_DIAG_OFFSET },
//     { "ADC_CFG", MC33771C_ADC_CFG_OFFSET },
//     { "ADC2_OFFSET_COMP", MC33771C_ADC2_OFFSET_COMP_OFFSET },
//     { "OV_UV_EN", MC33771C_OV_UV_EN_OFFSET },
//     { "TPL_CFG", MC33771C_TPL_CFG_OFFSET },
//     { "CB1_CFG", MC33771C_CB1_CFG_OFFSET },
//     { "CB2_CFG", MC33771C_CB2_CFG_OFFSET },
//     { "CB3_CFG", MC33771C_CB3_CFG_OFFSET },
//     { "CB4_CFG", MC33771C_CB4_CFG_OFFSET },
//     { "CB5_CFG", MC33771C_CB5_CFG_OFFSET },
//     { "CB6_CFG", MC33771C_CB6_CFG_OFFSET },
//     { "CB7_CFG", MC33771C_CB7_CFG_OFFSET },
//     { "CB8_CFG", MC33771C_CB8_CFG_OFFSET },
//     { "CB9_CFG", MC33771C_CB9_CFG_OFFSET },
//     { "CB10_CFG", MC33771C_CB10_CFG_OFFSET },
//     { "CB11_CFG", MC33771C_CB11_CFG_OFFSET },
//     { "CB12_CFG", MC33771C_CB12_CFG_OFFSET },
//     { "CB13_CFG", MC33771C_CB13_CFG_OFFSET },
//     { "CB14_CFG", MC33771C_CB14_CFG_OFFSET },
//     { "GPIO_CFG1", MC33771C_GPIO_CFG1_OFFSET },
//     { "GPIO_CFG2", MC33771C_GPIO_CFG2_OFFSET },
//     { "FAULT_MASK1", MC33771C_FAULT_MASK1_OFFSET },
//     { "FAULT_MASK2", MC33771C_FAULT_MASK2_OFFSET },
//     { "FAULT_MASK3", MC33771C_FAULT_MASK3_OFFSET },
//     { "WAKEUP_MASK1", MC33771C_WAKEUP_MASK1_OFFSET },
//     { "WAKEUP_MASK2", MC33771C_WAKEUP_MASK2_OFFSET },
//     { "WAKEUP_MASK3", MC33771C_WAKEUP_MASK3_OFFSET },
//     { "TH_ALL_CT", MC33771C_TH_ALL_CT_OFFSET },
//     { "TH_CT14", MC33771C_TH_CT14_OFFSET },
//     { "TH_CT13", MC33771C_TH_CT13_OFFSET },
//     { "TH_CT12", MC33771C_TH_CT12_OFFSET },
//     { "TH_CT11", MC33771C_TH_CT11_OFFSET },
//     { "TH_CT10", MC33771C_TH_CT10_OFFSET },
//     { "TH_CT9", MC33771C_TH_CT9_OFFSET },
//     { "TH_CT8", MC33771C_TH_CT8_OFFSET },
//     { "TH_CT7", MC33771C_TH_CT7_OFFSET },
//     { "TH_CT6", MC33771C_TH_CT6_OFFSET },
//     { "TH_CT5", MC33771C_TH_CT5_OFFSET },
//     { "TH_CT4", MC33771C_TH_CT4_OFFSET },
//     { "TH_CT3", MC33771C_TH_CT3_OFFSET },
//     { "TH_CT2", MC33771C_TH_CT2_OFFSET },
//     { "TH_CT1", MC33771C_TH_CT1_OFFSET },
//     { "TH_AN6_OT", MC33771C_TH_AN6_OT_OFFSET },
//     { "TH_AN5_OT", MC33771C_TH_AN5_OT_OFFSET },
//     { "TH_AN4_OT", MC33771C_TH_AN4_OT_OFFSET },
//     { "TH_AN3_OT", MC33771C_TH_AN3_OT_OFFSET },
//     { "TH_AN2_OT", MC33771C_TH_AN2_OT_OFFSET },
//     { "TH_AN1_OT", MC33771C_TH_AN1_OT_OFFSET },
//     { "TH_AN0_OT", MC33771C_TH_AN0_OT_OFFSET },
//     { "TH_AN6_UT", MC33771C_TH_AN6_UT_OFFSET },
//     { "TH_AN5_UT", MC33771C_TH_AN5_UT_OFFSET },
//     { "TH_AN4_UT", MC33771C_TH_AN4_UT_OFFSET },
//     { "TH_AN3_UT", MC33771C_TH_AN3_UT_OFFSET },
//     { "TH_AN2_UT", MC33771C_TH_AN2_UT_OFFSET },
//     { "TH_AN1_UT", MC33771C_TH_AN1_UT_OFFSET },
//     { "TH_AN0_UT", MC33771C_TH_AN0_UT_OFFSET },
//     { "TH_ISENSE OC", MC33771C_TH_ISENSE_OC_OFFSET },
//     { "TH_COULOMB_CNT_MSB", MC33771C_TH_COULOMB_CNT_MSB_OFFSET },
//     { "TH_COULOMB_CNT_LSB", MC33771C_TH_COULOMB_CNT_LSB_OFFSET }
// };

// static const bcc_drv_register_t s_confRegsMc33772c[MC33772C_REG_CONF_CNT] = {
//     /* Note: INIT register is initialized automatically. */
//     /* Note: SYS_CFG_GLOBAL reg. contains only command GO2SLEEP
//      * (no initialization needed). */
//     { "SYS_CFG1", MC33771C_SYS_CFG1_OFFSET },
//     { "SYS_CFG2", MC33771C_SYS_CFG2_OFFSET },
//     { "SYS_DIAG", MC33771C_SYS_DIAG_OFFSET },
//     { "ADC_CFG", MC33771C_ADC_CFG_OFFSET },
//     { "ADC2_OFFSET_COMP", MC33771C_ADC2_OFFSET_COMP_OFFSET },
//     { "OV_UV_EN", MC33771C_OV_UV_EN_OFFSET },
//     { "TPL_CFG", MC33771C_TPL_CFG_OFFSET },
//     { "CB1_CFG", MC33771C_CB1_CFG_OFFSET },
//     { "CB2_CFG", MC33771C_CB2_CFG_OFFSET },
//     { "CB3_CFG", MC33771C_CB3_CFG_OFFSET },
//     { "CB4_CFG", MC33771C_CB4_CFG_OFFSET },
//     { "CB5_CFG", MC33771C_CB5_CFG_OFFSET },
//     { "CB6_CFG", MC33771C_CB6_CFG_OFFSET },
//     { "GPIO_CFG1", MC33771C_GPIO_CFG1_OFFSET },
//     { "GPIO_CFG2", MC33771C_GPIO_CFG2_OFFSET },
//     { "FAULT_MASK1", MC33771C_FAULT_MASK1_OFFSET },
//     { "FAULT_MASK2", MC33771C_FAULT_MASK2_OFFSET },
//     { "FAULT_MASK3", MC33771C_FAULT_MASK3_OFFSET },
//     { "WAKEUP_MASK1", MC33771C_WAKEUP_MASK1_OFFSET },
//     { "WAKEUP_MASK2", MC33771C_WAKEUP_MASK2_OFFSET },
//     { "WAKEUP_MASK3", MC33771C_WAKEUP_MASK3_OFFSET },
//     { "TH_ALL_CT", MC33771C_TH_ALL_CT_OFFSET },
//     { "TH_CT6", MC33771C_TH_CT6_OFFSET },
//     { "TH_CT5", MC33771C_TH_CT5_OFFSET },
//     { "TH_CT4", MC33771C_TH_CT4_OFFSET },
//     { "TH_CT3", MC33771C_TH_CT3_OFFSET },
//     { "TH_CT2", MC33771C_TH_CT2_OFFSET },
//     { "TH_CT1", MC33771C_TH_CT1_OFFSET },
//     { "TH_AN6_OT", MC33771C_TH_AN6_OT_OFFSET },
//     { "TH_AN5_OT", MC33771C_TH_AN5_OT_OFFSET },
//     { "TH_AN4_OT", MC33771C_TH_AN4_OT_OFFSET },
//     { "TH_AN3_OT", MC33771C_TH_AN3_OT_OFFSET },
//     { "TH_AN2_OT", MC33771C_TH_AN2_OT_OFFSET },
//     { "TH_AN1_OT", MC33771C_TH_AN1_OT_OFFSET },
//     { "TH_AN0_OT", MC33771C_TH_AN0_OT_OFFSET },
//     { "TH_AN6_UT", MC33771C_TH_AN6_UT_OFFSET },
//     { "TH_AN5_UT", MC33771C_TH_AN5_UT_OFFSET },
//     { "TH_AN4_UT", MC33771C_TH_AN4_UT_OFFSET },
//     { "TH_AN3_UT", MC33771C_TH_AN3_UT_OFFSET },
//     { "TH_AN2_UT", MC33771C_TH_AN2_UT_OFFSET },
//     { "TH_AN1_UT", MC33771C_TH_AN1_UT_OFFSET },
//     { "TH_AN0_UT", MC33771C_TH_AN0_UT_OFFSET },
//     { "TH_ISENSE OC", MC33771C_TH_ISENSE_OC_OFFSET },
//     { "TH_COULOMB_CNT_MSB", MC33771C_TH_COULOMB_CNT_MSB_OFFSET },
//     { "TH_COULOMB_CNT_LSB", MC33771C_TH_COULOMB_CNT_LSB_OFFSET }
// };

// /*******************************************************************************
//  * API
//  ******************************************************************************/

// /*FUNCTION**********************************************************************
//  *
//  * Function Name : sendNops
//  * Description   : This function sends NOP commands to all CIDs.
//  *
//  *END**************************************************************************/
// void sendNops()
// {
//     uint8_t cid;

//     for (cid = 1; cid <= g_bccData.drvConfig.devicesCnt; cid++)
//     {
//         BCC_SendNop(&g_bccData.drvConfig, (bcc_cid_t)cid);
//     }
// }

// /*FUNCTION**********************************************************************
//  *
//  * Function Name : printInitialSettings
//  * Description   : This function prints content of registers configured in
//  *                 initialization phase to serial console output.
//  *
//  *END**************************************************************************/
// bcc_status_t printInitialSettings(bcc_cid_t cid)
// {
//     uint64_t guid;
//     uint16_t regVal;
//     uint8_t i;
//     std::string printPattern = "  | %-18s | 0x%02X%02X |\r\n";
//     bcc_status_t error;

//     Serial.printf("###############################################\r\n");
//     Serial.printf("# CID %d (MC3377%sC): Initial value of registers\r\n", cid,
//             (g_bccData.drvConfig.device[cid - 1] == BCC_DEVICE_MC33771C) ?
//                     "1" : "2");
//     Serial.printf("###############################################\r\n\r\n");

//     Serial.printf("  -------------------------------\r\n");
//     Serial.printf("  | Register           | Value  |\r\n");
//     Serial.printf("  -------------------------------\r\n");

//     error = BCC_Reg_Read(&g_bccData.drvConfig, cid, MC33771C_INIT_OFFSET, 1U, &regVal);
//     if (error != BCC_STATUS_SUCCESS)
//     {
//         return error;
//     }

//     Serial.printf(printPattern.c_str(), "INIT", regVal >> 8, regVal & 0xFFU);

//     if (g_bccData.drvConfig.device[cid - 1] == BCC_DEVICE_MC33771C)
//     {
//         for (i = 0U; i < MC33771C_REG_CONF_CNT; i++)
//         {
//             error = BCC_Reg_Read(&g_bccData.drvConfig, cid,
//                     s_confRegsMc33771c[i].address, 1U, &regVal);
//             if (error != BCC_STATUS_SUCCESS)
//             {
//                 return error;
//             }
//             Serial.printf(printPattern.c_str(), s_confRegsMc33771c[i].name,
//                     regVal >> 8, regVal & 0xFFU);
//         }
//     }
//     else
//     {
//         for (i = 0U; i < MC33772C_REG_CONF_CNT; i++)
//         {
//             error = BCC_Reg_Read(&g_bccData.drvConfig, cid,
//                     s_confRegsMc33772c[i].address, 1U, &regVal);
//             if (error != BCC_STATUS_SUCCESS)
//             {
//                 return error;
//             }
//             Serial.printf(printPattern.c_str(), s_confRegsMc33772c[i].name,
//                     regVal >> 8, regVal & 0xFFU);
//         }
//     }

//     Serial.printf("  -------------------------------\r\n");
//     Serial.printf("\r\n");

//     Serial.printf("  ------------------------\r\n");
//     Serial.printf("  | Fuse Mirror | Value  |\r\n");
//     Serial.printf("  |  Register   |        |\r\n");
//     Serial.printf("  ------------------------\r\n");
//     for (i = 0U; i <= ((g_bccData.drvConfig.device[cid - 1] == BCC_DEVICE_MC33771C) ?
//             MC33771C_LAST_FUSE_ADDR : MC33772C_LAST_FUSE_ADDR); i++)
//     {
//         error = BCC_FuseMirror_Read(&g_bccData.drvConfig, cid, i, &regVal);
//         if (error != BCC_STATUS_SUCCESS)
//         {
//             return error;
//         }

//         Serial.printf("  | $%02X         | 0x%02X%02X |\r\n", i, regVal >> 8, regVal & 0xFFU);
//     }
//     Serial.printf("  ------------------------\r\n");
//     Serial.printf("\r\n");

//     error = BCC_GUID_Read(&g_bccData.drvConfig, cid, &guid);
//     if (error != BCC_STATUS_SUCCESS)
//     {
//         return error;
//     }

//     Serial.printf("  Device GUID: %02X%04X%04X\r\n",
//             (uint16_t)((guid >> 32) & 0x001FU),
//             (uint16_t)((guid >> 16) & 0xFFFFU),
//             (uint16_t)(guid & 0xFFFFU));
//     Serial.printf("\r\n");

//     return BCC_STATUS_SUCCESS;
// }

// /*FUNCTION**********************************************************************
//  *
//  * Function Name : printFaultRegisters
//  * Description   : This function prints content of fault registers to serial
//  *                 console output.
//  *
//  *END**************************************************************************/
// bcc_status_t printFaultRegisters(bcc_cid_t cid)
// {
//     uint16_t status[BCC_STAT_CNT]; /* Status registers. */
//     bcc_status_t error;

//     Serial.printf("###############################################\r\n");
//     Serial.printf("# CID %d (MC3377%sC): Device status\r\n", cid,
//             (g_bccData.drvConfig.device[cid - 1] == BCC_DEVICE_MC33771C) ?
//                     "1" : "2");
//     Serial.printf("###############################################\r\n\r\n");

//     error = BCC_Fault_GetStatus(&g_bccData.drvConfig, cid, status);
//     if (error != BCC_STATUS_SUCCESS)
//     {
//         return error;
//     }

//     Serial.printf("  ----------------------------------------------------\r\n");
//     Serial.printf("  | Status\t| Raw Value\t| An event occurred? |\r\n");
//     Serial.printf("  ----------------------------------------------------\r\n");

//     PRINT_STATUS_REG_COMP("CELL_OV", status[BCC_FS_CELL_OV],
//             BCC_CELL_OV_FLT_NOEVENT);
//     PRINT_STATUS_REG_COMP("CELL_UV", status[BCC_FS_CELL_UV],
//             BCC_CELL_UV_FLT_NOEVENT);
//     PRINT_STATUS_REG_COMP("CB_OPEN", status[BCC_FS_CB_OPEN],
//             BCC_CB_OPEN_FLT_NOEVENT);
//     PRINT_STATUS_REG_COMP("CB_SHORT", status[BCC_FS_CB_SHORT],
//             BCC_CB_SHORT_FLT_NOEVENT);
//     PRINT_STATUS_REG_COMP("AN_OT_UT", status[BCC_FS_AN_OT_UT],
//             BCC_AN_OT_UT_FLT_NOEVENT);
//     PRINT_STATUS_REG_COMP("GPIO_SHORT", status[BCC_FS_GPIO_SHORT],
//             BCC_GPIO_SHORT_NOEVENT);
//     PRINT_STATUS_REG_COMP("FAULT1", status[BCC_FS_FAULT1],
//             BCC_FAULT1_STATUS_NOEVENT);
//     PRINT_STATUS_REG_COMP("FAULT2", status[BCC_FS_FAULT2],
//             BCC_FAULT2_STATUS_NOEVENT);
//     PRINT_STATUS_REG_COMP("FAULT3", status[BCC_FS_FAULT3],
//             BCC_FAULT3_STATUS_NOEVENT);

//     /* Note: GPIO_STS is not a fault register. */
//     PRINT_STATUS_REG("GPIO_STS", status[BCC_FS_GPIO_STATUS], "-");

//     /* Note: COM_STATUS is not a fault register. */
//     PRINT_STATUS_REG("COM_STATUS", status[BCC_FS_COMM], "-");

//     Serial.printf("  ----------------------------------------------------\r\n\r\n");

// #ifdef TPL
//     if ((g_bccData.drvConfig.device[cid - 1] == BCC_DEVICE_MC33771C) &&
//         (status[BCC_FS_FAULT1] & 0x0040))
//     {
//         Serial.printf("  Note: Bit 6 of FAULT1 register is given by the absence of EEPROM memory on the EVB. Thus, it does not indicate any fault.\r\n\r\n");
//     }
// #endif

//     /* Notes:
//     * - The fault bits in the CELL_OV register are ORed to the FAULT1_STATUS
//     * [CT_OV_FLT] bit.
//     * - The fault bits in the CELL_UV register are ORed to the FAULT1_STATUS
//     * [CT_UV_FLT] bit.
//     */

//     return BCC_STATUS_SUCCESS;
// }
