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

// /*!
//  * @file bcc.h
//  *
//  * Battery cell controller SW driver for MC33771C and MC33772C v2.2.
//  */

// #pragma once

// /*******************************************************************************
//  * Includes
//  ******************************************************************************/

// #include <stdbool.h>
// #include <stddef.h>

// #include "models.h"
// /*! @} */

// /*******************************************************************************
//  * API
//  ******************************************************************************/

// /*!
//  * @addtogroup function_group
//  * @{
//  */

// /*!
//  * @brief This function initializes the battery cell controller device(s),
//  * assigns CID and initializes internal driver data.
//  *
//  * Note that this function initializes only the INIT register of connected
//  * BCC device(s).
//  *
//  * Note that the function is implemented for an universal use. It is capable to
//  * move BCC devices into the NORMAL mode from all modes (INIT, IDLE, SLEEP and
//  * NORMAL) except the RESET mode. Such implementation can generate more wake-up
//  * sequences via CSB_TX pin than required for the current mode. These extra
//  * sequences project to the COM_ERR_COUNT bit field in COM_STATUS register.
//  * Therefore, it is recommended to clear the COM_STATUS register after calling
//  * of this function.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Init(bcc_drv_config_t* const drvConfig);

// /*!
//  * @brief This function sends No Operation command the to BCC device. It can be
//  * used to reset the communication timeout of the device without performing
//  * any operation.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_SendNop(bcc_drv_config_t* const drvConfig, const bcc_cid_t cid);

// /*!
//  * @brief This function sets sleep mode to all battery cell controller devices.
//  *
//  * In case of TPL communication mode, MC33664 has to be put into the sleep mode
//  * separately, by the BCC_TPL_Disable function.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Sleep(bcc_drv_config_t* const drvConfig);

// /*!
//  * @brief This function sets normal mode to all battery cell controller devices.
//  *
//  * It is recommended to perform a check whether the device(s) is already woken
//  * up by reading the FAULT or the INTB pin before calling this function. If the
//  * device is already woken up, this function will increase the COM_ERR_COUNT bit
//  * field in the COM_STATUS register and, moreover, will case so-called NULL
//  * response (all bit filed set to zero except message counter and correct CRC)
//  * in the following SPI response in case of the SPI communication mode (BCC
//  * device received an invalid request from MCU).
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  *
//  * @return bcc_status_t Error code.
//  */
// void BCC_WakeUp(const bcc_drv_config_t* const drvConfig);

// /*!
//  * @brief This function resets BCC device using software reset. It enters reset
//  * via SPI or TPL interface. In TPL, you can use BCC_CID_UNASSIG as CID
//  * parameter for software reset of all devices in the chain (global write).
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_SoftwareReset(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid);

// /*!
//  * @brief This function resets BCC device using RESET pin.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  */
// void BCC_HardwareReset(const bcc_drv_config_t* const drvConfig);

// /*!
//  * @brief This function enables MC33664 device (sets a normal mode).
//  * Intended for TPL mode only!
//  *
//  * During the driver initialization (BCC_Init function), normal mode of MC33664
//  * is set automatically. This function can be used when sleep mode of BCC
//  * device(s) and MC33664 together is required. The typical function flow is as
//  * follows: BCC_Sleep -> BCC_TPL_Disable -> ... -> BCC_TPL_Enable -> BCC_WakeUp.
//  *
//  * @param drvInstance BCC driver instance (content of the bcc_drv_config_t
//  *                    structure). Passed to the external functions defined by
//  *                    the user in order to distinguish between more TPL
//  *                    interfaces.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_TPL_Enable(const uint8_t drvInstance);

// /*!
//  * @brief This function puts MC33664 device into the sleep mode.
//  * Intended for TPL mode only!
//  *
//  * This function can be (optionally) used after BCC_Sleep function. Function
//  * BCC_TPL_Enable must be then called before BCC_WakeUp function!
//  *
//  * @param drvInstance BCC driver instance (content of the bcc_drv_config_t
//  *                    structure). Passed to the external functions defined by
//  *                    the user.
//  */
// void BCC_TPL_Disable(const uint8_t drvInstance);

// /*!
//  * @brief This function reads a value from addressed register (or desired
//  * number of registers) of selected battery cell controller device.
//  *
//  * In case of simultaneous read of more registers, address is incremented
//  * in ascending manner.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param regAddr   Register address. See MC33771C.h and MC33772C.h header files
//  *                  for possible values (MC3377*C_*_OFFSET macros).
//  * @param regCnt    Number of registers to be read.
//  * @param regVal    Pointer to memory where content of selected 16 bit registers
//  *                  is stored.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Reg_Read(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const uint8_t regAddr, const uint8_t regCnt,
//     uint16_t* regVal);

// /*!
//  * @brief This function writes a value to addressed register of selected battery
//  * cell controller device.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param regAddr   Register address. See MC33771C.h and MC33772C.h header files
//  *                  for possible values (MC3377*C_*_OFFSET macros).
//  * @param regVal    New value of selected register.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Reg_Write(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const uint8_t regAddr, const uint16_t regVal);

// /*!
//  * @brief This function writes a value to addressed register of all configured
//  * BCC devices. Intended for TPL mode only!
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param regAddr   Register address. See MC33771C.h and MC33772C.h header files
//  *                  for possible values (MC3377*C_*_OFFSET macros).
//  * @param regVal    New value of selected register.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Reg_WriteGlobal(bcc_drv_config_t* const drvConfig,
//      const uint8_t regAddr, const uint16_t regVal);

// /*!
//  * @brief This function updates content of a selected register; affects bits
//  * specified by a bit mask only.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param regAddr   Register address. See MC33771C.h and MC33772C.h header files
//  *                  for possible values (MC3377*C_*_OFFSET macros).
//  * @param regMask   Bit mask. Bits set to 1 will be updated.
//  * @param regVal    New value of register bits defined by bit mask.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Reg_Update(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const uint8_t regAddr, const uint16_t regMask,
//     const uint16_t regVal);

// /*!
//  * @brief This function starts ADC conversion in selected BCC device. It sets
//  * number of samples to be averaged and Start of Conversion bit in ADC_CFG
//  * register.
//  *
//  * You can use function BCC_Meas_IsConverting to check the conversion status or
//  * a blocking function BCC_Meas_StartAndWait instead.
//  * Measured values can be retrieved e.g. by function BCC_Meas_GetRawValues.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param avg       Number of samples to be averaged.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Meas_StartConversion(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const bcc_avg_t avg);

// /*!
//  * @brief This function starts ADC conversion for all devices in TPL chain. It
//  * uses a Global Write command to set ADC_CFG register. Intended for TPL mode
//  * only!
//  *
//  * You can use function BCC_Meas_IsConverting to check conversion status and
//  * function BCC_Meas_GetRawValues to retrieve the measured values.
//  *
//  * @param drvConfig   Pointer to driver instance configuration.
//  * @param adcCfgValue Value of ADC_CFG register to be written to all devices in
//  *                    the chain. Note that SOC bit is automatically added by
//  *                    this function.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Meas_StartConversionGlobal(bcc_drv_config_t* const drvConfig,
//     uint16_t adcCfgValue);

// /*!
//  * @brief This function checks status of conversion defined by End of Conversion
//  * bit in ADC_CFG register.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param completed Pointer to check result. True if a conversion is complete.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Meas_IsConverting(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, bool* const completed);

// /*!
//  * @brief This function starts an on-demand conversion in selected BCC device
//  * and waits for completion.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param avg       Number of samples to be averaged.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Meas_StartAndWait(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const bcc_avg_t avg);

// /*!
//  * @brief This function reads the measurement registers and returns raw values.
//  *
//  * Macros defined in bcc_utils.h can be used to perform correct unit conversion.
//  *
//  * Warning: The "data ready" flag is ignored.
//  *
//  * @param drvConfig    Pointer to driver instance configuration.
//  * @param cid          Cluster Identification Address of the BCC device.
//  * @param measurements Array containing all values measured by BCC. Size of this
//  *                     array must be BCC_MEAS_CNT*16b. Indexes into the array
//  *                     are defined in enumeration bcc_measurements_t
//  *                     placed in this header file.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Meas_GetRawValues(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, uint16_t* const measurements);

// /*!
//  * @brief This function reads the Coulomb counter registers.
//  *
//  * Note that the Coulomb counter is independent on the on-demand conversions.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param cc        Coulomb counter data.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Meas_GetCoulombCounter(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, bcc_cc_data_t* const cc);

// /*!
//  * @brief This function reads the ISENSE measurement and converts it to [uV].
//  *
//  * @param drvConfig  Pointer to driver instance configuration.
//  * @param cid        Cluster Identification Address of the BCC device.
//  * @param isenseVolt Pointer to memory where the ISENSE voltage (in [uV]) will
//  *                   be stored.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Meas_GetIsenseVoltage(bcc_drv_config_t* const drvConfig,
//    const bcc_cid_t cid, int32_t* const isenseVolt);

// /*!
//  * @brief This function reads the stack measurement and converts it to [uV].
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param stackVolt Pointer to memory where the stack voltage (in [uV]) will
//  *                  be stored.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Meas_GetStackVoltage(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, uint32_t* const stackVolt);

// /*!
//  * @brief This function reads the cell measurements and converts them to [uV].
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param cellVolt  Pointer to the array where the cell voltages (in [uV]) will
//  *                  be stored. Array must have an suitable size (14*32b for
//  *                  MC33771C and 6*32b for MC33772C). Cell 1 voltage is stored
//  *                  at [0], Cell 2 voltage at [1], etc.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Meas_GetCellVoltages(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, uint32_t* const cellVolt);

// /*!
//  * @brief This function reads the voltage measurement of a selected cell and
//  * converts it to [uV].
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param cellIndex Cell index. Use 0U for CELL 1, 1U for CELL2, etc.
//  * @param cellVolt  Pointer to memory where the cell voltage (in [uV]) will
//  *                  be stored.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Meas_GetCellVoltage(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, uint8_t cellIndex, uint32_t* const cellVolt);

// /*!
//  * @brief This function reads the voltage measurement for all ANx and converts
//  * them to [uV]. Intended for ANx configured for absolute measurements only!
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param anVolt    Pointer to the array where ANx voltages (in [uV]) will
//  *                  be stored. Array must have an suitable size
//  *                  (BCC_GPIO_INPUT_CNT * 32b). AN0 voltage is stored at [0],
//  *                  AN1 at [1], etc.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Meas_GetAnVoltages(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, uint32_t* const anVolt);

// /*!
//  * @brief This function reads the voltage measurement of a selected ANx and
//  * converts it to [uV]. Intended for ANx configured for absolute measurements
//  * only!
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param anIndex   ANx index. Use 0U for AN0, 1U for AN1, etc.
//  * @param anVolt    Pointer to memory where the ANx voltage (in [uV]) will
//  *                  be stored.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Meas_GetAnVoltage(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, uint8_t anIndex, uint32_t* const anVolt);

// /*!
//  * @brief This function reads the BCC temperature and converts it to the
//  * selected unit.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param unit      Temperature unit.
//  * @param icTemp    Pointer to memory where the IC temperature will be stored.
//  *                  Resolution is 0.1; unit is selected by parameter "unit".
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Meas_GetIcTemperature(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, bcc_temp_unit_t unit, int16_t* const icTemp);

// /*!
//  * @brief This function reads the fault status registers of the BCC device.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param fltStatus Array containing all fault status information provided by
//  *                  BCC. Required size of the array is BCC_STAT_CNT*16b.
//  *                  Indexes into the array are defined in bcc_fault_status_t
//  *                  enumeration placed in this header file.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Fault_GetStatus(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, uint16_t* const fltStatus);

// /*!
//  * @brief This function clears selected fault status register.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param statSel   Selection of a fault status register to be cleared. See
//  *                  definition of this enumeration in this header file.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_Fault_ClearStatus(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const bcc_fault_status_t statSel);

// /*!
//  * @brief This function sets the mode of one BCC GPIOx/ANx pin.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param gpioSel   Index of pin to be configured. Index starts at 0 (GPIO0).
//  * @param mode      Pin mode.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_GPIO_SetMode(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const uint8_t gpioSel, const bcc_pin_mode_t mode);

// /*!
//  * @brief This function reads a value of one BCC GPIO pin.
//  *
//  * Note that such GPIO/AN pin should be configured as digital input or output.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param gpioSel   Index of GPIOx to be read. Index starts at 0 (GPIO0).
//  * @param val       Pointer where the pin value will be stored. Possible values
//  *                  are: False (logic 0, low level) and True (logic 1, high
//  *                  level).
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_GPIO_ReadPin(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const uint8_t gpioSel, bool* const val);

// /*!
//  * @brief This function sets output value of one BCC GPIO pin.
//  *
//  * Note that this function is determined for GPIO/AN pins configured as output
//  * pins.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param gpioSel   Index of GPIO output to be set. Index starts at 0 (GPIO0).
//  * @param val       Output value. Possible values are: False (logic 0, low
//  *                  level) and True (logic 1, high level).
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_GPIO_SetOutput(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const uint8_t gpioSel, const bool val);

// /*!
//  * @brief This function enables or disables the cell balancing via
//  * SYS_CFG1[CB_DRVEN] bit.
//  *
//  * Note that each cell balancing driver needs to be setup separately, e.g. by
//  * BCC_CB_SetIndividual function.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param enable    State of cell balancing. False (cell balancing disabled) or
//  *                  True (cell balancing enabled).
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_CB_Enable(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const bool enable);

// /*!
//  * @brief This function enables or disables cell balancing for a specified cell
//  * and sets its timer.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param cellIndex Cell index. Use 0U for CELL 1, 1U for CELL2, etc.
//  * @param enable    True for enabling of CB, False otherwise.
//  * @param timer     Timer for enabled CB driver in minutes. Value of zero
//  *                  represents 30 seconds.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_CB_SetIndividual(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const uint8_t cellIndex, const bool enable,
//     const uint16_t timer);

// /*!
//  * @brief This function pauses cell balancing. It can be useful during an on
//  * demand conversion. As a result more precise measurement can be done. Note
//  * that it is user obligation to re-enable cell balancing after measurement
//  * ends.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param pause     True (pause) or False (unpause).
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_CB_Pause(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const bool pause);

// /*!
//  * @brief This function reads a fuse mirror register of selected BCC device.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param fuseAddr  Address of a fuse mirror register to be read.
//  * @param value     Pointer to memory where the read value will be stored.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_FuseMirror_Read(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const uint8_t fuseAddr, uint16_t* const value);

// /*!
//  * @brief This function writes a fuse mirror register of selected BCC device.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param fuseAddr  Address of a fuse mirror register to be written.
//  * @param value     Value to be written.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_FuseMirror_Write(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const uint8_t fuseAddr, const uint16_t value);

// /*!
//  * @brief This function reads an unique serial number of the BCC device from the
//  * content of mirror registers.
//  *
//  * GUID is created according to the following table:
//  * |    Device    | GUID [36:21] | GUID [20:5] | GUID [4:0] |
//  * |:------------:|:------------:|:-----------:|:----------:|
//  * |   MC33771C   | 0x18 [15:0]  | 0x19 [15:0] | 0x1A [4:0] |
//  * | fuse address |  (16 bit)    |  (16 bit)   |  (5 bit)   |
//  * |:------------:|:------------:|:-----------:|:----------:|
//  * |   MC33772C   | 0x10 [15:0]  | 0x11 [15:0] | 0x12 [4:0] |
//  * | fuse address |  (16 bit)    |  (16 bit)   |  (5 bit)   |
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of the BCC device.
//  * @param guid      Pointer to memory where 37b unique ID will be stored.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_GUID_Read(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, uint64_t* const guid);

// /*!
//  * @brief This function reads a byte from specified address of EEPROM memory
//  * connected to BCC device via I2C bus.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of BCC device the EEPROM
//  *                  memory is connected to.
//  * @param addr      Address of EEPROM data will be read from. The admission
//  *                  range is from 0 to 127.
//  * @param data      Data read from specified address of EEPROM memory.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_EEPROM_Read(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const uint8_t addr, uint8_t* const data);

// /*!
//  * @brief This function writes a byte to specified address of EEPROM memory
//  * connected to BCC device via I2C bus.
//  *
//  * Warning: EEPROM write time depends on device selection and is usually
//  * around 5 ms. Therefore, another EEPROM (write & read) operations to the same
//  * EEPROM memory cannot be done within 5 ms after returning from this function.
//  *
//  * @param drvConfig Pointer to driver instance configuration.
//  * @param cid       Cluster Identification Address of BCC device the EEPROM
//  *                  memory is connected to.
//  * @param addr      Address of EEPROM data will be written to. The admission
//  *                  range is from 0 to 127.
//  * @param data      Data written to specified address of EEPROM memory.
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t BCC_EEPROM_Write(bcc_drv_config_t* const drvConfig,
//     const bcc_cid_t cid, const uint8_t addr, const uint8_t data);

// /*******************************************************************************
//  * Platform specific functions
//  ******************************************************************************/

// /*!
//  * @brief Waits for specified amount of microseconds. This function needs to be
//  * implemented for specified MCU by the user.
//  *
//  * The timing within the implementation of this functions should be accurate.
//  * E.g. In case of timer usage, the time for setting and starting the timer
//  * should be taken also into account and compensated. Waiting time down to 5 us
//  * must be supported.
//  *
//  * Note: Functions BCC_MCU_WaitUs and BCC_MCU_WaitMs are both blocking. Thus,
//  * they are called independently by the BCC SW driver and can be implemented by
//  * the same timer.
//  *
//  * Note: Timeout mechanism (implemented by functions BCC_MCU_StartTimeout and
//  * BCC_MCU_TimeoutExpired) is also not used together with BCC_MCU_WaitUs.
//  * I.e. resources allocated by BCC_MCU_StartTimeout function can be released
//  * when BCC_MCU_WaitUs is called.
//  *
//  * @param delay Number of microseconds to wait.
//  */
// extern void BCC_MCU_WaitUs(uint32_t delay);

// /*!
//  * @brief Waits for specified amount of milliseconds. This function needs to be
//  * implemented for specified MCU by the user.
//  *
//  * Note: Functions BCC_MCU_WaitUs and BCC_MCU_WaitMs are both blocking. Thus,
//  * they are called independently by the BCC SW driver and can be implemented by
//  * the same timer.
//  *
//  * Note: Timeout mechanism (implemented by functions BCC_MCU_StartTimeout and
//  * BCC_MCU_TimeoutExpired) is also not used together with BCC_MCU_WaitMs.
//  * I.e. resources allocated by BCC_MCU_StartTimeout function can be released
//  * when BCC_MCU_WaitMs is called.
//  *
//  * @param delay Number of milliseconds to wait.
//  */
// extern void BCC_MCU_WaitMs(uint16_t delay);

// /*!
//  * @brief Starts a non-blocking timeout mechanism. After expiration of the time
//  * passed as a parameter, function BCC_MCU_TimeoutExpired should signalize an
//  * expired timeout. This function needs to be implemented for specified MCU by
//  * the user.
//  *
//  * Note: An optimization of this function to last as short as possible is
//  * recommended but not absolutely required for the proper function of the driver.
//  * If the timeout mechanism is implemented by a timer, the time needed for
//  * setting the timer does not have to be compensated - i.e. The timeout can be
//  * signalized a bit later than requested.
//  *
//  * Note: When function BCC_MCU_WaitUs or BCC_MCU_WaitMs is called, information
//  * about the timeout state will not be required by the BCC SW driver anymore
//  * and resources allocated by this function can be released even if the timeout
//  * has not expired yet.
//  *
//  * @param timeoutUs Length of the timeout in microseconds.
//  *
//  * @return Returns BCC_STATUS_TIMEOUT_START in case of error, BCC_STATUS_SUCCESS
//  *         otherwise.
//  */
// extern bcc_status_t BCC_MCU_StartTimeout(uint32_t timeoutUs);

// /*!
//  * @brief Returns state of the timeout mechanism started by the function
//  * BCC_MCU_StartTimeout. This function needs to be implemented for specified MCU
//  * by the user.
//  *
//  * @return True if timeout expired, false otherwise.
//  */
// extern bool BCC_MCU_TimeoutExpired(void);

// /*!
//  * @brief User implementation of assert.
//  *
//  * @param x True if everything is OK.
//  */
// // extern void BCC_MCU_Assert(const bool x);

// // /*!
// //  * @brief This function performs one 48b transfer via SPI bus. Intended for SPI
// //  * mode only. This function needs to be implemented for specified MCU by the
// //  * user.
// //  *
// //  * The byte order of buffers is given by BCC_MSG_* macros (in bcc.h).
// //  *
// //  * @param drvInstance Instance of BCC driver.
// //  * @param txBuf       Pointer to TX data buffer (of BCC_MSG_SIZE size).
// //  * @param rxBuf       Pointer to RX data buffer (of BCC_MSG_SIZE size).
// //  *
// //  * @return bcc_status_t Error code.
// //  */
// // extern bcc_status_t BCC_MCU_TransferSpi(const uint8_t drvInstance,
// //     uint8_t txBuf[], uint8_t rxBuf[]);

// // /*!
// //  * @brief This function sends and receives data to MC33664 via TX and RX SPI
// //  * buses. Intended for TPL mode only. This function needs to be implemented
// //  * for specified MCU by the user.
// //  *
// //  * TX SPI bus always performs only one 48b SPI transfer. Expected number of RX
// //  * transfers is passed as the last parameter. The byte order of buffers is given
// //  * by BCC_MSG_* macros (in bcc.h) and can be changed by the user.
// //  *
// //  * @param drvInstance Instance of BCC driver.
// //  * @param txBuf       Pointer to TX data buffer (of BCC_MSG_SIZE size).
// //  * @param rxBuf       Pointer to buffer for received data. Its size is at least
// //  *                    (BCC_MSG_SIZE * recvTrCnt) bytes.
// //  * @param recvTrCnt   Number of 48b transfers to be received. A non-zero value.
// //  *
// //  * @return bcc_status_t Error code.
// //  */
// // extern bcc_status_t BCC_MCU_TransferTpl(const uint8_t drvInstance,
// //     uint8_t txBuf[], uint8_t rxBuf[], const uint16_t recvTrCnt);

// // /*!
// //  * @brief Writes logic 0 or 1 to the CSB (SPI mode) or CSB_TX pin (TPL mode).
// //  * This function needs to be implemented by the user.
// //  *
// //  * @param drvInstance Instance of BCC driver.
// //  * @param value       Zero or one to be set to CSB_TX pin.
// //  */
// // extern void BCC_MCU_WriteCsbPin(const uint8_t drvInstance, const uint8_t value);

// // /*!
// //  * @brief Writes logic 0 or 1 to the RESET pin. This function needs to be
// //  * implemented by the user. If no RESET pin is used, keep the function body
// //  * empty.
// //  *
// //  * @param drvInstance Instance of BCC driver.
// //  * @param value       Zero or one to be set to RESET pin.
// //  */
// // extern void BCC_MCU_WriteRstPin(const uint8_t drvInstance, const uint8_t value);

// // /*!
// //  * @brief Writes logic 0 or 1 to the EN pin of MC33664. This function is
// //  * called only in the TPL mode and it needs to be implemented by the user.
// //  *
// //  * @param drvInstance Instance of BCC driver.
// //  * @param value       Zero or one to be set to EN pin.
// //  */
// // extern void BCC_MCU_WriteEnPin(const uint8_t drvInstance, const uint8_t value);

// // /*!
// //  * @brief Reads logic value of INTB pin of MC33664. This function is
// //  * called only in the TPL mode and it needs to be implemented by the user.
// //  *
// //  * @param drvInstance Instance of BCC driver.
// //  *
// //  * @return Zero value for logic zero, non-zero value otherwise.
// //  */
// // extern uint32_t BCC_MCU_ReadIntbPin(const uint8_t drvInstance);

// /*! @} */

