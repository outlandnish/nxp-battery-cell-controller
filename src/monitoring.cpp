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

// #include <math.h>
// #include "bcc/bcc.h"
// #include "common.h"            /* SPI, TPL macros */
// #include "monitoring.h"

// /*******************************************************************************
//  * Definitions
//  ******************************************************************************/

// /*! @brief Size of NTC look-up table. */
// #define NTC_TABLE_SIZE        (NTC_MAXTEMP - NTC_MINTEMP + 1)
// /*! @brief 0 degree Celsius converted to Kelvin. */
// #define NTC_DEGC_0            273.15

// /*!
//  * @brief Calculates final temperature value.
//  *
//  * @param tblIdx    Index of value in NTC table which is close
//  *                  to the register value provided by user.
//  * @param degTenths Fractional part of temperature value.
//  * @return Temperature.
//  */
// #define NTC_COMP_TEMP(tblIdx, degTenths) \
//     ((((tblIdx) + NTC_MINTEMP) * 10) + (degTenths))

// /*******************************************************************************
//  * Global variables (constants)
//  ******************************************************************************/

// /**
//  * NTC look up table intended for resistance to temperature conversion. After
//  * table initialization, array item contains raw value from a register.
//  * Index of the item is temperature value.
//  */
// uint16_t g_ntcTable[NTC_TABLE_SIZE];

// /*******************************************************************************
//  * Function prototypes
//  ******************************************************************************/

// /*!
//  * @brief This function calculates temperature from raw value of MEAS_ANx
//  * register. It uses precalculated values stored in g_ntcTable table.
//  *
//  * If the temperature is higher than NTC_MAXTEMP, it returns NTC_MAXTEMP.
//  * If the temperature is lower than NTC_MINTEMP, it returns NTC_MINTEMP.
//  *
//  * @param regVal Value of MEAS_ANx register. You can use function
//  *               BCC_Meas_GetRawValues to get values of measurement registers.
//  *
//  * @return Temperature value in deg. of Celsius * 10.
//  */
// int16_t getNtcCelsius(uint16_t regVal);

// /*!
//  * @brief This function starts on-demand conversion and reads measured values.
//  *
//  * @param cid         Cluster Identification Address.
//  * @param measurement Content of the measurement registers (result).
//  *
//  * @return bcc_status_t Error code.
//  */
// bcc_status_t getMeasurements(uint8_t cid, uint16_t measurements[]);

// /*!
//  * @brief This function prints value of a register to serial console output.
//  *
//  * @param regName Name of a register to be printed.
//  * @param rawVal  Raw value of a register.
//  * @param resVal  Interpreted value of a register.
//  * @param unit    Unit of interpreted value.
//  */
// void printMeas(const char *regName, uint16_t rawVal, uint32_t resVal,
//     const char *unit);

// /*!
//  * @brief This function converts raw value of ANx register to temperature.
//  * Result is printed to serial console output.
//  *
//  * @param regName Name of a register to be printed.
//  * @param regVal  Raw value from ANx register.
//  */
// void printANxTemp(const char *regName, uint16_t regVal);

// /*!
//  * @brief This function prints content of the measurement registers to serial
//  * console output.
//  *
//  * @param measurement Values of measurement registers.
//  * @param cid         Cluster Identification Address.
//  */
// void printMeasResults(uint16_t measurements[], uint8_t cid);

// /*******************************************************************************
//  * Private functions
//  ******************************************************************************/

// /*FUNCTION**********************************************************************
//  *
//  * Function Name : getNtcCelsius
//  * Description   : This function calculates temperature from raw value of
//  *                 MEAS_ANx register.
//  *
//  *END**************************************************************************/
// int16_t getNtcCelsius(uint16_t regVal)
// {
//     int16_t left = 0;    /* Pointer (index) to the left border of interval
//                             (NTC table). */
//     int16_t right = NTC_TABLE_SIZE - 1; /* Pointer (index) to the right border
//                                            of interval (NTC table). */
//     int16_t middle;      /* Pointer (index) to the middle of interval
//                             (NTC table). */
//     int8_t degTenths;    /* Fractional part of temperature value. */

//     /* Check range of NTC table. */
//     if (g_ntcTable[NTC_TABLE_SIZE - 1] > regVal)
//     {
//         return NTC_COMP_TEMP(NTC_TABLE_SIZE - 1, 0);
//     }
//     if (g_ntcTable[0] < regVal)
//     {
//         return NTC_COMP_TEMP(0, 0);
//     }

//     regVal &= BCC_GET_MEAS_RAW(regVal);

//     /* Search for an array item which is close to the register value provided
//     * by user (regVal). Used method is binary search in sorted array. */
//     while ((left + 1) != right)
//     {
//         /* Split interval into halves. */
//         middle = (left + right) >> 1U;
//         if (g_ntcTable[middle] <= regVal)
//         {
//             /* Select right half (array items are in descending order). */
//             right = middle;
//         }
//         else
//         {
//             /* Select left half. */
//             left = middle;
//         }
//     }

//     /* Notes: found table item (left) is less than the following item in the
//     * table (left + 1).
//     * The last item cannot be found (algorithm property). */

//     /* Calculate fractional part of temperature. */
//     degTenths = (g_ntcTable[left] - regVal) /
//             ((g_ntcTable[left] - g_ntcTable[left + 1]) / 10);
//     return NTC_COMP_TEMP(left, degTenths);
// }

// /*FUNCTION**********************************************************************
//  *
//  * Function Name : getMeasurements
//  * Description   : This function starts on-demand conversion and reads measured
//  *                 values.
//  *
//  *END**************************************************************************/
// static bcc_status_t getMeasurements(const bcc_cid_t cid, uint16_t measurements[])
// {
//     bcc_status_t error;

//     /* Start conversion and wait for the conversion time. */
//     error = BCC_Meas_StartAndWait(&g_bccData.drvConfig, cid, BCC_AVG_1);
//     if (error != BCC_STATUS_SUCCESS)
//     {
//         return error;
//     }

//     /* Read measured values. */
//     return BCC_Meas_GetRawValues(&g_bccData.drvConfig, cid, measurements);
// }

// /*FUNCTION**********************************************************************
//  *
//  * Function Name : printMeas
//  * Description   : This function prints value of a register to serial console
//  *                 output.
//  *
//  *END**************************************************************************/
// void printMeas(const char *regName, uint16_t rawVal, uint32_t resVal,
//     const char *unit)
// {
//     Serial.printf("  | %s\t| %d %s  \t| 0x%04x\t|\r\n", regName, resVal, unit, rawVal);
// }

// /*FUNCTION**********************************************************************
//  *
//  * Function Name : printANxTemp
//  * Description   : This function converts raw value of ANx register to
//  *                 temperature. Result is printed to serial console output.
//  *
//  *END**************************************************************************/
// void printANxTemp(const char *regName, uint16_t regVal)
// {
//     int16_t resVal;
//     int16_t degC;

//     resVal = getNtcCelsius(regVal);
//     degC = resVal / 10;

//     Serial.printf("  | %s\t| %d.%d degC\t| 0x%04x\t|\r\n", regName, degC,
//             (resVal > 0) ? resVal - degC * 10 : degC * 10 - resVal, regVal);
// }

// /*FUNCTION**********************************************************************
//  *
//  * Function Name : printMeasResults
//  * Description   : This function prints content of the measurement registers
//  *                 to serial console output.
//  *
//  *END**************************************************************************/
// void printMeasResults(uint16_t measurements[], uint8_t cid)
// {
//     uint32_t rawVal;   /* Raw value read from registers. */
//     int32_t resVal;    /* Converted value. */

//     Serial.printf("###############################################\r\n");
//     Serial.printf("# CID %d (MC3377%sC): Measurements\r\n", cid,
//             (g_bccData.drvConfig.device[cid - 1] == BCC_DEVICE_MC33771C) ?
//                     "1" : "2");
//     Serial.printf("###############################################\r\n\r\n");

//     Serial.printf("  -----------------------------------------------\r\n");
//     Serial.printf("  | Measurement | Value    \t| Raw value\t|\r\n");
//     Serial.printf("  -----------------------------------------------\r\n");


//     /* MC33771C TPL EVB does not support current measurement. */
//     if ((g_bccData.drvConfig.device[cid - 1] == BCC_DEVICE_MC33772C) ||
//         (g_bccData.drvConfig.commMode == BCC_MODE_SPI))
//     {
//         /* Content of CC registers (raw values only).
//          * CC registers resets on read. */
//         resVal = BCC_GET_COULOMB_CNT(measurements[BCC_MSR_COULOMB_CNT1],
//                 measurements[BCC_MSR_COULOMB_CNT2]);
//         rawVal = (uint32_t)resVal;
//         Serial.printf("  | C CNT\t| %d  \t| 0x%08x\t|\r\n",  resVal, rawVal);
//         printMeas("CC SAMPLES", measurements[BCC_MSR_CC_NB_SAMPLES],
//                 measurements[BCC_MSR_CC_NB_SAMPLES], " ");

//         /* ISENSE value in uV. */
//         rawVal = BCC_GET_ISENSE_RAW(measurements[BCC_MSR_ISENSE1],
//                 measurements[BCC_MSR_ISENSE2]);
//         resVal = BCC_GET_ISENSE_VOLT(measurements[BCC_MSR_ISENSE1],
//                 measurements[BCC_MSR_ISENSE2]);
//         Serial.printf("  | ISENSE\t| %d uV \t| 0x%08x\t|\r\n", resVal, rawVal);

//         /* ISENSE value in mA. */
//         resVal = BCC_GET_ISENSE_AMP(DEMO_RSHUNT, measurements[BCC_MSR_ISENSE1],
//                 measurements[BCC_MSR_ISENSE2]);
//         Serial.printf("  | ISENSE\t| %d mA \t| 0x%08x\t|\r\n", resVal, rawVal);
//     }

//     /* Stack voltage. */
//     printMeas("STACK", measurements[BCC_MSR_STACK_VOLT],
//             BCC_GET_STACK_VOLT(measurements[BCC_MSR_STACK_VOLT]) / 1000U, "mV");

//     /* Cells voltage. */
//     printMeas("CELL 1", measurements[BCC_MSR_CELL_VOLT1],
//             BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT1]) / 1000U, "mV");
//     printMeas("CELL 2", measurements[BCC_MSR_CELL_VOLT2],
//             BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT2]) / 1000U, "mV");
//     printMeas("CELL 3", measurements[BCC_MSR_CELL_VOLT3],
//             BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT3]) / 1000U, "mV");
//     printMeas("CELL 4", measurements[BCC_MSR_CELL_VOLT4],
//             BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT4]) / 1000U, "mV");
//     printMeas("CELL 5", measurements[BCC_MSR_CELL_VOLT5],
//             BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT5]) / 1000U, "mV");
//     printMeas("CELL 6", measurements[BCC_MSR_CELL_VOLT6],
//             BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT6]) / 1000U, "mV");

//     /* Send NOP command to all nodes in order to prevent communication timeout. */
//     sendNops();

//     if (g_bccData.drvConfig.device[cid - 1] == BCC_DEVICE_MC33771C)
//     {
//         printMeas("CELL 7", measurements[BCC_MSR_CELL_VOLT7],
//                 BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT7]) / 1000U, "mV");
//         printMeas("CELL 8", measurements[BCC_MSR_CELL_VOLT8],
//                 BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT8]) / 1000U, "mV");
//         printMeas("CELL 9", measurements[BCC_MSR_CELL_VOLT9],
//                 BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT9]) / 1000U, "mV");
//         printMeas("CELL 10", measurements[BCC_MSR_CELL_VOLT10],
//                 BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT10]) / 1000U, "mV");
//         printMeas("CELL 11", measurements[BCC_MSR_CELL_VOLT11],
//                 BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT11]) / 1000U, "mV");
//         printMeas("CELL 12", measurements[BCC_MSR_CELL_VOLT12],
//                 BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT12]) / 1000U, "mV");
//         printMeas("CELL 13", measurements[BCC_MSR_CELL_VOLT13],
//                 BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT13]) / 1000U, "mV");
//         printMeas("CELL 14", measurements[BCC_MSR_CELL_VOLT14],
//                 BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT14]) / 1000U, "mV");
//     }

//     /* Analog inputs measurements.
//      * MC33771C TPL: Temperature measurements at all GPIOx pins.
//      * MC33771C SPI: Temperature measurements at GPIO1 and GPIO3-6 pins.
//      * MC33772C TPL: Temperature measurements at GPIO0-4 pins and
//      *               Voltage measurement at GPIO5 and GPIO6.
//      * MC33772C SPI: Temperature measurements at GPIO1, GPIO3, GPIO4 pins and
//      *               Voltage measurement at GPIO5 and GPIO6. */
// #ifdef TPL
//     printANxTemp("AN 0", measurements[BCC_MSR_AN0]);
// #endif
//     printANxTemp("AN 1", measurements[BCC_MSR_AN1]);
// #ifdef TPL
//     printANxTemp("AN 2", measurements[BCC_MSR_AN2]);
// #endif
//     printANxTemp("AN 3", measurements[BCC_MSR_AN3]);
//     printANxTemp("AN 4", measurements[BCC_MSR_AN4]);

//     if (g_bccData.drvConfig.device[cid - 1] == BCC_DEVICE_MC33771C)
//     {
//         printANxTemp("AN 5", measurements[BCC_MSR_AN5]);
//         printANxTemp("AN 6", measurements[BCC_MSR_AN6]);
//     }
//     else
//     {
//         printMeas("AN 5", measurements[BCC_MSR_AN5],
//                 BCC_GET_VOLT(measurements[BCC_MSR_AN5]) / 1000U, "mV");
//         printMeas("AN 6", measurements[BCC_MSR_AN6],
//                 BCC_GET_VOLT(measurements[BCC_MSR_AN6]) / 1000U, "mV");
//     }

//     /* IC temperature measurement. */
//     resVal = BCC_GET_IC_TEMP_C(measurements[BCC_MSR_ICTEMP]);
//     Serial.printf("  | IC TEMP\t| %d.%d degC\t| 0x%04x\t|\r\n", resVal/10,
//             (resVal > 0) ? resVal % 10 : (-resVal) % 10,
//             measurements[BCC_MSR_ICTEMP]);


//     /* ADCIA and ADCIB Band Gap Reference measurements. */
//     printMeas("VBG ADC1A", measurements[BCC_MSR_VBGADC1A],
//             BCC_GET_VOLT(measurements[BCC_MSR_VBGADC1A]) / 1000U, "mV");
//     printMeas("VBG ADC1B", measurements[BCC_MSR_VBGADC1B],
//             BCC_GET_VOLT(measurements[BCC_MSR_VBGADC1B]) / 1000U, "mV");

//     Serial.printf("  -----------------------------------------------\r\n");
//     Serial.printf("\r\n");
// }

// /*******************************************************************************
//  * API
//  ******************************************************************************/

// /*FUNCTION**********************************************************************
//  *
//  * Function Name : BCC_FillNtcTable
//  * Description   : This function fills the NTC look up table.
//  *
//  *END**************************************************************************/
// void fillNtcTable(const ntc_config_t* const ntcConfig)
// {
//     double ntcVal, expArg;
//     uint16_t i = 0;
//     int32_t temp;

//     for (temp = NTC_MINTEMP; temp <= NTC_MAXTEMP; temp++)
//     {
//         expArg = ntcConfig->beta * ((1.0 / (NTC_DEGC_0 + temp)) -
//                 (1.0 / (NTC_DEGC_0 + ntcConfig->refTemp)));
//         ntcVal = exp(expArg) * ntcConfig->refRes;
//         g_ntcTable[i] = (uint16_t)round((ntcVal /
//                 (ntcVal + ntcConfig->rntc)) * MC33771C_MEAS_AN0_MEAS_AN_MASK);
//         i++;
//     }
// }

// /*FUNCTION**********************************************************************
//  *
//  * Function Name : doMeasurements
//  * Description   : This function reads the measurement registers and print them
//  *                 to serial console output.
//  *
//  *END**************************************************************************/
// bcc_status_t doMeasurements(uint8_t cid)
// {
//     uint16_t measurements[BCC_MEAS_CNT];
//     bcc_status_t error;

//     if ((error = getMeasurements(cid, measurements)) != BCC_STATUS_SUCCESS)
//     {
//         return error;
//     }

//     printMeasResults(measurements, cid);

//     return BCC_STATUS_SUCCESS;
// }
