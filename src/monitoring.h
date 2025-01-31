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

// #ifndef MONITORING_H_
// #define MONITORING_H_

// #include "bcc/bcc.h"

// /*******************************************************************************
//  * Definitions
//  ******************************************************************************/

// /* NTC precomputed table configuration. */
// /*! @brief Minimal temperature in NTC table.
//  *
//  * It directly influences size of the NTC table (number of precomputed values).
//  * Specifically lower boundary.
//  */
// #define NTC_MINTEMP           (-40)

// /*! @brief Maximal temperature in NTC table.
//  *
//  * It directly influences size of the NTC table (number of precomputed values).
//  * Specifically higher boundary.
//  */
// #define NTC_MAXTEMP           (120)

// /*******************************************************************************
//  * Structure definition
//  ******************************************************************************/

// /*!
// * @brief NTC Configuration.
// *
// * The device has seven GPIOs which enable temperature measurement.
// * NTC thermistor and fixed resistor are external components and must be set
// * by the user. These values are used to calculate temperature. Beta parameter
// * equation is used to calculate temperature. GPIO port of BCC device must be
// * configured as Analog Input to measure temperature.
// * This configuration is common for all GPIO ports and all devices (in case of
// * daisy chain).
// */
// typedef struct
// {
//     uint32_t beta;         /*!< Beta parameter of NTC thermistor in [K].
//                                 Admissible range is from 1 to 1000000. */
//     uint32_t rntc;         /*!< R_NTC - NTC fixed resistance in [Ohm].
//                                 Admissible range is from 1 to 1000000. */
//     uint32_t refRes;       /*!< NTC Reference Resistance in [Ohm].
//                                 Admissible range is from 1 to 1000000. */
//     uint8_t refTemp;       /*!< NTC Reference Temperature in degrees [Celsius].
//                                 Admissible range is from 0 to 200. */
// } ntc_config_t;

// /*******************************************************************************
//  * API
//  ******************************************************************************/

// /*!
//  * @brief This function fills the NTC look up table.
//  *
//  * NTC look up table is intended for resistance to temperature conversion.
//  * An array item contains raw value from a register. Index of the item is
//  * temperature value.
//  *
//  * ArrayItem = (Vcom * Rntc) / (0.00015258789 * (NTC + Rntc))
//  * Where:
//  *  - ArrayItem is an item value of the table,
//  *  - Vcom is maximal voltage (5V),
//  *  - NTC is the resistance of NTC thermistor (Ohm),
//  *  - 0.00015258789 is resolution of measured voltage in Volts
//  *    (V = 152.58789 uV * Register_value),
//  *  - Rntc is value of a resistor connected to Vcom (see MC3377x datasheet,
//  *    section MC3377x PCB components).
//  *
//  * Beta formula used to calculate temperature based on NTC resistance:
//  *   1 / T = 1 / T0 + (1 / Beta) * ln(Rt / R0)
//  * Where:
//  *  - R0 is the resistance (Ohm) at temperature T0 (Kelvin),
//  *  - Beta is material constant (Kelvin),
//  *  - T is temperature corresponding to resistance of the NTC thermistor.
//  *
//  * Equation for NTC value is given from the Beta formula:
//  *   NTC = R0 * exp(Beta * (1/T - 1/T0))
//  *
//  * @param ntcConfig Pointer to NTC components configuration.
//  */
// void fillNtcTable(const ntc_config_t* const ntcConfig);

// /*!
//  * @brief This function reads the measurement registers and print them to serial
//  * console output.
//  *
//  * @param cid Cluster Identification Address.
//  *
//  * @return Error code (BCC_STATUS_SUCCESS - no error).
//  */
// bcc_status_t doMeasurements(uint8_t cid);

// #endif /* MONITORING_H_ */
