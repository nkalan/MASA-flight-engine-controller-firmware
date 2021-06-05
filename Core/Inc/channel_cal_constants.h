/*
 * channel_cal_constants.h
 *
 *  Created on: Jun 4, 2021
 *      Author: natha
 *
 *
 *  File containing ADC channel voltage divider resistor values.
 *  The function converts the adc counts output to (transducer) voltage.
 *
 *  There are 22 ADC channels total on the board, split across several ADCs.
 *  Specify the input (counts) and the sensor channel (0-21),
 *  NOT the raw ADC channel (0-15)
 */

#ifndef INC_CHANNEL_CAL_CONSTANTS_H_
#define INC_CHANNEL_CAL_CONSTANTS_H_

#include "stdint.h"

/* ADC0 */
#define IVLV0_ADC_NUM       (0U)
#define EVLV0_ADC_NUM       (0U)
#define IVLV1_ADC_NUM       (0U)
#define EVLV1_ADC_NUM       (0U)
#define IVLV2_ADC_NUM       (0U)
#define EVLV2_ADC_NUM       (0U)
#define IVLV3_ADC_NUM       (0U)
#define EVLV3_ADC_NUM       (0U)
#define IVLV4_ADC_NUM       (0U)
#define EVLV4_ADC_NUM       (0U)
#define IVLV5_ADC_NUM       (0U)
#define EVLV5_ADC_NUM       (0U)
#define IVLV6_ADC_NUM       (0U)
#define EVLV6_ADC_NUM       (0U)
#define IVLV7_ADC_NUM       (0U)
#define EVLV7_ADC_NUM       (0U)

#define IVLV0_ADC_CH        (0U)
#define EVLV0_ADC_CH        (1U)
#define IVLV1_ADC_CH        (2U)
#define EVLV1_ADC_CH        (3U)
#define IVLV2_ADC_CH        (4U)
#define EVLV2_ADC_CH        (5U)
#define IVLV3_ADC_CH        (6U)
#define EVLV3_ADC_CH        (7U)
#define IVLV4_ADC_CH        (8U)
#define EVLV4_ADC_CH        (9U)
#define IVLV5_ADC_CH       (10U)
#define EVLV5_ADC_CH       (11U)
#define IVLV6_ADC_CH       (12U)
#define EVLV6_ADC_CH       (13U)
#define IVLV7_ADC_CH       (14U)
#define EVLV7_ADC_CH       (15U)

/* ADC1 */
#define IVLV8_ADC_NUM       (1U)
#define EVLV8_ADC_NUM       (1U)
#define IVLV9_ADC_NUM       (1U)
#define EVLV9_ADC_NUM       (1U)
#define IVLV10_ADC_NUM      (1U)
#define EVLV10_ADC_NUM      (1U)
#define IVLV11_ADC_NUM      (1U)
#define EVLV11_ADC_NUM      (1U)
#define IVLV12_ADC_NUM      (1U)
#define EVLV12_ADC_NUM      (1U)
#define IVLV13_ADC_NUM      (1U)
#define EVLV13_ADC_NUM      (1U)

#define IVLV8_ADC_CH        (4U) // 0-3 are disconnected
#define EVLV8_ADC_CH        (5U)
#define IVLV9_ADC_CH        (6U)
#define EVLV9_ADC_CH        (7U)
#define IVLV10_ADC_CH       (8U)
#define EVLV10_ADC_CH       (9U)
#define IVLV11_ADC_CH      (10U)
#define EVLV11_ADC_CH      (11U)
#define IVLV12_ADC_CH      (12U)
#define EVLV12_ADC_CH      (13U)
#define IVLV13_ADC_CH      (14U)
#define EVLV13_ADC_CH      (15U)

/* ADC2 */
#define EBATT_ADC_NUM       (2U)
#define IBUS_ADC_NUM        (2U)
#define E5V_ADC_NUM         (2U)
#define E3V3_ADC_NUM        (2U)
#define I3V3_ADC_NUM        (2U)
#define I5V_ADC_NUM         (2U)
#define TBRD0_ADC_NUM       (2U)
#define TBRD1_ADC_NUM       (2U)
#define TBRD2_ADC_NUM       (2U)
#define PT0_ADC_NUM         (2U)
#define PT1_ADC_NUM         (2U)
#define PT2_ADC_NUM         (2U)
#define PT3_ADC_NUM         (2U)
#define PT4_ADC_NUM         (2U)
#define PT5_ADC_NUM         (2U)
#define PT6_ADC_NUM         (2U)

#define EBATT_ADC_CH        (0U)
#define IBUS_ADC_CH         (1U)
#define E5V_ADC_CH          (2U)
#define E3V3_ADC_CH         (3U)
#define I3V3_ADC_CH         (4U)
#define I5V_ADC_CH          (5U)
#define TBRD0_ADC_CH        (6U)
#define TBRD1_ADC_CH        (7U)
#define TBRD2_ADC_CH        (8U)
#define PT0_ADC_CH          (9U)
#define PT1_ADC_CH         (10U)
#define PT2_ADC_CH         (11U)
#define PT3_ADC_CH         (12U)
#define PT4_ADC_CH         (13U)
#define PT5_ADC_CH         (14U)
#define PT6_ADC_CH         (15U)

/* ADC3 */
#define PT7_ADC_NUM         (3U)
#define PT8_ADC_NUM         (3U)
#define PT9_ADC_NUM         (3U)
#define PT10_ADC_NUM        (3U)
#define PT11_ADC_NUM        (3U)
#define PT12_ADC_NUM        (3U)
#define PT13_ADC_NUM        (3U)
#define PT14_ADC_NUM        (3U)
#define PT15_ADC_NUM        (3U)
#define PT16_ADC_NUM        (3U)
#define PT17_ADC_NUM        (3U)
#define PT18_ADC_NUM        (3U)
#define PT19_ADC_NUM        (3U)
#define PT20_ADC_NUM        (3U)
#define POT0_ADC_NUM        (3U)
#define POT0_ADC_NUM        (3U)

#define PT7_ADC_CH          (0U)
#define PT8_ADC_CH          (1U)
#define PT9_ADC_CH          (2U)
#define PT10_ADC_CH         (3U)
#define PT11_ADC_CH         (4U)
#define PT12_ADC_CH         (5U)
#define PT13_ADC_CH         (6U)
#define PT14_ADC_CH         (7U)
#define PT15_ADC_CH         (8U)
#define PT16_ADC_CH         (9U)
#define PT17_ADC_CH        (10U)
#define PT18_ADC_CH        (11U)
#define PT19_ADC_CH        (12U)
#define PT20_ADC_CH        (13U)
#define POT0_ADC_CH        (14U)
#define POT1_ADC_CH        (15U)

float adc_counts_to_volts(uint32_t adc_counts, uint8_t adc_num, uint8_t adc_channel);

#endif /* INC_CHANNEL_CAL_CONSTANTS_H_ */
