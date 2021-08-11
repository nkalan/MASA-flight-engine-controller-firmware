/*
 * MAX31855.c
 *
 *  Created on: Jan 4, 2021
 *      Author: arthur
 */
#include "MAX31855.h"

int32_t findClosestTTMV(float target) {
	int32_t right = MAX31855_TTMV_LUT_SZ - 1;
	int32_t left = 0;
	int32_t mid = 0;
    // Find the two closest microvolt points
    while (left < right) {
        mid = ((right-left)/2)+left;
        if (MAX31855_TTMV_LUT[mid] < target) {
            left = mid+1;
        } else {
            right = mid;
        }
    } // TODO: write bs alg to find correct microvolt conversions
    return left;
}

float read_tc(SPI_HandleTypeDef *SPI_BUS, MAX31855_Pinfo *pinfo) {
	uint8_t tx[4] = { 0 };
    uint8_t rx[4] = { 0 };
    // Read thermocouples raw temperature
    __disable_irq();
    HAL_GPIO_WritePin(pinfo->MAX31855_CS_PORT, pinfo->MAX31855_CS_ADDR,
            GPIO_PIN_RESET);
    HAL_SPI_Receive(SPI_BUS, rx, 4, 1);
    HAL_GPIO_WritePin(pinfo->MAX31855_CS_PORT, pinfo->MAX31855_CS_ADDR,
            GPIO_PIN_SET);
    __enable_irq();

    int32_t spiData = rx[0] << 24 | rx[1] << 16 | rx[2] << 8 | rx[3];
    int32_t thermocoupleData;
    int32_t refJuncData;
    int32_t faultFlag;
    float uncorrectedThermocoupleTemp;
    float refJuncTemp;
    float totalOutputMicroVolts;
    float refJuncMicroVolts;
    float thermocoupleMicroVolts;
    float correctedThermocoupleTemp = 0.0f;
    uint8_t ocFaultFlag = rx[3]&0b1;

    faultFlag = (spiData & 0x00010000) >> 16;
    if ((spiData & 0x80000000) == 0x80000000) {
        thermocoupleData = (spiData ^ 0xFFFFFFFF) >> 18;
        thermocoupleData++;
        thermocoupleData = thermocoupleData * -1;
    } else {
        thermocoupleData = spiData >> 18;
    }
    if ((spiData & 0x00008000) == 0x00008000) {
        refJuncData = ((spiData ^ 0xFFFFFFFF) >> 4) & 0x00000FFF;
        refJuncData++;
        refJuncData = refJuncData * -1;
    } else {
        refJuncData = (spiData >> 4) & 0x00000FFF;
    }
    // Calculate out the uncorrected temperatures from the MAX31855 and
    // find the total output voltage in micro volts
    // (using the MAX31855 equation from datasheet)
    uncorrectedThermocoupleTemp = thermocoupleData * 0.25;
    refJuncTemp = refJuncData * 0.0625;
    totalOutputMicroVolts = (COLD_JUNC_SENSITIVITY_COEFF_T)
            * (uncorrectedThermocoupleTemp - refJuncTemp);
    // Find the reference junction voltage by using the lookup table and
    // finding the equivalent voltage in microvolts for a specific temperature.
    // Use linear interpolation to find the most accurate microvolts for the
    // given temperature --> y2 = m(x2-x1) + y1 (we must cast back as a signed
    // int since pgm_read returns unsigned)
    int refJuncMicrovoltsHigh;
    int refJuncMicrovoltsLow;
    int refJuncMicrovoltsSlope;
    refJuncMicrovoltsHigh = (int) (MAX31855_TTMV_LUT[((int) ceil(
            refJuncTemp) + MAX31855_LUT_OFFSET)]);
    refJuncMicrovoltsLow = (int) (MAX31855_TTMV_LUT[((int) floor(
            refJuncTemp) + MAX31855_LUT_OFFSET)]);
    refJuncMicrovoltsSlope = (refJuncMicrovoltsHigh - refJuncMicrovoltsLow);
    refJuncMicroVolts = refJuncMicrovoltsSlope
            * (refJuncTemp - floor(refJuncTemp)) + refJuncMicrovoltsLow;
    // Calculate the voltage of the desired thermocouple junction itself (thermocouple junction and ref junction polarities are opposing in our application
    // with a type T thermocouple --> V_out = V_tc - V_ref)
    thermocoupleMicroVolts = totalOutputMicroVolts + refJuncMicroVolts;
    // Check to make sure this voltage is within our range of -200 to 350C then proceed to lookup table processing, or else return an out or range error
    if (thermocoupleMicroVolts < MAX31855_minVoltage
            || thermocoupleMicroVolts > MAX31855_maxVoltage) {
    } else {
        // Perform a reverse lookup table sorting to find the temperature from
        // microvolts (this code implements a binary search algorithm...
        // The closest two voltage values (less than and greater than) are
        // stored into variables so we can use them to interpolate for 0.01C resolution
        int32_t correctedMicrovoltsHigh;
        int32_t correctedMicrovoltsLow;
        int32_t correctedMicrovoltsSlope;
        int32_t closestIdx;
        // Set the starting points
        closestIdx = findClosestTTMV(thermocoupleMicroVolts);
        correctedMicrovoltsHigh = (int32_t) MAX31855_TTMV_LUT[closestIdx];
        correctedMicrovoltsLow  = (int32_t) MAX31855_TTMV_LUT[closestIdx-1];

        // Find the final corrected temperature from microvolts using
        // linear interpolation - x2 = (y2-y1)/m + x1
        correctedMicrovoltsSlope = correctedMicrovoltsHigh
                - correctedMicrovoltsLow;
        correctedThermocoupleTemp = ((thermocoupleMicroVolts
                - correctedMicrovoltsLow) / correctedMicrovoltsSlope)
                + ((closestIdx - 1) - MAX31855_LUT_OFFSET);
    }

    return correctedThermocoupleTemp + 273.15; // replace with return value
}

const int16_t MAX31855_TTMV_LUT[MAX31855_TTMV_LUT_SZ] = {
            -5603, -5587, -5571,
        -5555, -5539, -5523, -5506, -5489, -5473, -5456, -5439, -5421, -5404,
        -5387, -5369, -5351, -5334, -5316, -5297, -5279, -5261, -5242, -5224,
        -5205, -5186, -5167, -5148, -5128, -5109, -5089, -5070, -5050, -5030,
        -5010, -4989, -4969, -4949, -4928, -4907, -4886, -4865, -4844, -4823,
        -4802, -4780, -4759, -4737, -4715, -4693, -4671, -4648, -4626, -4604,
        -4581, -4558, -4535, -4512, -4489, -4466, -4443, -4419, -4395, -4372,
        -4348, -4324, -4300, -4275, -4251, -4226, -4202, -4177, -4152, -4127,
        -4102, -4077, -4052, -4026, -4000, -3975, -3949, -3923, -3897, -3871,
        -3844, -3818, -3791, -3765, -3738, -3711, -3684, -3657, -3629, -3602,
        -3574, -3547, -3519, -3491, -3463, -3435, -3407, -3379, -3350, -3322,
        -3293, -3264, -3235, -3206, -3177, -3148, -3118, -3089, -3059, -3030,
        -3000, -2970, -2940, -2910, -2879, -2849, -2818, -2788, -2757, -2726,
        -2695, -2664, -2633, -2602, -2571, -2539, -2507, -2476, -2444, -2412,
        -2380, -2348, -2316, -2283, -2251, -2218, -2186, -2153, -2120, -2087,
        -2054, -2021, -1987, -1954, -1920, -1887, -1853, -1819, -1785, -1751,
        -1717, -1683, -1648, -1614, -1579, -1545, -1510, -1475, -1440, -1405,
        -1370, -1335, -1299, -1264, -1228, -1192, -1157, -1121, -1085, -1049,
        -1013, -976, -940, -904, -867, -830, -794, -757, -720, -683, -646,
        -608, -571, -534, -496, -459, -421, -383, -345, -307, -269, -231, -193,
        -154, -116, -77, -39, 0, 39, 78, 117, 156, 195, 234, 273, 312, 352,
        391, 431, 470, 510, 549, 589, 629, 669, 709, 749, 790, 830, 870, 911,
        951, 992, 1033, 1074, 1114, 1155, 1196, 1238, 1279, 1320, 1362, 1403,
        1445, 1486, 1528, 1570, 1612, 1654, 1696, 1738, 1780, 1823, 1865, 1908,
        1950, 1993, 2036, 2079, 2122, 2165, 2208, 2251, 2294, 2338, 2381, 2425,
        2468, 2512, 2556, 2600, 2643, 2687, 2732, 2776, 2820, 2864, 2909, 2953,
        2998, 3043, 3087, 3132, 3177, 3222, 3267, 3312, 3358, 3403, 3448, 3494,
        3539, 3585, 3631, 3677, 3722, 3768, 3814, 3860, 3907, 3953, 3999, 4046,
        4092, 4138, 4185, 4232, 4279, 4325, 4372, 4419, 4466, 4513, 4561, 4608,
        4655, 4702, 4750, 4798, 4845, 4893, 4941, 4988, 5036, 5084, 5132, 5180,
        5228, 5277, 5325, 5373, 5422, 5470, 5519, 5567, 5616, 5665, 5714, 5763,
        5812, 5861, 5910, 5959, 6008, 6057, 6107, 6156, 6206, 6255, 6305, 6355,
        6404, 6454, 6504, 6554, 6604, 6654, 6704, 6754, 6805, 6855, 6905, 6956,
        7006, 7057, 7107, 7158, 7209, 7260, 7310, 7361, 7412, 7463, 7515, 7566,
        7617, 7668, 7720, 7771, 7823, 7874, 7926, 7977, 8029, 8081, 8133, 8185,
        8237, 8289, 8341, 8393, 8445, 8497, 8550, 8602, 8654, 8707, 8759, 8812,
        8865, 8917, 8970, 9023, 9076, 9129, 9182, 9235, 9288, 9341, 9395, 9448,
        9501, 9555, 9608, 9662, 9715, 9769, 9822, 9876, 9930, 9984, 10038,
        10092, 10146, 10200, 10254, 10308, 10362, 10417, 10471, 10525, 10580,
        10634, 10689, 10743, 10798, 10853, 10907, 10962, 11017, 11072, 11127,
        11182, 11237, 11292, 11347, 11403, 11458, 11513, 11569, 11624, 11680,
        11735, 11791, 11846, 11902, 11958, 12013, 12069, 12125, 12181, 12237,
        12293, 12349, 12405, 12461, 12518, 12574, 12630, 12687, 12743, 12799,
        12856, 12912, 12969, 13026, 13082, 13139, 13196, 13253, 13310, 13366,
        13423, 13480, 13537, 13595, 13652, 13709, 13766, 13823, 13881, 13938,
        13995, 14053, 14110, 14168, 14226, 14283, 14341, 14399, 14456, 14514,
        14572, 14630, 14688, 14746, 14804, 14862, 14920, 14978, 15036, 15095,
        15153, 15211, 15270, 15328, 15386, 15445, 15503, 15562, 15621, 15679,
        15738, 15797, 15856, 15914, 15973, 16032, 16091, 16150, 16209, 16268,
        16327, 16387, 16446, 16505, 16564, 16624, 16683, 16742, 16802, 16861,
        16921, 16980, 17040, 17100, 17159, 17219, 17279, 17339, 17399, 17458,
        17518, 17578, 17638, 17698, 17759, 17819 };
