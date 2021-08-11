/** MAX31856.c
 * This is MASA's library for the MAX31856 thermocouple library
 * Datasheet: https://datasheets.maximintegrated.com/en/ds/MAX31856.pdf
 *
 * Samantha Liu (samzliu@umich.edu)
 * Michigan Aeronautical Science Association
 * Created June 19, 2021
 * Last edited: July 8 2021
 */

#include "MAX31856.h"


// Define SPI commands, status bits, registers, here
// Make sure to include the part number at the front of all #defines
#define MAX31856_CFG_REG_1_TC_TYPE_MASK (0x07)

// Datasheet pg 24-25, concatenate the 3 bytes and convert to real number
#define MAX31856_LNRZD_TC_TEMP_B2 (0x0C) //High
#define MAX31856_LNRZD_TC_TEMP_B1 (0x0D)
#define MAX31856_LNRZD_TC_TEMP_B0 (0x0E)

// Add any additional defines here as required
// Datasheet pg 20, stuff related to TC TYPE[3:0]
#define MAX31856_CR1_REG_Read (0x01)
#define MAX31856_CR1_REG_Write (0x81)
#define MAX31856_TCTYPE_T ((uint8_t) 0b0111)

//Datasheet pg 19, Configuration 0 Register for changing conversion mode
#define MAX31856_CR0_REG_Read (0x00)
#define MAX31856_CR0_REG_Write (0x80)

#define MAX31856_TIMEOUT (0x01)

// Add any private helper functions here as required
void MAX31856_Read_Register(MAX31856_TC_Array* tcs, int tc_index, uint8_t reg_addr, uint8_t * rx, int n) {
	/*
	 * Reads a register using SPI
	 * Parameters:
	 * tcs: array of thermocouples struct, each containing a SPI bus, chip_select, chip_release, and num_tcs
	 * tc_index: index of the tc to read. This matters for chip_select and chip_release
	 * reg_addr: which register to read
	 * rx: buffer arry for receiving
	 * n: how many bytes to receive
	 */
	uint8_t reg_buffer[1];
	reg_buffer[0] = reg_addr;
    __disable_irq();
    (*tcs->chip_select)(tc_index);
    HAL_SPI_Transmit(tcs->SPI_bus, (uint8_t *)reg_buffer, 1, MAX31856_TIMEOUT);
    HAL_SPI_Receive(tcs->SPI_bus, (uint8_t *)rx, n, MAX31856_TIMEOUT); //interrupt?
    (*tcs->chip_release)(tc_index);
    __enable_irq();
}

void MAX31856_Write_Register(MAX31856_TC_Array* tcs, int tc_index, uint8_t *tx, int n) {
	/*
	 * Writes a register using SPI
	 * Parameters:
	 * tcs: array of thermocouples struct, each containing a SPI bus, chip_select, chip_release, and num_tcs
	 * tc_index: index of the tc to read. This matters for chip_select and chip_release
	 * tx: buffer arry that gets transmitted.
	 *     Note that the tx must be set before it is passed into this function.
	 *     tx[0] is the register address to transmit into, followed by n bytes of data
	 * n: how many bytes to write. Note that HAL_SPI_Transmit uses n + 1.
	 */
    __disable_irq();
    (*tcs->chip_select)(tc_index);
    HAL_SPI_Transmit(tcs->SPI_bus, (uint8_t *)tx, n + 1, MAX31856_TIMEOUT);
    (*tcs->chip_release)(tc_index);
    __enable_irq();
}


void MAX31856_init_thermocouples(MAX31856_TC_Array* tcs) {
	// MAX31856 is K type default on power up
	// Iterate through all TCs in the array and switch them to T type

	// Make sure each thermocouple automatically converts data.
	// This might be the default, but check the datasheet.
	// We want to be able to read it by only using chip select and MISO,
	// and shouldn't need to transmit commands.

	uint8_t rx[1];
	uint8_t tx[2];
	uint8_t check[2];
  
  for (uint8_t i = 0; i < tcs->num_tcs; i++) {
    // Read the type (default should be K, 00000011) in CR1_REG
	rx[0] = 0;
    MAX31856_Read_Register(tcs, i, MAX31856_CR1_REG_Read, rx, 1);

    // Switch to type T
    rx[0] &= 0xF0; // mask off bottom 4 bits, clear bits 3:0
    rx[0] |= MAX31856_TCTYPE_T;
    
    // Write the register
    tx[0] = MAX31856_CR1_REG_Write;
    tx[1] = rx[0];
    MAX31856_Write_Register(tcs, i, tx, 1);

//    // Checking the register
//    check[0] = 0;
//    check[1] = 0;
//    MAX31856_Read_Register(tcs, i, MAX31856_CR0_REG_Read, check, 2);

    // Change to automatic conversion every 100 ms (datasheet p19)
    rx[0] = 0;

    // Changing to Automatic Conversion mode
    MAX31856_Read_Register(tcs, i, MAX31856_CR0_REG_Read, rx, 1);

    // Set bit 7 to high (default should be 0, Normally Off, and all 0's for this register)
    rx[0] |= 0b10000000;

    // Write the register CR0
    tx[0] = MAX31856_CR0_REG_Write;
    tx[1] = rx[0];
    MAX31856_Write_Register(tcs, i, tx, 1);

    // Checking the register
    check[0] = 0;
    check[1] = 0;
    MAX31856_Read_Register(tcs, i, MAX31856_CR0_REG_Read, check, 2);
  }

  return;
}

float MAX31856_read_thermocouple(MAX31856_TC_Array* tcs, uint8_t tc_index) {
	// Read the temperature value from a single thermocouple chip, specified by tc_index.
	// Then convert that number into a real temp in Celcius, then convert that to Kelvin and return it.

  uint8_t rx[3] = { 0, 0, 0 };
  uint32_t temp32;
  float real_temp_c;
  
  // Receive into rx
  MAX31856_Read_Register(tcs, tc_index, MAX31856_LNRZD_TC_TEMP_B2, rx, 3);
  
  // Convert rx into real_temp
  temp32 = 0b00000000| rx[0] << 16 | rx[1] << 8 | rx[2];
  if (temp32 & 0x800000) { //if first bit is 1, temp is negative
      temp32 |= 0xFF000000; // fill the first 8 bits that will be 0 later
      temp32 = ~temp32; // 2's complement
      temp32 >>= 5; //last 5 bits are not used
      temp32 += 1;
	  real_temp_c = temp32 / 128.0 * -1;
	  // 2 decimal places
	  real_temp_c = (int)(real_temp_c * 100 - 0.5);
	  real_temp_c /= 100;
  }
  else {
	  temp32 >>= 5;
	  real_temp_c = temp32 / 128.0;
	  // 2 decimal places
	  real_temp_c = (int)(real_temp_c * 100 + 0.5);
	  real_temp_c /= 100;
  }
  
  // Convert from Celsius to Kelvin and return
  return real_temp_c + 273.15;
}
