# MAX31856 thermocouple library User Guide (Last Updated: Jul 11 2021)
This repository contains the MASA firmware library for the MAX31856  thermocouple to digital converter.

[MAX31856 Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX31856.pdf)

[Dev board datasheet (includes soldering + setup instructions)](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-max31856-thermocouple-amplifier.pdf)

## What is different about this library
The main difference between this firmware library and others is that previous libraries assumed that each TC chip had its own chip select (CS) pin connected to the microcontroller, but the flight EC will instead have a 4-bit decoder that can select 1 TC chip at a time, and the nosecone recovery board will have 2 TCs each with their own CS pin connected to the micro. This library should support both boards, so chip select and chip release have been abstracted to be function pointers, with the thermocouple index as an input. That way, the flight EC can implement its own GPIO MUX logic, and the nosecone board can implement a simpler GPIO toggle for chip select.

[Old MAX31855 TC library](https://gitlab.eecs.umich.edu/masa/avionics/firmware-libraries/-/tree/feature-MAX31856-dev/MAX31855)
The difference between this library and the MAX31855 library is that this library uses an internal look up table (LUT) for linearization.

## TC Array Struct
```
typedef struct {
  SPI_HandleTypeDef* SPI_bus;
  void (*chip_select)(uint8_t);  // Function pointer to select a single TC given its index
  void (*chip_release)(uint8_t);  // Function pointer to release all TCs in the array
  uint8_t num_tcs;

} MAX31856_TC_Array;
```

## Functions and procedure
The `MAX31856_init_thermocouples` function first reads register CR1 from the chip, then change bits 3:0 of that byte (without changing the others) from the default type K to type T. Then, it transmits the register back to the chip. It does the same thing with the converstion mode, reading CR0, changing bit 7 from 0 to 1, then transmits it back. 

The `MAX31856_read_thermocouple` function reads 3 bytes from register 0C. Then it fixes the sign if necessary, changes to decimal, and converts to Kelvin to return. 

## SPI configuration
* Frame Format: Motorola
* Data Size: 8 bits
* First Bit: MSB First
* Maximum Serial Clock Frequency: 5MHz

* Configuration for STM32CudeIDE:
* Prescaler: 16
* CPOL: Low
* CPHA: 2 Edge (This is really important! According to the Digikey tutorial, CPOL low and CPHA 2 are the most commonly used, but remember to change to this in the IDE. Receive is supposed to sample on the falling edge. See datasheet pg 15-17. )

## Testing
When testing with a nucleo board, we need to implement the `chip_select` and `chip_release` functions. Because the nucleo board acts nomal (like the nosecone recovery board), `chip_select` and `chip_release` will just write the CS pin using GPIO. 


