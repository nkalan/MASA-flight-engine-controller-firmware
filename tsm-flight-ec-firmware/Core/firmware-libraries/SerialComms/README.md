# User Guide

* [MAX3491 Technical Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX3483-MAX3491.pdf)

## Board to Computer Hardware Setup Details

### MAX3491 RS422 Chip Hardware pinouts

A common mistake is to incorrectly flip the RJ-45 ethernet cable pins and the MAX3491 chip. On the current RS422 to USB converter board that was designed by Sternie (a previous MASA member), the TX- and TX+ pins on the custom board should be connected to the RX+ and RX- pins on the RJ45 connector. The suspected reason for this is because the MAX3491 chip inverts one of the signals for the differential pair, and we need to replicate this inversion by connecting our TX+- to the opposite polarity RX-+ pairs.

## Configuring UART Settings

A higher baud rate can be selected for this library. However, if this is done, please ensure that the clock configuration for UART is at least 16 times faster than the baud rate to ensure proper behavior.

Under the Parameter Settings tab in the .ioc file, use the following configuration

* Mode: Asynchronous
* Hardware Flow Control (RS232): Disable
* Baud Rate: 115200 Bits/s
* Word Length: 8 Bits
* Parity: None
* Stop Bits: 1
* Data Direction: Receive and Transmit
* Over Sampling: 16 times

Under the NVIC Settings tab in the .ioc file, use the following configuration

* USART2 global interrupt: Enabled

## Configuring Hardware Pinouts

Write things here about connecting to the computer and other boards

## Comms Library Setup Struct Explanation

The packet header needs to specified before any commands are transmitted using this library. The packet header will contain essential information, such as the `packet_type`, which will be used to identify how this packet should be handled once it is received. The packet type definitions can be found in the /firmware-libraries/SerialComms/python/packet_telem_template.csv file. 

The `origin_addr` is used for specifying which board the packet originated from. This can be useful for interboard communications. 

In addition, the `target_addr` will be used in the future for allowing you to address specific boards or specify which board transmitted the message. 

The `priority` byte can be used to set different priorities for each telem packet received if multiple are received in parallel. However, all mission critical commands should be notified using an external gpio interrupt instead to guarantee timely handling.

The `do_cobbs` byte is used to indicate whether or not the latter packet is cobbs encoded or not. This is mainly used when encoding the data.

Finally, the `timestamp` variable is used to specify a 32 bit unsigned integer that counts the time in milliseconds that have elapsed since the board thas started up. It is important to keep the counts of this timestamp in milliseconds to ensure consistent behavior across all boards. 

Note: with a 32 bit unsigned integer for timestamps, we have log about 1100 hours, which is more than enough range.

```
// Packet Header 
typedef struct CLB_Packet_Header {
    uint8_t packet_type;        // CMD/DATA packet ID
    uint8_t origin_addr;        // origin board address
    uint8_t target_addr;        // target board address
    uint8_t priority;           // priority of packet
    uint8_t do_cobbs;           // 1 to enable cobbs encoding
    uint16_t checksum;          // checksum to ensure robustness (generated)
    uint32_t timestamp;         // timestamp for data
} CLB_Packet_Header;
```

In addition to the packet header, firmware writers should also define the `CLB_send_data_info` struct as well. This struct is used to specify which uart channel should be used when transmitting packets and also packs the packet into an array of bytes that can be written directly into flash. If the developer is only concerned with transmitting packets over uart, they can simply configure the `uartx` field with a pointer to their uart channel.

The array of bytes is a pointer `flash_arr` and the size of this array is in `flash_arr_used`. The user can also specify the size of the array buffer that the comms library stores the array of bytes into using the `flash_arr_sz` variable. 

```
// Send Data Info
typedef struct CLB_send_data_info {
	UART_HandleTypeDef* uartx;
	int16_t flash_arr_sz;
	int16_t flash_arr_used;
	uint8_t *flash_arr;
} CLB_send_data_info;
```

## Autogeneration Files

In order to use this library properly, there are a few modifications that you will need to make to two configuration csv files. After making these modifications, you will need to call two python scripts and copy over the autogenerated files. I will describe these steps in more detail below.

1. To modify what data is included in the default telemetry packet, you will need to modify the `SerialComms/python/telem_data_template.csv` file. Specifically, you should only modify the `should_generate` column. Writing a *y* in this column indicates that you want the current row to be included in the telemetry packet. Subsequently, writing a *n* implies the opposite. If you feel that a telemetry point is not currently available that you will need, let someone from avionics know and they will make this change for you manually. Finally, the last part you will need to update is row with the name Own Board Addr. You will need to set the min and max value columns in this row to the address of your board. Failure to do this will result in undefined behavior when receiving telemetry. For people developing firmware that will run specific boards, you will have to create a copy of the template csv file with the name `telem_data_youboardnamehere.csv`. Note: the data is sent in byte sized chunks over RS-422, which means that if you select a telemetry point that is x bits, you will need to select ceil(x/8) rows. 

2. To add/enable commands that can be processed by the target board, you will need to modify the `SerialComms/python/telem_cmd_template.csv` file. If you need to add a new command, please update the `packet_type` column for the new function to be the next consecutive number in the list. Note: packet types 0-7 are reserved for critical functions that must be included on all boards. At this point, the only critical function implemented is 0, which sends a full telem packet to the target board. In addition, you must add to the `supported_target_addr` column your board addr. If you are confused as what this, please refer to point 1. 

3. After modifying the config files, navigate to the `SerialComms/python` directory. Then, call `telem_file_generator.py telem_data_template.csv` to generate the corresponding `globals.c`, `globals.h`, `pack_telem_defines.c`, and `pack_telem_defines.h` files.  You will need to copy and paste the autogenerated portion of the `globals.c` and `globals.h` into the `${Project_DIR}/Core/Src` and `${Project_DIR}/Core/Inc` directories respectively. Remember to delete the globals.c and globals.h files after copy and pasting their contents to avoid compilation errors. The other two files do not need to be touched. 

For convenience, I have also included a Makefile in the Python directory, if you run `make build` while in that directory, the Makefile will automatically run the parser script and autogenerate the output files to the correct directory in your current project and also in the GUI project. However, you will need to export a new `GUI_PATH` variable in your PATH. This will point to the path of your GUI project directory, or any arbitrary path on your computer if you do not have the GUI git repo cloned. An explanation for how to do this can be found online. I recommend setting this up if you plan on frequently using the comms library.

4. If you plan using a variable to keep of board state, make sure to select the STATES line in the `telem_data_template.csv` file and copy and paste this defintion into the user section of the autogenerated file `globals.h`. Note: you may need to modify this struct to better suit your board's needs.

5. If your board will need to use various calibrations, you will need to add these calibration to the `calibrations.csv` file. Before doing this, be sure to consult someone from ATLO to ensure that the calibration parameters are correct before doing so.

```
enum States {
    Idle    = 0,
    Armed   = 1,
    Running = 2,
    Stop    = 3
};
```

5. After following this setup, you will be able to use the sample comms library code below for transmitting the default telem packet.

## Sample Comms Library Transmit Code

### Sample code for transmitting default telem packet (COBBS encoded)
```
CLB_Packet_Header header;
header.packet_type = 0; // default packet
header.origin_addr = your_board_addr_here; 
header.target_addr = 7; // computer
header.priority = 1; // medium
header.do_cobbs = 1; // disable cobbs
header.timestamp= HAL_GetTick();
init_data(NULL, -1, &header);   // pack all telem data
CLB_send_data_info info;
info.uartx = your_uart_channel_here;
send_data(&info, CLB_Telem);
```

### Sample code for transmitting custom telem packet
```
CLB_Packet_Header header;
header.packet_type = 0; // default packet
header.target_addr = 0; // computer
header.priority = 1; // medium
header.do_cobbs = 0; // disable cobbs
header.timestamp= HAL_GetTick();
uint8_t buffer[10] = {0};
int16_t buffer_sz = 10;
init_data(buffer, buffer_sz, &header);
CLB_send_data_info info;
info.uartx = your_uart_channel_here;
send_data(&info, CLB_Telem);
```

### Sample code for receiving custom telem packet

Please keep in mind that once a telem buffer is received, it is up to the programmer to declare functions for command handling. Writing functions to handle commands is a process that will be covered in the later sections. If this code seems out of date or has issues, it is most likely because we are in the middle of transitioning towards using dma.

```
// initialize as global variable
uint8_t last_byte_uart;
volatile uint16_t telem_buffer_sz = 0;

// put into arbitrary intialization function
...other code for one time function calls...
HAL_UART_Receive_IT(&huart2, &last_byte_uart, 1);

// place into main.c
// Note: huartx should whichever uart peripheral in which you
//          receive telemetry packets
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huartx) {
        HAL_UART_Receive_IT(&huartx, &last_byte_uart, 1);
        telem_buffer[telem_buffer_sz++] = last_byte_uart;
    }
}

// place into main while loop
// EXPECTED_TELEM_BUFFER_SZ depends on how long the telem packet is
if (telem_buffer_sz == EXPECTED_TELEM_BUFFER_SZ) {
    receive_data(&huartx, telem_buffer, telem_buffer_sz);
    telem_buffer_sz = 0;
}
```

## Handling reception of custom commands

The list of commands currently available to the board can be found in the `pack_cmd_defines.h` file in the firmware-libraries/SerialComms/inc/ directory. These commands and the order in which their arguments are in are defined in the firmware-libraries/SerialComms/python/ directory. In addition, we have developed custom scripts for autogenerating the `pack_cmd_defines.h` file as well as the `telem.c` file if you would like to add more commands. the `telem.c` file contains a list of all available function as well as function arguments that are initialized at the beginning of each function. 

It is the programmers job to actually implement additional functionality for each of these commands. In addition, the `telem.c` should be placed in th ${Project_Directory}/Core/src directory in order allow easy access to global variables. It will be common practice to have to include external global variables from the main.c file in order to adequately handle most commands.
             
## Developer Guide

### Test Procedure

To verify that you can properly send packets, repeat the following test procedures. 

The most basic test is to verify that you can receive a telemetry packet. To do so, simply connect the board to the server by running `python server.py` (or `python3 server.py` if your default python folder is not python), which can be found in the gui repo. Then, just make sure that the packet size received matches the packet size that you have set and that the values look reasonable.

A simply test to check if commands are going through by sending a command to blink an led on your board. This is left an exercise to the developer.
