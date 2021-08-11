# Valve Libraries (LAST UPDATED: July 11, 2021)

This library serves to abstract the implementation of mutating valve states by wrapping around an existing valve mutator.

## Valve Struct Explanation

Valve struct consists channel information. 

```
typedef struct {
	uint8_t num_channels;
	uint8_t valve_channels[VALVELIB_MAX_CHANNELS]; // array containing states of each valve
	void (*set_valve_func)(uint32_t, uint8_t); // existing function used to set valve states

} Valve;
```

In particular, `num_channels` specifies the channels in use, which should be below the maximum. Note that the constant `VALVELIB_MAX_CHANNELS` is defines the max at 5 channels.

The `valve_channels` array initializes an array of length `VALVELIB_MAX_CHANNELS` which assigns a specified valve to its corresponding channel indicated by its index.

Lastly, `set_valve_func` takes initializes a function pointer, i.e. a memory address to a function responsible for setting the valve state. It is important note that each `Valve` instance requires a `set_valve_func` passed in, even if several instances share the same function.


## Valve functions

The two functions contain `power_valve(Valve *valve)` and `depower_valve(Valve *valve)` which both take in a `Valve` struct pointer. Depending on which function called, the `valve_channels` array will be iterated through, pass in individually into the struct's`set_valve_func' to set the appropriate state.

## Testing

The following section records the process of testing this library on a nucleo board. The point is seeing that pins on the nucleo board are being set and reset without actually using any valves or the press board (for example).

There are several things that need to be initialized in the main function. In parenthesis, I have located where they are in the press board firmware.

* `num_channels` would correspond to something like `NUM_TANKS` in the press board firmware (globals.h); it is the number of channels for the valve, such as `vlv_assignments.ControlValves[]`(main.h), that is being powered/depowered. Notice how this part stores the valve numbers, which in this library are stored in `valve_channels[]`.
* `set_valve_func` is a pointer to `setValve(uint32_t vlv_num, uint8_t state)`(hardware.c). In order to do everything in main.c, also copy `setValveHelper(uint32_t vlv_num, GPIO_PinState gpio_state)`.
* `uint32_t valve_states` (the same as in globals.h) stores the state of the valves. We need to check this variable.
* If we are testing with valves 0 and 5, we need to: 
```
#define en_vlv0_Pin GPIO_PIN_8
#define en_vlv0_GPIO_Port GPIOA
#define en_vlv3_Pin GPIO_PIN_10
#define en_vlv3_GPIO_Port GPIOB
```
(main.h).
In the .ioc file of the project, set these pins to GPIO_Output.

After initializing, run something like `power_valve(test_vlv)` in `while(1)`. Debug and see that the `valve_states` has become 000...0001001. Also use a multimeter to see that the pins are set.

