# Valve Libraries (LAST UPDATED: July 11, 2021)

This library serves to abstract the implementation of mutating valve states by wrapping around an existing implementation.

## Valve Struct EXplanation

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

Lastly, `set_valve_func' takes initializes a function pointer, i.e. a memory address to a function responsible for setting the valve state. It is important note that each `Valve` instance requires a `set_valve_func' passed in, even if several instances share the same function.


## Valve functions

The two functions contain `power_valve(Valve *valve)` and `depower_valve(Valve *valve)` which both take in a `Valve` struct pointer. Depending on which function called, the `valve_channels` array will be iterated through, pass in individually into the struct's`set_valve_func' to set the appropriate state.