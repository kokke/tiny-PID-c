# tiny-PID-c

## PID Library for closed loop control

Simple, stupid PID library for closed-loop control.

### API

```C
void   pid_init(struct pid* pid, real_t p_gain, real_t i_gain, real_t d_gain);
void   pid_set_flags(struct pid* pid, uint8_t flags);
void   pid_set_limits(struct pid* pid, real_t out_min, real_t out_max);
real_t pid_update(struct pid* pid, real_t error, real_t input);
```

Where the `flag` is one or more of the bitmasks below:

```c
enum
{
  DEFAULT_BEHAVIOR           = 0,
  CLAMP_OUTPUT               = 1, // Clamp to the given min/max output values
  RESET_ACC_ON_ZERO_CROSS    = 2, // Clegg integrator, reset i_acc when the error == 0 or changes sign (limit overshoot)
  CLAMP_ACC_TO_OUTPUT_BOUNDS = 4, // Limit i_accumulator to the output min/max bounds
};
```


### Effects of increasing parameters independently

| Parameter |  Rise time   | Overshoot | Settling time | Steady-state error    |     Stability      |
|-----------|--------------|-----------|---------------|-----------------------|--------------------|
|     P     |   Decrease   | Increase  | Small change  |      Decrease         |      Degrade       |
|     I     |   Decrease   | Increase  | Increase      |      Eliminate        |      Degrade       |
|     D     | Small change | Decrease  | Decrease      | No effect (in theory) | Improve if small D |
