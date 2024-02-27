/*

PID Library for closed-loop control


Effects of increasing parameters independently
==============================================


|-----------|--------------|-----------|---------------|--------------------|--------------------|
| Parameter |  Rise time   | Overshoot | Settling time | Steady-state error |     Stability      |
|-----------|--------------|-----------|---------------|--------------------|--------------------|
|     P     |   Decrease   | Increase  | Small change  |      Decrease      |      Degrade       |
|-----------|--------------|-----------|---------------|--------------------|--------------------|
|     I     |   Decrease   | Increase  | Increase      |      Eliminate     |      Degrade       |
|-----------|--------------|-----------|---------------|--------------------|--------------------|
|     D     | Small change | Decrease  | Decrease      | No effect (theory) | Improve if small D |
|-----------|--------------|-----------|---------------|--------------------|--------------------|

*/

#ifndef __PID_HEADER_H__
#define __PID_HEADER_H__

#include <stdint.h>



typedef float real_t;


enum
{
  DEFAULT_BEHAVIOR           = 0,
  CLAMP_OUTPUT               = 1, // Clamp to the given min/max output values
  RESET_ACC_ON_ZERO_CROSS    = 2, // Clegg integrator, reset i_acc when the error == 0 or changes sign (limit overshoot)
  CLAMP_ACC_TO_OUTPUT_BOUNDS = 4, // Limit i_accumulator to the output min/max bounds
};



struct pid
{
  real_t  i_gain;        // integral gain
  real_t  p_gain;        // proportional gain
  real_t  d_gain;        // derivative gain
  real_t  d_last_input;  // numerical derivative
  real_t  i_accumulator; // numerical integral/accumulator
  real_t  output_min;    // output min/max limits
  real_t  output_max;
  uint8_t flags;         // "feature"-flag
};



void   pid_init (struct pid* pid, real_t p_gain, real_t i_gain, real_t d_gain);
void   pid_set_flags(struct pid* pid, uint8_t flags);
void   pid_set_limits(struct pid* pid, real_t out_min, real_t out_max);
real_t pid_update(struct pid* pid, real_t error, real_t input);




#endif //  __PID_HEADER_H__

