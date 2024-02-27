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

#include <assert.h>
#include <stdint.h>
#include "pid.h"




void pid_init(struct pid* pid, real_t p_gain, real_t i_gain, real_t d_gain)
{
  pid->p_gain        = p_gain;
  pid->i_gain        = i_gain;
  pid->d_gain        = d_gain;
  pid->d_last_input  = 0.0;
  pid->i_accumulator = 0.0;
  pid->flags         = DEFAULT_BEHAVIOR;
}


void pid_set_flags(struct pid* pid, uint8_t flags)
{
  pid->flags = flags;
}


void pid_set_limits(struct pid* pid, real_t out_min, real_t out_max)
{
  assert(out_min < out_max);
  pid->output_min = out_min;
  pid->output_max = out_max;
}


real_t pid_update(struct pid* pid, real_t error, real_t input)
{
  real_t p_term;
  real_t i_term;
  real_t d_term;

  /* accumulate errors with constraints (min/max) */
  pid->i_accumulator += error;

  /* limit output to saturation values? */
  if (pid->flags & CLAMP_ACC_TO_OUTPUT_BOUNDS)
  {
    if (pid->i_accumulator > pid->output_max)
    {
      pid->i_accumulator = pid->output_max;
    }
    else if (pid->i_accumulator < pid->output_min)
    {
      pid->i_accumulator = pid->output_min;
    }
  }

  /* reset accumulator on zero-crossing? */
  if (pid->flags & RESET_ACC_ON_ZERO_CROSS)
  {
    static real_t last_error;
    if (    (error > 0 && last_error <= 0)
         || (error < 0 && last_error >= 0))
    {
      pid->i_accumulator = 0;
    }
    last_error = error;
  }

  /* calculate P, I and D terms individually */
  p_term = pid->p_gain * error;
  i_term = pid->i_gain * pid->i_accumulator;
  d_term = pid->d_gain * (pid->d_last_input - input);
  pid->d_last_input = input; /* update 'derivative' after use */

  /* calculate and return correction */
  real_t correction = p_term + i_term + d_term;

  /* clamp outputs? */
  if (pid->flags & CLAMP_OUTPUT)
  {
    if (correction > pid->output_max)
    {
      correction = pid->output_max;
    }
    else if (correction < pid->output_min)
    {
      correction = pid->output_min;
    }
  }
  return correction;
}

