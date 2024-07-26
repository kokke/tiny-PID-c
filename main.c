/*

PID Control demo - simulating a control problem 
===============================================

The task here is to "drive a car" in the middle of the road.

We randomly "push" the car to one of the sides, simulating e.g. a gust of wind.

The control task then becomes to steer us towards the middle.


In this "control task", there aren't any real limits.
We could easily just write P=1.0 and be done with it.
So to make this a bit more fun, we introduce a restriction.
You cannot regulate the position by more than 10 pixels.

The current parameter values are suboptimal on purpose,
to demonstrate oscillation and overshoot.


*/


#include <assert.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "pid.h"


#define SCREEN_WIDTH     101
#define ROAD_WIDTH        80
#define ROAD_BEGIN        11
#define ROAD_MIDDLE       (ROAD_BEGIN + (ROAD_WIDTH/2))
#define ROAD_END          (ROAD_BEGIN + ROAD_WIDTH)
#define DESIRED_FPS       10
#define MAX_CORRECTION   7.5


static void draw_road(int iposition, struct pid* p)
{
  char buf[SCREEN_WIDTH + 2];

  int i;
  for (i = 0; i < SCREEN_WIDTH; ++i)
  {
         if (i == iposition)   buf[i] = 'X';
    else if (i == ROAD_BEGIN)  buf[i] = '|';
    else if (i == ROAD_END)    buf[i] = '|';
    else if (i == ROAD_MIDDLE) buf[i] = '|';
    else                       buf[i] = ' ';
  }
  buf[i++] = ' ';
  buf[i] = 0;

  printf(buf);
  printf("pos = %d, acc=%.02f\n", iposition, p->i_accumulator);
}


static real_t move_randomly(real_t p)
{
  unsigned int r = rand() & 0xFF;
  if (r > 0xF0)
  {
    int max = SCREEN_WIDTH / 2;
    if (r & 1)
      p += rand() % max;
    else
      p -= rand() % max;
  }
  else
  {
    int max = 5;
    if (r & 1)
      p += rand() % max;
    else
      p -= rand() % max;
  }
  return p;
}


int main(void)
{
  struct pid p;

  // setup the PID controller
  pid_init(&p, 0.85, 0.05, 0.02);
  // saturate correction at [-15:+15]
  pid_set_limits(&p, -MAX_CORRECTION, MAX_CORRECTION);
  // enable windup-protection, clump total output gain
  pid_set_flags(&p, RESET_ACC_ON_ZERO_CROSS | CLAMP_OUTPUT);

  // setpoint is the goal we steer towards
  real_t setpoint = (real_t)ROAD_MIDDLE;
  real_t position = setpoint;

  while (1)
  {
    // limit position to visible area
         if (position < (real_t)0)            position = (real_t)0;
    else if (position > (real_t)SCREEN_WIDTH) position = (real_t)SCREEN_WIDTH;

    // randomly push the position around
    position = move_randomly(position);
    // draw the road and our "car"
    draw_road((int)(position + 0.5), &p);
    // 20 FPS
    usleep(1000000 / DESIRED_FPS);

    // how far are we from the target/setpoint?
    real_t diff = position - setpoint;
    // calculate a correction
    real_t corr = pid_update(&p, diff, position);
    // apply correction
    position -= corr;
  }
  return 0;
}
