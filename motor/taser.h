/*
 * 2011-09-07 Sebastian Rockel
 */
#ifndef _TASER_H
#define _TASER_H

// Default max speeds
#define MOTOR_DEF_MAX_SPEED 0.5
#define MOTOR_DEF_MAX_TURNSPEED DTOR(100)

typedef struct player_taser_data
{
  player_position2d_data_t position;
} __attribute__ ((packed)) player_taser_data_t;

#endif
