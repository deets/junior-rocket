#ifndef __FARDUINO_CONSTANTS_H__
#define __FARDUINO_CONSTANTS_H__

//all times in microseconds
#define MIN_ACCELERATION_TIME 400000

#ifdef farduino_maple_v1
#define ONE_G 2048.0
#define ONE_DEG_PER_SECOND 16.4
#else
#define ONE_G 1000.0
#define ONE_DEG_PER_SECOND 16.0	
#endif

#define ONE_SECOND 1000000
#define MIN_FLIGHT_TIME 5000000
#define MAX_TIME_TO_PEAK 7000000
#define MOTOR_BURNOUT_TIME 2100000
#define PEAK_DISCRIMINATION_TIME 800000
#define MAX_SAMPLE_COUNT 200000
#define STAGE_DELAY 1000000
#define MIN_PRESSURE_DROP -0.8

#ifdef farduino_maple_v1
#define PYRO0 PA14
#define PYRO1 PA13
#define PYRO2 PA12
#define PYRO3 PA11
#else
#define PYRO0 PB5
#define PYRO1 PB4
#define PYRO2 PB3
#define PYRO3 PA15
#endif

#endif
