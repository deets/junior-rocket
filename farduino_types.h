#ifndef __FARDUINO_TYPES_H__
#define __FARDUINO_TYPES_H__

typedef struct{
  double mean_a[3];
  double mean_w[3];
  double mean_B[3];
  
  double sigma_a[3];
  double sigma_w[3];
  double sigma_B[3];  
} inertial_measurement_t;




typedef enum {
  state_IDLE,
  state_ACCELERATION,
  state_LAUNCH,
  state_BURNOUT,
  state_SEPARATION,
  state_COASTING,
  state_PEAK_REACHED,
  state_FALLING,
  state_DROGUE_OPENED,
  state_LANDED
} stage_state_t;



#endif
