#ifndef __CONTROLLER_BODYRATE_H__
#define __CONTROLLER_BODYRATE_H__

#include "stabilizer_types.h"
#include "filter.h"
#include "math3d.h"
#include "log.h"
#include "param.h"
#include "position_controller.h"
#include "attitude_controller.h"
#include "position_controller_indi.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

// these parameters are used in the filtering of the angular acceleration [Hz]
#define STABILIZATION_BODYRATE_FILT_CUTOFF 8.0f

// the yaw sometimes requires more filtering
#define STABILIZATION_BODYRATE_FILT_CUTOFF_R STABILIZATION_BODYRATE_FILT_CUTOFF

// Control effectiveness coefficients values Volodscoi
#define STABILIZATION_BODYRATE_G1_P 0.0066146f 
#define STABILIZATION_BODYRATE_G1_Q 0.0052125f
#define STABILIZATION_BODYRATE_G1_R 0.001497f
#define STABILIZATION_BODYRATE_G2_R 0.000043475f

// Control effectiveness coefficients values Max Kemmeren, these will become the new ones, not finalized yet
// #define STABILIZATION_BODYRATE_G1_P 0.0032502f
// #define STABILIZATION_BODYRATE_G1_Q 0.0027555f
// #define STABILIZATION_BODYRATE_G1_R 0.00068154f
// #define STABILIZATION_BODYRATE_G2_R 0.00001725f

//Proportional gains inner BODYRATE, attitude error
#define STABILIZATION_BODYRATE_REF_ERR_P 5.0f 
#define STABILIZATION_BODYRATE_REF_ERR_Q 5.0f
#define STABILIZATION_BODYRATE_REF_ERR_R 5.0f

//Derivative gains inner BODYRATE, attitude rate error
#define STABILIZATION_BODYRATE_REF_RATE_P 24.0f 
#define STABILIZATION_BODYRATE_REF_RATE_Q 24.0f
#define STABILIZATION_BODYRATE_REF_RATE_R 24.0f

// Actuator model coefficient
#define STABILIZATION_BODYRATE_ACT_DYN_P 0.03149f
#define STABILIZATION_BODYRATE_ACT_DYN_Q 0.03149f
#define STABILIZATION_BODYRATE_ACT_DYN_R 0.03149f

/**
 * @brief angular rates
 * @details Units: rad/s */
struct FloatRates {
  float p; ///< in rad/s
  float q; ///< in rad/s
  float r; ///< in rad/s
};

struct ReferenceSystem {
  float err_p;
  float err_q;
  float err_r;
  float rate_p;
  float rate_q;
  float rate_r;
};

struct BodyrateVariables {
  float thrust;
  struct FloatRates angular_accel_ref;
  struct FloatRates du;
  struct FloatRates u_in;
  struct FloatRates u_act_dyn;
  float rate_d[3];

  Butterworth2LowPass u[3];
  Butterworth2LowPass rate[3];
  struct FloatRates g1;
  float g2;

  struct ReferenceSystem reference_acceleration;
  struct FloatRates act_dyn;
  float filt_cutoff;
  float filt_cutoff_r;
};

void controllerBodyrateInit(void);
bool controllerBodyrateTest(void);
void controllerBodyrate(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#endif //__CONTROLLER_BODYRATE_H__
