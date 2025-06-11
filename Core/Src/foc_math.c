#include "foc_math.h"

#define POLE_PAIRS 7  // 2pi/(Angle A - Angle B), 7 for A2212
#define ANGLE_A_RAD 0.97
// If angle of B is under angle of A, then the direction of the sensor is
// flipped so set this to -1
#define DIRECTION -1

float32_t mech2elec(float32_t angle) {
  float32_t theta = (angle - ANGLE_A_RAD) * POLE_PAIRS * DIRECTION;
  return fmodf(theta, 2.0f * M_PI);
}

// Clarke/park transform
// (https://www.mathworks.com/help/mcb/ref/clarketransform.html,
// https://www.mathworks.com/help/sps/ref/parktransform.html)
// Theta is electrical angle
void abc_to_dq(float32_t a, float32_t b, float32_t c, float32_t cos_theta,
               float32_t sin_theta, float32_t* d, float32_t* q) {
  // Clarke Transform (3-phase)
  float32_t alpha = M_2_3 * (a - 0.5f * b - 0.5f * c);
  float32_t beta = M_2_3 * (M_SQRT3_2 * b - M_SQRT3_2 * c);

  // Park transform
  arm_park_f32(alpha, beta, d, q, sin_theta, cos_theta);
}

// PI controller parameters
#define BANDWIDTH 80 * 2 * M_PI   // Bandwidth in Hz * 2pi
#define INDUCTANCE 1.42e-5        // Henries
#define RESISTANCE 0.10037280035  // Ohms
#define KV 1000                   // 1000kv motor
#define MAX_DUTY 0.8f             // Duty cycle out of 1
#define CUTOFF_FREQ 2000          // For CSA low-pass filter

// Calculated values
const float32_t DT = 1.0f / 10000.0f;  // 1/looprate in hz
                                       // TODO: Get actual value
const float32_t kP = BANDWIDTH * INDUCTANCE;
const float32_t kI = (RESISTANCE / INDUCTANCE) * BANDWIDTH * INDUCTANCE;
const float32_t FF_emf = 60.0f / (2 * M_PI * KV);
// const float32_t csa_alpha = DT / (DT + (1.0f / (2.0f * M_PI * CUTOFF_FREQ)));
const float32_t csa_alpha = 0.0f;

float32_t d_i = 0;
float32_t q_i = 0;

void foc_reset() {
  d_i = 0;
  q_i = 0;
}

// NOTE: ang_vel is mechanical ang vel
void foc_pi_update(float32_t ref_i, float32_t d, float32_t q, float32_t vbus,
                   float32_t* alpha, float32_t* beta, float32_t sin_theta,
                   float32_t cos_theta, float32_t ang_vel) {
  // PI controller
  float32_t d_err = -d;
  float32_t q_err = ref_i - q;

  // Calculate control outputs (+backemf feedforward)
  float32_t vd = kP * d_err + d_i;
  float32_t vq = kP * q_err + q_i;  // + ang_vel * FF_emf;

  // Convert from voltage to duty cycle
  float32_t V2d = 1.5f / vbus;
  float32_t dd = V2d * vd;
  float32_t dq = V2d * vq;

  // Integrator + antiwindup
  // Max duty cycle for SVM is sqrt(3)/2, if duty cycle is >80% of this d_scale
  // will scale down the value to fit into this
  float32_t d_mag;
  arm_sqrt_f32(dd * dd + dq * dq, &d_mag);
  float32_t d_scale = 0.8 * M_SQRT3_2 / d_mag;

  if (d_scale < 1.0f) {  // Need to scale down
    dd *= d_scale;
    dq *= d_scale;
    d_i *= 0.99f;
    q_i *= 0.99f;
  } else {  // Everythings going fine
    d_i += kI * d_err * DT;
    q_i += kI * q_err * DT;
  }

  // Inverse park for svm
  arm_inv_park_f32(dd, dq, alpha, beta, sin_theta, cos_theta);
}

float32_t calc_vel_delta(float32_t delta, float32_t* del) {
  delta *= (float)DIRECTION;
  if (delta > M_PI) {
    delta -= 2 * M_PI;
  } else if (delta < -M_PI) {
    delta += 2 * M_PI;
  }
  *del = delta;
  return delta / DT;
}

void svm(float32_t alpha, float32_t beta, float32_t* d_a, float32_t* d_b,
         float32_t* d_c) {
  float32_t tA, tB, tC;
  int32_t sextant;

  if (beta >= 0.f) {
    if (alpha >= 0.f)
      sextant = (M_1_SQRT3 * beta > alpha) ? 2 : 1;
    else
      sextant = (-M_1_SQRT3 * beta > alpha) ? 3 : 2;
  } else {
    if (alpha >= 0.f)
      sextant = (-M_1_SQRT3 * beta > alpha) ? 5 : 6;
    else {
      sextant = (M_1_SQRT3 * beta > alpha) ? 4 : 5;
    }
  }

  switch (sextant) {
    case 1: {
      float32_t t1 = alpha - M_1_SQRT3 * beta;
      float32_t t2 = M_1_SQRT3 * 2.0f * beta;
      tA = (1.f - t1 - t2) * .5f;
      tB = tA + t1;
      tC = tB + t2;
    } break;
    case 2: {
      float32_t t2 = alpha + M_1_SQRT3 * beta;
      float32_t t3 = -alpha + M_1_SQRT3 * beta;
      tB = (1.f - t2 - t3) * .5f;
      tA = tB + t3;
      tC = tA + t2;
    } break;
    case 3: {
      float32_t t3 = M_1_SQRT3 * 2.0f * beta;
      float32_t t4 = -alpha - M_1_SQRT3 * beta;
      tB = (1.f - t3 - t4) * .5f;
      tC = tB + t3;
      tA = tC + t4;
    } break;
    case 4: {
      float32_t t4 = -alpha + M_1_SQRT3 * beta;
      float32_t t5 = -M_1_SQRT3 * 2.0f * beta;
      tC = (1.0f - t4 - t5) * .5f;
      tB = tC + t5;
      tA = tB + t4;
    } break;
    case 5: {
      float32_t t5 = -alpha - M_1_SQRT3 * beta;
      float32_t t6 = alpha - M_1_SQRT3 * beta;
      tC = (1.f - t5 - t6) * .5f;
      tA = tC + t5;
      tB = tA + t6;
    } break;
    case 6: {
      float32_t t6 = -M_1_SQRT3 * 2.0f * beta;
      float32_t t1 = alpha + M_1_SQRT3 * beta;
      tA = (1.f - t6 - t1) * .5f;
      tC = tA + t1;
      tB = tC + t6;
    } break;
  }

  // Assign the results to the output pointers
  *d_a = 1.f - tA;
  *d_b = 1.f - tB;
  *d_c = 1.f - tC;
}