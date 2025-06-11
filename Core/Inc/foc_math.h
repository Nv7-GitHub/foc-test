/**
 ********************************************************************************
 * @file    foc.h
 ********************************************************************************
 */

#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

#include "arm_math.h"

#define M_1_SQRT3 0.57735026919f  // 1/sqrt(3)
#define M_2_3 0.66666666667f      // 2/3
#define M_SQRT3_2 0.86602540378f  // sqrt(3)/2

float32_t mech2elec(float32_t angle);
void abc_to_dq(float32_t a, float32_t b, float32_t c, float32_t cos_theta,
               float32_t sin_theta, float32_t* d, float32_t* q);
void foc_reset();
void foc_pi_update(float32_t ref_i, float32_t d, float32_t q, float32_t vbus,
                   float32_t* alpha, float32_t* beta, float32_t sin_theta,
                   float32_t cos_theta, float32_t ang_vel);
void svm(float32_t alpha, float32_t beta, float32_t* d_a, float32_t* d_b,
         float32_t* d_c);
float32_t calc_vel_delta(float32_t delta, float32_t* del);

extern const float32_t csa_alpha;

#ifdef __cplusplus
}
#endif

#endif