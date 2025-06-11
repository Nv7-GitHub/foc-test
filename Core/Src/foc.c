#include "foc.h"

// ADC parameters
#define ADC_MAX 4096.0f  // ADC count
#define VREF 3.3f        // Volts

// Current sense setup
#define RESISTANCE 5.0f / 1000.0f  // Ohms
#define GAIN 40                    // V/V
#define CSA_STEADY 2029.0f
#define CSB_STEADY 2028.0f
#define CSC_STEADY 2022.0f

// VBUS voltage divider
#define R1 1000.0f  // Ohms
#define R2 330.0f   // Ohms

// Sensing output
float vel = 0;
float pos = 0;
float32_t prevTheta = 0;

const float32_t CSA_SCALE =
    VREF / (RESISTANCE * GAIN * ADC_MAX);  // Amp per ADC tick
const float32_t VBUS_SCALE =
    (VREF / ADC_MAX) * ((R1 + R2) / R2);  // Volt per ADC tick

uint16_t CSA[4];  // CSA, CSB, CSC, VBUS
uint16_t CSA_prev[3];
float32_t I_ref = 0.0f;
bool FOC_EN = false;

// Low-pass filtered values for CSA/CSB/CSC
float32_t csa_f, csb_f, csc_f;

#define SCALE_CSA(CSA, STEADY) ((float)(CSA) - STEADY) * CSA_SCALE

extern HRTIM_HandleTypeDef hhrtim1;
extern SPI_HandleTypeDef hspi1;
extern DAC_HandleTypeDef hdac1;

void FOC_Enable() {
  FOC_EN = true;
  for (int i = 0; i < 5; i++) {
    prevTheta = MT_READ(&hspi1);
  }
  vel = 0;
  pos = 0;
  I_ref = 0.0f;
  csa_f = 0.0f;
  csb_f = 0.0f;
  csc_f = 0.0f;
}

void FOC_Disable() {
  DUTY_CYCLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, 0.0f);
  DUTY_CYCLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 0.0f);
  DUTY_CYCLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, 0.0f);
  FOC_EN = false;
}

extern float32_t q_i;

void FOC_Handler() {
  if (!FOC_EN) {
    return;
  }

  // DEBUG
  HAL_GPIO_WritePin(TIMING_OUT_GPIO_Port, TIMING_OUT_Pin, GPIO_PIN_SET);

  // Read sensors
  float theta_raw = MT_READ(&hspi1);
  float32_t theta = mech2elec(theta_raw);
  float32_t sin_theta = arm_sin_f32(theta);
  float32_t cos_theta = arm_cos_f32(theta);

  // Filter current sense
  CSA_prev[0] = CSA[0];
  CSA_prev[1] = CSA[1];
  CSA_prev[2] = CSA[2];
  if (abs(CSA[0] - CSA_prev[0]) < 1000) {
    csa_f =
        csa_f * csa_alpha + (1.0f - csa_alpha) * SCALE_CSA(CSA[0], CSA_STEADY);
  }
  if (abs(CSA[1] - CSA_prev[1]) < 1000) {
    csb_f =
        csb_f * csa_alpha + (1.0f - csa_alpha) * SCALE_CSA(CSA[1], CSB_STEADY);
  }
  if (abs(CSA[2] - CSA_prev[2]) < 1000) {
    csc_f =
        csc_f * csa_alpha + (1.0f - csa_alpha) * SCALE_CSA(CSA[2], CSC_STEADY);
  }

  // Calculate measured currents
  float32_t di, qi;
  abc_to_dq(csa_f, csb_f, csc_f, cos_theta, sin_theta, &di, &qi);

  // Update current controller
  float32_t alpha, beta;
  foc_pi_update(I_ref, di, qi, CSA[3] * VBUS_SCALE, &alpha, &beta, sin_theta,
                cos_theta, vel);

  // SPWM
  float ad, bd, cd;
  svm(alpha, beta, &ad, &bd, &cd);

  // Update duty cycles
  DUTY_CYCLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, ad);
  DUTY_CYCLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, bd);
  DUTY_CYCLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, cd);

  // Update vel/pos
  float32_t delta = theta_raw - prevTheta;
  prevTheta = theta_raw;
  vel = vel * 0.9f + calc_vel_delta(delta, &delta) * 0.1f;
  pos += delta;

  // DEBUG
  HAL_GPIO_WritePin(TIMING_OUT_GPIO_Port, TIMING_OUT_Pin, GPIO_PIN_RESET);
  /*HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                   (uint32_t)(I_ref * 512 + 2048));*/
  /*HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                   (uint32_t)(vel * 2.43 + 2048));*/
  /*HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                   (uint32_t)(qi * 409.5 + 2048.0f));*/
  /*HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                   (uint32_t)(csc_f * 409.5 + 2048.0f));*/
  /*HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                   (uint32_t)(q_i * 171 + 2048.0f));*/
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                   (uint32_t)(ad * 2048.0f));
}