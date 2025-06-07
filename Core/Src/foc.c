#include "foc.h"

// ADC parameters
#define ADC_MAX 4096.0f  // ADC count
#define VREF 3.3f        // Volts

// Current sense setup
#define RESISTANCE 5.0f / 1000.0f  // Ohms
#define GAIN 40                    // V/V
#define CSA_STEADY 2029
#define CSB_STEADY 2028
#define CSC_STEADY 2022

// VBUS voltage divider
#define R1 1000.0f  // Ohms
#define R2 330.0f   // Ohms

const float32_t CSA_SCALE =
    VREF / (RESISTANCE * GAIN * ADC_MAX);  // Amp per ADC tick
const float32_t VBUS_SCALE =
    (VREF / ADC_MAX) * ((R1 + R2) / R2);  // Volt per ADC tick

uint16_t CSA[4];  // CSA, CSB, CSC, VBUS
float32_t I_ref = 0.5;
bool FOC_EN = false;

#define SCALE_CSA(CSA, STEADY) (float)(CSA - STEADY) * CSA_SCALE

extern HRTIM_HandleTypeDef hhrtim1;
extern SPI_HandleTypeDef hspi1;
extern DAC_HandleTypeDef hdac1;

void FOC_Enable() { FOC_EN = true; }

void FOC_Disable() {
  DUTY_CYCLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, 0.0f);
  DUTY_CYCLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 0.0f);
  DUTY_CYCLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, 0.0f);
  FOC_EN = false;
}

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

  // Calculate measured currents
  float32_t di, qi;
  abc_to_dq(SCALE_CSA(CSA[0], CSA_STEADY), SCALE_CSA(CSA[1], CSB_STEADY),
            SCALE_CSA(CSA[2], CSC_STEADY), cos_theta, sin_theta, &di, &qi);

  // Update current controller
  float32_t alpha, beta;
  foc_pi_update(I_ref, di, qi, CSA[3] * VBUS_SCALE, &alpha, &beta, sin_theta,
                cos_theta);

  if (alpha > 0.2) {
    alpha = 0.2;
  } else if (alpha < -0.2) {
    alpha = -0.2;
  }
  if (beta > 0.2) {
    beta = 0.2;
  } else if (beta < -0.2) {
    beta = -0.2;
  }

  // SPWM
  float ad, bd, cd;
  svm(alpha, beta, &ad, &bd, &cd);

  // Update duty cycles
  DUTY_CYCLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, ad);
  DUTY_CYCLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, bd);
  DUTY_CYCLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, cd);

  // DEBUG
  HAL_GPIO_WritePin(TIMING_OUT_GPIO_Port, TIMING_OUT_Pin, GPIO_PIN_RESET);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                   (uint32_t)(qi * 2048));
}