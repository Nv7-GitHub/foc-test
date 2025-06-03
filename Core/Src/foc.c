#include "foc.h"

// ADC parameters
#define ADC_MAX 4096.0f  // ADC count
#define VREF 3.3f        // Volts

// Current sense setup
#define RESISTANCE 5.0f / 1000.0f  // Ohms
#define GAIN 40                    // V/V

// VBUS voltage divider
#define R1 1000.0f  // Ohms
#define R2 330.0f   // Ohms

const float32_t CSA_SCALE =
    VREF / (RESISTANCE * GAIN * ADC_MAX);  // Amp per ADC tick
const float32_t VBUS_SCALE =
    (VREF / ADC_MAX) * ((R1 + R2) / R2);  // Volt per ADC tick

uint16_t CSA[4];  // CSA, CSB, CSC, VBUS
float32_t I_ref;
bool FOC_EN = true;

void FOC_Handler(HRTIM_HandleTypeDef *hrtim, SPI_HandleTypeDef *hspi) {
  if (!FOC_EN) {
    DUTY_CYCLE(hrtim, HRTIM_TIMERINDEX_TIMER_A, 0.0f);
    DUTY_CYCLE(hrtim, HRTIM_TIMERINDEX_TIMER_B, 0.0f);
    DUTY_CYCLE(hrtim, HRTIM_TIMERINDEX_TIMER_C, 0.0f);
    return;
  }

  // Read sensors
  float32_t theta = mech2elec(MT_READ(hspi));
  float32_t sin_theta = arm_sin_f32(theta);
  float32_t cos_theta = arm_cos_f32(theta);

  // Calculate measured currents
  float32_t di, qi;
  abc_to_dq(CSA[0] * CSA_SCALE, CSA[1] * CSA_SCALE, CSA[2] * CSA_SCALE,
            cos_theta, sin_theta, &di, &qi);

  // Update current controller
  float32_t alpha, beta;
  foc_pi_update(I_ref, di, qi, CSA[3] * VBUS_SCALE, &alpha, &beta, sin_theta,
                cos_theta);

  // SPWM
  float ad, bd, cd;
  svm(alpha, beta, &ad, &bd, &cd);

  // Update duty cycles
  DUTY_CYCLE(hrtim, HRTIM_TIMERINDEX_TIMER_A, ad);
  DUTY_CYCLE(hrtim, HRTIM_TIMERINDEX_TIMER_B, bd);
  DUTY_CYCLE(hrtim, HRTIM_TIMERINDEX_TIMER_C, cd);
}