/**
 ********************************************************************************
 * @file    foc.h
 ********************************************************************************
 */

#ifndef __FOC_H
#define __FOC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "foc_math.h"
#include "peripheral.h"

extern uint16_t CSA[4];  // CSA, CSB, CSC, VBUS
extern float I_ref;

void FOC_Handler();
void FOC_Enable();
void FOC_Disable();

extern float vel;
extern float pos;

#ifdef __cplusplus
}
#endif

#endif