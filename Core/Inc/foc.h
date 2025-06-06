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

#include "foc_math.h"
#include "peripheral.h"

extern uint16_t CSA[4];  // CSA, CSB, CSC, VBUS
extern bool FOC_EN;
extern float I_Ref;

void FOC_Handler();

#ifdef __cplusplus
}
#endif

#endif