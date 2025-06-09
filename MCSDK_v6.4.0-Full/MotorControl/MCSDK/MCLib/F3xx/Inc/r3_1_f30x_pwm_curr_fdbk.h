/**
  ******************************************************************************
  * @file    r3_1_f30X_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          R3_1_f30X_pwm_curr_fdbk component of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup R3_1_F30X_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef R3_1_F30X_PWMCURRFDBK_H
#define R3_1_F30X_PWMCURRFDBK_H


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup R3_1_pwm_curr_fdbk
  * @{
  */

#define GPIO_NoRemap_TIM1 ((uint32_t)(0))
#define SHIFTED_TIMs      ((uint8_t) 1)
#define NO_SHIFTED_TIMs   ((uint8_t) 0)

#define NONE    ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))

/* Exported types ------------------------------------------------------- */

/**
  * @brief  PWM and current feedback component parameters definition for single ADC configurations. Common to every MCU except H7XX.
  */
typedef const struct
{
  /* HW IP involved ------------------------------------------------------------- */
  ADC_TypeDef * ADCx;                       /*!< ADC peripheral to be used. Absent in F0XX and G0XX */
  TIM_TypeDef * TIMx;                       /*!< Timer used for PWM generation */
  COMP_TypeDef * CompOCPASelection;         /*!< Internal comparator used for Phase A protection. Specific to F30X and G4XX */
  COMP_TypeDef * CompOCPBSelection;         /*!< Internal comparator used for Phase B protection. Specific to F30X and G4XX */
  COMP_TypeDef * CompOCPCSelection;         /*!< Internal comparator used for Phase C protection. Specific to F30X and G4XX */
  COMP_TypeDef * CompOVPSelection;          /*!< Internal comparator used for Over Voltage protection. Specific to F30X and G4XX */

  /* Currents sampling parameters ----------------------------------------------- */
  uint16_t Tafter;                          /*!< Sum of dead time plus max value between rise time and noise time expressed in number of TIM clocks */
  uint16_t Tbefore;                         /*!< Time of the entire sampling sequence, expressed in number of TIM clocks */
  uint16_t Tcase2;                          /*!< Sum of dead time, noise time and sampling time divided by 2 ; expressed in number of TIM clocks. Absent in H7XX */
  uint16_t Tcase3;                          /*!< Sum of dead time, rise time and sampling time ; expressed in number of TIM clocks. Absent in H7XX */
  uint32_t ADCConfig[6];                    /*!< Stores ADC sequence for the 6 sectors */
  
  /* DAC settings --------------------------------------------------------------- */
  uint16_t DAC_OCP_Threshold;               /*!< Value of analog reference expressed
                                             *   as 16bit unsigned integer. Specific to F30X, G4XX and L4XX. \n
                                             *   Ex. 0 = 0V ; 65536 = VDD_DAC */
  uint16_t DAC_OVP_Threshold;               /*!< Value of analog reference expressed
                                             *   as 16bit unsigned integer. Specific to F30X, G4XX and L4XX. \n
                                             *   Ex. 0 = 0V ; 65536 = VDD_DAC */

  /* PWM Driving signals initialization ----------------------------------------- */
  uint8_t  RepetitionCounter;               /*!< Number of elapsed PWM periods before Compare Registers are updated again.
                                             *   In particular : @f$ RepetitionCounter\ =\ (2\times PWM\ Periods)\ -\ 1 @f$ */
                                             
  /* Internal COMP settings ----------------------------------------------------- */                                 
  uint8_t       CompOCPAInvInput_MODE;      /*!< COMPx inverting input mode. Specific to F30X, G4XX and H7XX. \n
                                             *   Must be either equal to EXT_MODE or INT_MODE. */                            
  uint8_t       CompOCPBInvInput_MODE;      /*!< COMPx inverting input mode. Specific to F30X, G4XX and H7XX. \n
                                             *   Must be either equal to EXT_MODE or INT_MODE. */                                            
  uint8_t       CompOCPCInvInput_MODE;      /*!< COMPx inverting input mode. Specific to F30X, G4XX and H7XX. \n
                                             *   Must be either equal to EXT_MODE or INT_MODE. */                                        
  uint8_t       CompOVPInvInput_MODE;       /*!< COMPx inverting input mode. Specific to F30X, G4XX and H7XX. \n
                                             *   Must be either equal to EXT_MODE or INT_MODE. */ 
  
  /* Dual MC parameters --------------------------------------------------------- */
  uint8_t  FreqRatio;                       /*!< Used in case of dual MC to
                                             *   synchronize TIM1 and TIM8. \n
                                             *   It has effect only on the second instanced
                                             *   object and must be equal to the
                                             *   ratio between the two PWM frequencies
                                             *   (higher/lower). Supported values are
                                             *   1, 2 or 3. */
  uint8_t  IsHigherFreqTim;                 /*!< When FreqRatio is greater than 1
                                             *   this param is used to indicate if this
                                             *   instance is the one with the highest
                                             *   frequency. \n
                                             *   Allowed value are: HIGHER_FREQ
                                             *   or LOWER_FREQ. */                                           

} R3_1_Params_t;

/**
  * @brief  PWM and current feedback component for single ADC configurations.
  */
typedef struct
{
  PWMC_Handle_t _Super;                     /*!< Base component handler. */
  uint32_t PhaseAOffset;                    /*!< Offset of Phase A current sensing network. */
  uint32_t PhaseBOffset;                    /*!< Offset of Phase B current sensing network. */
  uint32_t PhaseCOffset;                    /*!< Offset of Phase C current sensing network. */                                
  uint16_t Half_PWMPeriod;                  /*!< Half PWM Period in timer clock counts. */
  uint16_t ADC_ExternalTriggerInjected;     /*!< External ADC trigger source. Absent in G4XX. */
  uint16_t ADCTriggerEdge;                  /*!< Polarity of the ADC triggering, can be either on rising or falling edge. */
  volatile uint8_t PolarizationCounter;     /*!< Number of conversions performed during the calibration phase. */
  uint8_t PolarizationSector;               /*!< Sector selected during calibration phase. */
  
  R3_1_Params_t * pParams_str;
} PWMC_R3_1_Handle_t;


/* Exported functions ------------------------------------------------------- */

/*
  * Initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  * in three shunt topology using STM32F30X and ADC.
  */
void R3_1_Init( PWMC_R3_1_Handle_t * pHandle );

/*
  * Stores into the handle the voltage present on Ia and
  * Ib current feedback analog channels when no current is flowing into the
  * motor.
  */
void R3_1_CurrentReadingPolarization( PWMC_Handle_t * pHdl );

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format.
  */
void R3_1_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * Iab );

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format. Specific to overmodulation.
  */
void R3_1_GetPhaseCurrents_OVM( PWMC_Handle_t * pHdl, ab_t * Iab );

/*
  * Turns on low sides switches.
  */
void R3_1_TurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks );

/*
  * Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R3_1_SwitchOnPWM( PWMC_Handle_t * pHdl );

/*
  * Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R3_1_SwitchOffPWM( PWMC_Handle_t * pHdl );

/*
  * Configures the ADC for the currents sampling related to sector X (X = [1..6] ).
  */
uint16_t R3_1_SetADCSampPointSectX( PWMC_Handle_t * pHdl );

/*
  * Configure the ADC for the currents sampling related to sector X (X = [1..6] ) in case of overmodulation.
  */
uint16_t R3_1_SetADCSampPointSectX_OVM( PWMC_Handle_t * pHdl );

/*
  * Contains the TIMx Update event interrupt.
  */
void * R3_1_TIMx_UP_IRQHandler( PWMC_R3_1_Handle_t * pHdl );

/*
  * Sets the PWM mode for R/L detection.
  */
void R3_1_RLDetectionModeEnable( PWMC_Handle_t * pHdl );

/*
  * Disables the PWM mode for R/L detection.
  */
void R3_1_RLDetectionModeDisable( PWMC_Handle_t * pHdl );

/*
  * Sets the PWM dutycycle for R/L detection.
  */
uint16_t R3_1_RLDetectionModeSetDuty( PWMC_Handle_t * pHdl, uint16_t hDuty );

/*
  * Turns on low sides switches and start ADC triggering.
  */
void R3_1_RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHdl );

/*
  * Sets the calibrated offsets.
  */
void R3_1_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
  * Reads the calibrated offsets.
  */
void R3_1_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*R3_1_F30X_PWMNCURRFDBK_H*/

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
