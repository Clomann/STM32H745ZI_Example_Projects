/**
  ******************************************************************************
  * @file    ADC/ADC_OverSampler/Src/main.c
  * @author  MCD Application Team
  * @brief   This example describes how to set ADC oversampling parameters.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_pwr.h"

/** @addtogroup STM32H7xx_HAL_Examples
  * @{
  */

/** @addtogroup ADC_OverSampler
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ADC handle declaration */


/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef   sConfig;

/* Converted value declaration */
uint32_t                 uwConvertedValue;
/* Input voltage declaration */
uint32_t                 uwInputVoltage;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	/* This sample code shows how to use STM32H7xx ADC HAL API to configure the
	ADC to convert an analog input using the oversampling feature to increase
	resolution (up to 18 bits).
	To proceed, 4 steps are required. */

	/* Enable the CPU Cache */
	  CPU_CACHE_Enable();

	/* STM32H7xx HAL library initialization:
	- Systick timer is configured by default as source of time base, but user
	 can eventually implement his proper time base source (a general purpose
	 timer for example or other time source), keeping in mind that Time base
	 duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
	 handled in milliseconds basis.
	- Set NVIC Group Priority to 4
	- Low Level Initialization
	*/
	/* Set Interrupt Group Priority */
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* Init the low level hardware */

	/* Configure the system clock to 400 MHz */
	SystemClock_Config();

	/* ### - 1 - Initialize ADC peripheral #################################### */
	HAL_StatusTypeDef tmp_hal_status;

	/* Initialize ADC peripheral according to the passed parameters */

	__IO uint32_t wait_loop_index = 0UL;

	/* Init the low level hardware */
	GPIO_InitTypeDef GPIO_InitStruct;

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* ADC Periph clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);

	/* ADC Periph interface clock configuration */
	LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_CLKP);

	/* Enable GPIO clock ****************************************/
	LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);

	/*##-2- Configure peripheral GPIO ##########################################*/
	/* ADC Channel GPIO pin configuration */
	LL_GPIO_SetPinMode(ADCx_CHANNEL_GPIO_PORT, ADCx_CHANNEL_PIN, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinPull(ADCx_CHANNEL_GPIO_PORT, ADCx_CHANNEL_PIN, LL_GPIO_PULL_NO);

	/* - Exit from deep-power-down mode and ADC voltage regulator enable        */
	/* Disable ADC deep power down mode */
	LL_ADC_DisableDeepPowerDown(ADCx);

	/* System was in deep power down mode, calibration must
	be relaunched or a previously saved calibration factor
	re-applied once the ADC voltage regulator is enabled */


	/* Enable ADC internal voltage regulator */
	LL_ADC_EnableInternalRegulator(ADCx);
	while(!LL_ADC_IsInternalRegulatorEnabled(ADCx))
	{
	}

	/* Configuration of common ADC parameters                                 */

	/* Parameters update conditioned to ADC state:                            */
	/* Parameters that can be updated only when ADC is disabled:              */
	/*  - clock configuration                                                 */

	/* Reset configuration of ADC common register CCR:                      */
	/*                                                                      */
	/*   - ADC clock mode and ACC prescaler (CKMODE and PRESC bits)are set  */
	/*     according to adc->Init.ClockPrescaler. It selects the clock      */
	/*    source and sets the clock division factor.                        */
	/*                                                                      */
	/* Some parameters of this register are not reset, since they are set   */
	/* by other functions and must be kept in case of usage of this         */
	/* function on the fly (update of a parameter of ADC_InitTypeDef        */
	/* without needing to reconfigure all other ADC groups/channels         */
	/* parameters):                                                         */
	/*   - when multimode feature is available, multimode-related           */
	/*     parameters: MDMA, DMACFG, DELAY, DUAL (set by API                */
	/*     HAL_ADCEx_MultiModeConfigChannel() )                             */
	/*   - internal measurement paths: Vbat, temperature sensor, Vref       */
	/*     (set into HAL_ADC_ConfigChannel() or                             */
	/*     HAL_ADCEx_InjectedConfigChannel() )                              */
	LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADCx), ADC_CLOCK_ASYNC_DIV2);

	LL_ADC_InitTypeDef ADC_InitStruct;
	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct;

	/* Set the LeftShift parameter: it is applied to the final result with or without oversampling */
	ADC_InitStruct.Resolution = ADC_RESOLUTION_16B;
	ADC_InitStruct.LowPowerMode = LL_ADC_LP_AUTOWAIT;
	ADC_InitStruct.LeftBitShift = LL_ADC_LEFT_BIT_SHIFT_NONE;
	LL_ADC_Init(ADCx, &ADC_InitStruct);

	ADC_REG_InitStruct.DataTransferMode = LL_ADC_REG_DR_TRANSFER;
	ADC_REG_InitStruct.TriggerSource    = LL_ADC_REG_TRIG_SOFTWARE;
	ADC_REG_InitStruct.SequencerLength  = LL_ADC_REG_SEQ_SCAN_DISABLE;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	ADC_REG_InitStruct.ContinuousMode   = LL_ADC_REG_CONV_CONTINUOUS;
	ADC_REG_InitStruct.Overrun          = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
	LL_ADC_REG_Init(ADCx, &ADC_REG_InitStruct);

	/* Parameters update conditioned to ADC state:                            */
	/* Parameters that can be updated when ADC is disabled or enabled without */
	/* conversion on going on regular and injected groups:                    */
	/*  - Conversion data management      Init.ConversionDataManagement       */
	/*  - LowPowerAutoWait feature        Init.LowPowerAutoWait               */
	/*  - Oversampling parameters         Init.Oversampling                   */
	LL_ADC_SetOverSamplingDiscont(ADCx, LL_ADC_OVS_REG_CONT);
	LL_ADC_SetOverSamplingScope(ADCx, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
	LL_ADC_ConfigOverSamplingRatioShift(ADCx, 1024U, LL_ADC_OVS_SHIFT_RIGHT_6);

	/* Configure the BOOST Mode */
	uint32_t freq;
	freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC);
	freq /= ((ADC_CLOCK_ASYNC_DIV2 >> ADC_CCR_PRESC_Pos) << 1UL);
	MODIFY_REG(ADCx->CR, ADC_CR_BOOST, ADC_CR_BOOST_1 | ADC_CR_BOOST_0);


	/* Configuration of regular group sequencer:                              */
	/* - if scan mode is disabled, regular channels sequence length is set to */
	/*   0x00: 1 channel converted (channel on regular rank 1)                */
	/*   Parameter "NbrOfConversion" is discarded.                            */
	/*   Note: Scan mode is not present by hardware on this device, but       */
	/*   emulated by software for alignment over all STM32 devices.           */
	/* - if scan mode is enabled, regular channels sequence length is set to  */
	/*   parameter "NbrOfConversion".                                         */
	LL_ADC_REG_SetSequencerLength(ADCx, LL_ADC_REG_SEQ_SCAN_DISABLE);

	/* ### - 2 - Start calibration ############################################ */
	LL_ADC_Disable(ADCx);

	/* Start ADC calibration in mode single-ended or differential */
	LL_ADC_StartCalibration(ADCx , LL_ADC_CALIB_OFFSET, LL_ADC_SINGLE_ENDED );

	/* Wait for calibration completion */
	while(LL_ADC_IsCalibrationOnGoing(ADCx) != 0UL)
	{
	}

	/* ### - 3 - Channel configuration ######################################## */
	/* Parameters update conditioned to ADC state:                              */
	/* Parameters that can be updated when ADC is disabled or enabled without   */
	/* conversion on going on regular group:                                    */
	/*  - Channel number                                                        */
	/*  - Channel rank                                                          */

	/* ADC channels preselection */
	ADCx->PCSEL |= (1UL << (__LL_ADC_CHANNEL_TO_DECIMAL_NB((uint32_t)ADCx_CHANNEL) & 0x1FUL));

	/* Set ADC group regular sequence: channel on the selected scan sequence rank */
	LL_ADC_REG_SetSequencerRanks(ADCx, ADC_REGULAR_RANK_1, ADCx_CHANNEL);

	/* Set sampling time of the selected ADC channel */
	LL_ADC_SetChannelSamplingTime(ADCx, ADCx_CHANNEL, ADC_SAMPLETIME_8CYCLES_5);

	/* Configure the offset: offset enable/disable, channel, offset value */

	/* Shift the offset with respect to the selected ADC resolution. */
	/* Offset has to be left-aligned on bit 11, the LSB (right bits) are set to 0 */
	//	ADC_OFFSET_SHIFT_RESOLUTION(&AdcHandle, (uint32_t)ADC_OFFSET_NONE);


	/* Set mode single-ended or differential input of the selected ADC channel */
	LL_ADC_SetChannelSingleDiff(ADCx, ADCx_CHANNEL, ADC_SINGLE_ENDED);


	/* ### - 4 - Start conversion ############################################# */
	/* Perform ADC enable and conversion start if no conversion is on going */

	/* Enable the ADC peripheral */
	/* Enable the ADC peripheral */
	LL_ADC_Enable(ADCx);

	/* Poll for ADC ready flag raised except case of multimode enabled
	   and ADC slave selected. */
	while(LL_ADC_IsActiveFlag_ADRDY(ADCx) == 0UL)
	{
		/*  If ADEN bit is set less than 4 ADC clock cycles after the ADCAL bit
			has been cleared (after a calibration), ADEN bit is reset by the
			calibration logic.
			The workaround is to continue setting ADEN until ADRDY is becomes 1.
			Additionally, ADC_ENABLE_TIMEOUT is defined to encompass this
			4 ADC clock cycle duration */
		/* Note: Test of ADC enabled required due to hardware constraint to     */
		/*       not enable ADC if already enabled.                             */
		if(LL_ADC_IsEnabled(ADCx) == 0UL)
		{
		  LL_ADC_Enable(ADCx);
		}
	}

	/* Clear ADC group regular conversion flag and overrun flag               */
	/* (To ensure of no unknown state from potential previous ADC operations) */
	LL_ADC_ClearFlag_EOC(ADCx);
	LL_ADC_ClearFlag_EOS(ADCx);
	LL_ADC_ClearFlag_OVR(ADCx);

	/* Start ADC group regular conversion */
	LL_ADC_REG_StartConversion(ADCx);

  /* Infinite Loop */
  while (1)
  {

    /* Read the converted value */
    uwConvertedValue = ADCx->DR;

    /* Convert the result from 20 bit value to the voltage dimension (mV unit) */
    /* Vref = 3.3 V */
    uwInputVoltage = uwConvertedValue * 3300;
    uwInputVoltage = uwInputVoltage / 0xFFFF0;
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE BYPASS)
  *            SYSCLK(Hz)                     = 400000000 (CPU Clock)
  *            HCLK(Hz)                       = 200000000 (AXI and AHBs Clock)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 4
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
	  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
	  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
	  {
	  }
	  LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
	  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE3);
	  LL_RCC_HSI_Enable();

	   /* Wait till HSI is ready */
	  while(LL_RCC_HSI_IsReady() != 1)
	  {

	  }
	  LL_RCC_HSI_SetCalibTrimming(32);
	  LL_RCC_HSI_SetDivider(LL_RCC_HSI_DIV1);

	  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSI);
	  LL_RCC_PLL1P_Enable();
	  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
	  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
	  LL_RCC_PLL1_SetM(4);
	  LL_RCC_PLL1_SetN(60);
	  LL_RCC_PLL1_SetP(2);
	  LL_RCC_PLL1_SetQ(2);
	  LL_RCC_PLL1_SetR(2);
	  LL_RCC_PLL1_Enable();

	   /* Wait till PLL is ready */
	  while(LL_RCC_PLL1_IsReady() != 1)
	  {
	  }

	  LL_RCC_PLL2P_Enable();
	  LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
	  LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_MEDIUM);
	  LL_RCC_PLL2_SetM(4);
	  LL_RCC_PLL2_SetN(9);
	  LL_RCC_PLL2_SetP(2);
	  LL_RCC_PLL2_SetQ(2);
	  LL_RCC_PLL2_SetR(2);
	  LL_RCC_PLL2_SetFRACN(3072);
	  LL_RCC_PLL2FRACN_Enable();
	  LL_RCC_PLL2_Enable();

	   /* Wait till PLL is ready */
	  while(LL_RCC_PLL2_IsReady() != 1)
	  {
	  }

	   /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
	   LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);

	  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
	  LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
	  LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
	  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
	  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
	  LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
	  LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

	  LL_Init1msTick(480000000);

	  LL_SetSystemCoreClock(480000000);
#if (CONF_USE_MCO2==1U)
	  LL_RCC_ConfigMCO(LL_RCC_MCO2SOURCE_LSI, LL_RCC_MCO2_DIV_2);
#endif /* CONF_USE_MCO2 */
	 LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLL2P);
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
