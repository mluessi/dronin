/**
 ******************************************************************************
 * @addtogroup BrainFPVTargets BrainFPV Targets
 * @{
 * @addtogroup BrainFPV-RE1 BrainFPV-RE1 support files
 * @{
 *
 * @file       pios_board.h 
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @author     BrainFPV, Copyright (C) 2015
 * @brief      Board define file for FlyingF4
 * @see        The GNU Public License (GPL) Version 3
 * 
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#ifndef STM3210E_INS_H_
#define STM3210E_INS_H_

#include <stdbool.h>

#if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
#define DEBUG_LEVEL 0
#define DEBUG_PRINTF(level, ...) {if(level <= DEBUG_LEVEL && pios_com_debug_id > 0) { PIOS_COM_SendFormattedStringNonBlocking(pios_com_debug_id, __VA_ARGS__); }}
#else
#define DEBUG_PRINTF(level, ...)
#endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */

//------------------------
// Timers and Channels Used
//------------------------
/*
Timer | Channel 1 | Channel 2 | Channel 3 | Channel 4
------+-----------+-----------+-----------+----------
TIM1  |           |           |           |
TIM2  | --------------- PIOS_DELAY -----------------
TIM3  |           |           |           |
TIM4  |           |           |           |
TIM5  |           |           |           |
TIM6  |           |           |           |
TIM7  |           |           |           |
TIM8  |           |           |           |
------+-----------+-----------+-----------+----------
*/

//------------------------
// DMA Channels Used
//------------------------
/* Channel 1  -                                 */
/* Channel 2  -                                 */
/* Channel 3  -                                 */
/* Channel 4  -                                 */
/* Channel 5  -                                 */
/* Channel 6  -                                 */
/* Channel 7  -                                 */
/* Channel 8  -                                 */
/* Channel 9  -                                 */
/* Channel 10 -                                 */
/* Channel 11 -                                 */
/* Channel 12 -                                 */

//------------------------
// BOOTLOADER_SETTINGS
//------------------------
#define BOARD_READABLE					true
#define BOARD_WRITABLE					true
#define MAX_DEL_RETRYS					3


//------------------------
// PIOS_LED
//------------------------
#define PIOS_LED_GREEN		0
#define PIOS_LED_RED		1

#define PIOS_LED_HEARTBEAT	PIOS_LED_GREEN
#define PIOS_LED_ALARM		PIOS_LED_RED


//------------------------
// PIOS_WDG
//------------------------
#define PIOS_WATCHDOG_TIMEOUT			250
#define PIOS_WDG_REGISTER			RTC_BKP_DR4

//------------------------
// PIOS_I2C
// See also pios_board.c
//------------------------
#define PIOS_I2C_MAX_DEVS				2
extern uint32_t pios_i2c_flexi_id;
#define PIOS_I2C_MAIN_ADAPTER			(pios_i2c_flexi_id)
//-------------------------
// PIOS_COM
//
// See also pios_board.c
//-------------------------
extern uintptr_t pios_com_telem_rf_id;
extern uintptr_t pios_com_gps_id;
extern uintptr_t pios_com_telem_usb_id;
extern uintptr_t pios_com_bridge_id;
extern uintptr_t pios_com_vcp_id;
extern uintptr_t pios_com_mavlink_id;
extern uintptr_t pios_com_frsky_sensor_hub_id;
extern uintptr_t pios_com_frsky_sport_id;
extern uintptr_t pios_com_lighttelemetry_id;
extern uintptr_t pios_com_picoc_id;
extern uintptr_t pios_com_hott_id;
extern uintptr_t pios_com_picoc_id;
extern uintptr_t pios_com_logging_id;


#define PIOS_COM_GPS                    (pios_com_gps_id)
#define PIOS_COM_TELEM_USB              (pios_com_telem_usb_id)
#define PIOS_COM_TELEM_RF               (pios_com_telem_rf_id)
#define PIOS_COM_BRIDGE                 (pios_com_bridge_id)
#define PIOS_COM_VCP                    (pios_com_vcp_id)
#define PIOS_COM_MAVLINK                (pios_com_mavlink_id)
#define PIOS_COM_HOTT                   (pios_com_hott_id)
#define PIOS_COM_FRSKY_SENSOR_HUB       (pios_com_frsky_sensor_hub_id)
#define PIOS_COM_FRSKY_SPORT            (pios_com_frsky_sport_id)
#define PIOS_COM_LIGHTTELEMETRY         (pios_com_lighttelemetry_id)
#define PIOS_COM_PICOC                  (pios_com_picoc_id)
#define PIOS_COM_LOGGING                (pios_com_logging_id)

#if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
extern uintptr_t pios_com_debug_id;
#define PIOS_COM_DEBUG                  (pios_com_debug_id)
#endif	/* PIOS_INCLUDE_DEBUG_CONSOLE */


//------------------------
// TELEMETRY
//------------------------
#define TELEM_QUEUE_SIZE				80
#define PIOS_TELEM_STACK_SIZE			624
	

#define PIOS_SYSCLK						SYSCLK_FREQ
//	Peripherals that belongs to APB1 are:
//	DAC			|PWR				|CAN1,2
//	I2C1,2,3	|UART4,5			|USART3,2
//	I2S3Ext		|SPI3/I2S3			|SPI2/I2S2
//	I2S2Ext		|IWDG				|WWDG
//	RTC/BKP reg	
// TIM2,3,4,5,6,7,12,13,14

// Calculated as SYSCLK / APBPresc * (APBPre == 1 ? 1 : 2)   
// Default APB1 Prescaler = 4 
#define PIOS_PERIPHERAL_APB1_CLOCK		(PIOS_SYSCLK / 2)

//	Peripherals belonging to APB2
//	SDIO			|EXTI				|SYSCFG			|SPI1
//	ADC1,2,3				
//	USART1,6
//	TIM1,8,9,10,11
//
// Default APB2 Prescaler = 2
//
#define PIOS_PERIPHERAL_APB2_CLOCK		PIOS_SYSCLK


//-------------------------
// Interrupt Priorities
//-------------------------
#define PIOS_IRQ_PRIO_LOW				12              // lower than RTOS
#define PIOS_IRQ_PRIO_MID				8               // higher than RTOS
#define PIOS_IRQ_PRIO_HIGH				5               // for SPI, ADC, I2C etc...
#define PIOS_IRQ_PRIO_HIGHEST			4               // for USART etc...


//------------------------
// PIOS_RCVR
// See also pios_board.c
//------------------------
#define PIOS_RCVR_MAX_CHANNELS			12
#define PIOS_GCSRCVR_TIMEOUT_MS			100

//-------------------------
// Receiver PPM input
//-------------------------
#define PIOS_PPM_NUM_INPUTS				12

//-------------------------
// Receiver PWM input
//-------------------------
#define PIOS_PWM_NUM_INPUTS				8

//-------------------------
// Receiver DSM input
//-------------------------
#define PIOS_DSM_NUM_INPUTS				12

//-------------------------
// Receiver S.Bus input
//-------------------------
#define PIOS_SBUS_NUM_INPUTS			(16+2)

//-------------------------
// Servo outputs
//-------------------------
#define PIOS_SERVO_UPDATE_HZ			50
#define PIOS_SERVOS_INITIAL_POSITION	0 /* dont want to start motors, have no pulse till settings loaded */

//--------------------------
// Timer controller settings
//--------------------------
#define PIOS_TIM_MAX_DEVS				8

//-------------------------
// ADC
// PIOS_ADC_PinGet(0) = Voltage 1
// PIOS_ADC_PinGet(1) = Voltage 2
// PIOS_ADC_PinGet(4) = VREF
// PIOS_ADC_PinGet(5) = Temperature sensor
// -------------------------
#define PIOS_DMA_PIN_CONFIG                                                                 \
    {                                                                                           \
		{ GPIOC, GPIO_Pin_0, ADC_Channel_10 },                                                \
		{ GPIOC, GPIO_Pin_1, ADC_Channel_11 },                                                \
		{ GPIOC, GPIO_Pin_3, ADC_Channel_13 },                                                \
        { NULL, 0, ADC_Channel_Vrefint }, /* Voltage reference */         \
        { NULL, 0, ADC_Channel_TempSensor }, /* Temperature sensor */        \
    }

/* we have to do all this to satisfy the PIOS_ADC_MAX_SAMPLES define in pios_adc.h */
/* which is annoying because this then determines the rate at which we generate buffer turnover events */
/* the objective here is to get enough buffer space to support 100Hz averaging rate */
#define PIOS_ADC_NUM_CHANNELS           5
#define PIOS_ADC_MAX_OVERSAMPLING       2
#define PIOS_ADC_USE_ADC2               0

#define VREF_PLUS			3.3

#define PIOS_ADC_SUB_DRIVER_MAX_INSTANCES       3

//-------------------------
// USB
//-------------------------
#define PIOS_USB_ENABLED				1 /* Should remove all references to this */

#endif /* STM3210E_INS_H_ */


#if defined(PIOS_INCLUDE_VIDEO)
#include <pios.h>
#include <pios_stm32.h>
#include <pios_tim_priv.h>

struct pios_osd_bw_cfg_t {
	TIM_TimeBaseInitTypeDef tim_base_init;
	TIM_OCInitTypeDef tim_oc_init;
	GPIO_InitTypeDef gpio_init;
	uint32_t remap;
	const struct pios_tim_channel * bw_channels;
};
#endif /* PIOS_INCLUDE_VIDEO */


/**
 * @}
 * @}
 */
