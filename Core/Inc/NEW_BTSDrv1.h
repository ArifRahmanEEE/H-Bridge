/*
 * BTS7960_Driver.h
 *
 *  Created on: Jan 9, 2025
 *      Author: Arif
 */

#ifndef INC_NEW_BTSDRV1_H_
#define INC_NEW_BTSDRV1_H_

#include <stdint.h>
#include "stm32f4xx_hal.h" // Include HAL driver

// PIN define
#define HBRDG_PWM_CH_A_Pin	 		GPIO_PIN_9
#define HBRDG_PWM_CH_A_GPIO_Port	GPIOE
#define HBRDG_PWM_CH_B_Pin 			GPIO_PIN_11
#define HBRDG_PWM_CH_B_GPIO_Port 	GPIOE
#define HBRDG_EN_CH_A_Pin 			GPIO_PIN_11
#define HBRDG_EN_CH_A_GPIO_Port 	GPIOA
#define HBRDG_EN_CH_B_Pin 			GPIO_PIN_12
#define HBRDG_EN_CH_B_GPIO_Port 	GPIOA
#define BTS_DRIVER_Pin 				GPIO_PIN_5
#define BTS_DRIVER_GPIO_Port 		GPIOB


// Declaring integer constant

typedef enum{


	BTS_DRV_ENABLED,
	BTS_DRV_DISABLE,
	BTS_DRV_SET_LEFT = 0,
	BTS_DRV_SET_RIGHT,
	BTS_DRV_PWM,

}Motor_status;


// BtsDriver Control Object
typedef struct {
    uint8_t RightEnable;  // Motor enabled in the right direction
    uint8_t LeftEnable;   // Motor enabled in the left direction
    uint8_t DirectedRight; // Direction (0 = clockwise, 1 = counterclockwise)
    uint8_t DirectedLeft;  // Direction (0 = clockwise, 1 = counterclockwise)
    uint8_t DutyCycle;     // Duty cycle: 0-100 (%)
} BtsDrv;



// functions prototypes
   // Function Prototypes
   void BTSDrv_Init(TIM_HandleTypeDef *htim); // Initialize timer for motor control
   void BTS_Enable(BtsDrv *drv);              // Enable motor driver
   void BTS_Disable(BtsDrv *drv);             // Disable motor driver
   void BTS_SetDirection(BtsDrv *drv, uint8_t direction); // Set motor direction
   void BTS_SetDutyCycle(TIM_HandleTypeDef *htim, uint8_t duty_cycle_A, uint8_t duty_cycle_B); // Set PWM duty cycle





#endif /* INC_NEW_BTSDRV1_H_ */
