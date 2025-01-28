
/*
 * NEW_BTSDrv1.C
 *
 *  Created on: Jan 9, 2025
 *      Author: Arif
 */

#include "NEW_BTSDrv1.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdio.h>

void BTS_Enable(BtsDrv *drv) {
    /* Enable the BTS driver based on the specified direction
     * Steps:
     * 1. Validate the direction based on LeftEnable and RightEnable.
     * 2. Disable REN, LEN, RPWM, LPWM to ensure a stable state.
     * 3. Delay using a for loop for 10 milliseconds to stabilize the H-Bridge.
     * 4. Enable corresponding control pins for the selected direction.
     */

    // Step 1: Validate direction
    if (drv->LeftEnable || drv->RightEnable) {
        // Step 2: Disable REN, LEN, RPWM, and LPWM
        HAL_GPIO_WritePin(HBRDG_EN_CH_A_GPIO_Port, HBRDG_EN_CH_A_Pin, GPIO_PIN_RESET); // Disable REN
        HAL_GPIO_WritePin(HBRDG_EN_CH_B_GPIO_Port, HBRDG_EN_CH_B_Pin, GPIO_PIN_RESET); // Disable LEN
        HAL_GPIO_WritePin(HBRDG_PWM_CH_A_GPIO_Port, HBRDG_PWM_CH_A_Pin, GPIO_PIN_RESET); // Disable RPWM
        HAL_GPIO_WritePin(HBRDG_PWM_CH_B_GPIO_Port, HBRDG_PWM_CH_B_Pin, GPIO_PIN_RESET); // Disable LPWM

        // Step 3: Approximate 1ms delay at 1MHz clock frequency
        for (volatile uint32_t i = 0; i < 1000; i++) {
            __NOP(); // No operation
        }

        }

        // Step 4: Enable the corresponding direction
        if (drv->LeftEnable) {
            // Enable Left Direction
            HAL_GPIO_WritePin(HBRDG_EN_CH_B_GPIO_Port, HBRDG_EN_CH_B_Pin, GPIO_PIN_SET); // Enable LEN
            HAL_GPIO_WritePin(HBRDG_PWM_CH_B_GPIO_Port, HBRDG_PWM_CH_B_Pin, GPIO_PIN_SET); // Start LPWM
            drv->RightEnable = 0; // Ensure RightEnable is disabled
            drv->LeftEnable = 1;    //Update status: Left is Enabled

        } else if (drv->RightEnable) {
                   // Enable Right Direction
                   HAL_GPIO_WritePin(HBRDG_EN_CH_A_GPIO_Port, HBRDG_EN_CH_A_Pin, GPIO_PIN_SET); // Enable REN
                   HAL_GPIO_WritePin(HBRDG_PWM_CH_A_GPIO_Port, HBRDG_PWM_CH_A_Pin, GPIO_PIN_SET); // Start RPWM
                   drv->LeftEnable = 0; // Ensure LeftEnable is disabled
                   drv->RightEnable = 1;  //Update status: Right is Enabled
    } else {
        // Invalid direction OR Error
        drv->RightEnable = 0;
        drv->LeftEnable = 0;
        drv->DutyCycle = 0; // Reset duty cycle to safe state
    }
}



void BTS_Disable(BtsDrv *drv) {
    /* Action to disable the BTS driver
     * 1. Check the null pointer for drv struct
     * 2. Disable R_EN, L_EN
     * 3. Disable RPWM, LPWM
     * 4. Change status to disabled
     * 5. Add a small delay to stabilize the system
     */

    // Step 1: Check the null pointer for drv struct
    if (drv == NULL) {
        // Handle the error (return, log, etc.)
        return; // Exit if the pointer is NULL
    }

    // Step 2: Disable R_EN, L_EN (Right and Left Enable pins)
    HAL_GPIO_WritePin(HBRDG_EN_CH_A_GPIO_Port, HBRDG_EN_CH_A_Pin, GPIO_PIN_RESET); // Disable R_EN
    HAL_GPIO_WritePin(HBRDG_EN_CH_B_GPIO_Port, HBRDG_EN_CH_B_Pin, GPIO_PIN_RESET); // Disable L_EN

    // Step 3: Disable RPWM, LPWM (Right and Left PWM pins)
    HAL_GPIO_WritePin(HBRDG_PWM_CH_A_GPIO_Port, HBRDG_PWM_CH_A_Pin, GPIO_PIN_RESET); // Disable RPWM
    HAL_GPIO_WritePin(HBRDG_PWM_CH_B_GPIO_Port, HBRDG_PWM_CH_B_Pin, GPIO_PIN_RESET); // Disable LPWM

    // Step 4: Change status to disabled
    drv->RightEnable = 0;    // Right direction disabled
    drv->LeftEnable = 0;     // Left direction disabled
    drv->DutyCycle = 0;      // Reset duty cycle to 0%


    // Step 5: Add a small delay to stabilize the system
    HAL_Delay(10); // 10 ms delay (adjustable based on system requirements)
}
void BTS_SetDirection(BtsDrv *drv, uint8_t direction) {
    /* Action to set the target direction of the BTS driver
     * 1. Disable REN, LEN, RPWM, LPWM to ensure stable state
     * 2. Based on the `direction` parameter:
     *      a. If direction is LEFT:
     *          i. Enable LEN
     *          ii. Start LPWM
     *      b. If direction is RIGHT:
     *          i. Enable REN
     *          ii. Start RPWM
     *
     */

    // Step 1: Call BTS_Disable to ensure all signals are reset and the system is in a stable state
    BTS_Disable(drv);  // Disables all necessary signals and updates the driver status


    // Step 2: Based on the direction parameter, set the new direction
    if (direction == BTS_DRV_SET_LEFT) {
        // Enable Left Enable pin (LEN) and start Left PWM (LPWM)
        HAL_GPIO_WritePin(HBRDG_EN_CH_B_GPIO_Port, HBRDG_EN_CH_B_Pin, GPIO_PIN_SET);  // Enable LEN
        HAL_GPIO_WritePin(HBRDG_PWM_CH_B_GPIO_Port, HBRDG_PWM_CH_B_Pin, GPIO_PIN_SET); // Start LPWM
        drv->DirectedLeft = 1;   // Set direction to LEFT
        drv->DirectedRight = 0;  // Clear the right direction
    }

    else if (direction == BTS_DRV_SET_RIGHT) {
        // Enable Right Enable pin (REN) and start Right PWM (RPWM)
        HAL_GPIO_WritePin(HBRDG_EN_CH_A_GPIO_Port, HBRDG_EN_CH_A_Pin, GPIO_PIN_SET);  // Enable REN
        HAL_GPIO_WritePin(HBRDG_PWM_CH_A_GPIO_Port, HBRDG_PWM_CH_A_Pin, GPIO_PIN_SET); // Start RPWM
        drv->DirectedLeft = 0;   // Clear the left direction
        drv->DirectedRight = 1;  // Set direction to RIGHT
    }

}
void BTS_SetDutyCycle(TIM_HandleTypeDef *htim, uint8_t duty_cycle_A, uint8_t duty_cycle_B) {
    /* Set PWM duty cycles for H-Bridge motor control
     * Parameters:
     * - htim: Pointer to the timer handle
     * - duty_cycle_A: Desired duty cycle for Channel A (0-100%)
     * - duty_cycle_B: Desired duty cycle for Channel B (0-100%)

     * Assumes:
     * - Channel A corresponds to TIM_CHANNEL_1
     * - Channel B corresponds to TIM_CHANNEL_2
     * - Timer is pre-configured for 25kHz PWM with Prescaler = 31 and ARR = 104.
     */

    // Validate and clamp duty cycles
    if (duty_cycle_A > 100) duty_cycle_A = 100;
    if (duty_cycle_B > 100) duty_cycle_B = 100;

    // Calculate pulse widths based on duty cycles
    uint16_t pulse_A = (duty_cycle_A * __HAL_TIM_GET_AUTORELOAD(htim)) / 100;
    uint16_t pulse_B = (duty_cycle_B * __HAL_TIM_GET_AUTORELOAD(htim)) / 100;

    // Set PWM duty cycle for Channel A
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pulse_A);

    // Set PWM duty cycle for Channel B
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, pulse_B);

    // Start PWM output for both channels
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
}


