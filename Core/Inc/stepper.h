#include "stm32f1xx_hal.h"

#define DIR_PIN GPIO_PIN_1
#define DIR_PORT GPIOA
#define STEP_PIN GPIO_PIN_2
#define STEP_PORT GPIOA

void microDelay(TIM_HandleTypeDef* tim_device, uint16_t delay);
void step (TIM_HandleTypeDef* tim_device, int steps, uint8_t direction, uint16_t delay);


