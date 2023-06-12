#include "stepper.h"



void microDelay(TIM_HandleTypeDef* tim_device, uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(tim_device, 0);
  while (__HAL_TIM_GET_COUNTER(tim_device) < delay);
}


void step(TIM_HandleTypeDef* tim_device, int steps, uint8_t direction, uint16_t delay)
{
  int x;
  if (direction == 0) // clockwise
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
  else // counter-clockwise
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);

  for(x=0; x<steps; x=x+1)
  {
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
    microDelay(tim_device, delay);
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
    microDelay(tim_device, delay);
  }

}
