#ifndef ISR_TEMPLATES
#define ISR_TEMPLATES

#include "mpu-6050.h"

/**
  * @brief  Handles interrupts coming from INT pin
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * This function triggers a dma read of int status register to
  * determine which interrupt occured
  * 
  * @attention This function can (and should) be called from isr context 
  * 
  * @note This function should not be called from ISR context
  *
  * @retval HAL status.
  */
__attribute(weak) HAL_StatusTypeDef MPU_6050_int_isr(MPU_6050_t *handles);

/**
  * @brief  Handle interrupts coming from INT pin
  * @param  handles Pointer to MPU6050 handle structure.
  *
  * @details
  * Triggers single read if received DATA_RDY_INT(single read mode)
  * or FIFO reset if received FIFO_OFLOW_INT(burst read)
  * 
  * @attention This function can (and should) be called from isr context 
  * 
  * @note This just a template, example use case. 
  *
  * @retval HAL status.
  */
__attribute(weak) HAL_StatusTypeDef MPU_6050_i2c_rxcplt_isr(MPU_6050_t *handles);

#endif