#include "isr_templates.h"

HAL_StatusTypeDef MPU_6050_int_isr(MPU_6050_t *handles) {
    MEM_CHECK(handles);
    HAL_StatusTypeDef status = HAL_OK;
    if(handles->bus_status != MPU_BUS_UNLOCKED) {
        return HAL_BUSY;
    }

    status = HAL_I2C_Mem_Read_DMA(handles->hi2c, I2C_ADDRESS_HAL,
                                  INT_STATUS_REG, MPU6050_REG_SIZE,
                                  &handles->int_status, 1);

    return status;
}


HAL_StatusTypeDef MPU_6050_i2c_rxcplt_isr(MPU_6050_t *handles) {
    HAL_StatusTypeDef status = HAL_OK;

    switch(handles->meas_mode) {
    case MPU_SINGLE_MODE:
        if(handles->int_status & DATA_READY_INT) {
            handles->int_status &= ~DATA_READY_INT;
            status = MPU_6050_single_read(handles);
        }
    break;

    case MPU_BURST_MODE:
        MPU_6050_process_burst_cnt(handles);
        if(handles->int_status & FIFO_OFLOW_INT) {
            handles->int_status &= ~FIFO_OFLOW_INT;
            handles->fifo_oflow_flag = 1U;
        }
        else if(handles->fifo_count >= BURST_COUNT) {
            status = MPU_6050_burst_read(handles);
        }
    break;

    case MPU_LOWPOWER_CYCLE_MODE:
        if(handles->int_status & DATA_READY_INT) {
            handles->int_status &= ~DATA_READY_INT;
            status = MPU_6050_low_power_read(handles);
        }
    break;
    
    default:
        status = HAL_ERROR;
    break;
    }

    return status;
}