# MPU6050 STM32 Library (HAL / DMA)

C driver for the **InvenSense MPU6050** IMU, written for **STM32** microcontrollers using **STM32 HAL** (I2C transfer).  
Author: **papaj-23**

## Features

- STM32 HAL I2C support (`HAL_I2C_Mem_Read/Write`)
- DMA reads for:
  - single 14B measurement payload read (ACCEL+TEMP+GYRO)
  - FIFO count read (2B)
  - FIFO burst read (N bytes)
- Operating modes:
  - `MPU_SINGLE_MODE` — Data Ready interrupt, FIFO off
  - `MPU_BURST_MODE` — FIFO overflow interrupt, FIFO on
  - `MPU_LOWPOWER_CYCLE_MODE` — accel-only duty-cycled mode (gyro + temp disabled)
- Per-channel enable/disable (accel axes, gyro axes, temp)
- FIFO content selection (accel / gyro axes / temp)
- FIFO reset helper (for overflow handling)
- Gyro/Accel range configuration + signal path reset
- Built-in self-test procedure (factory trim + response ratio in %)
- Parsing utilities:
  - raw 14B → `int16_t[7]`
  - `int16_t[7]` → scaled `MPU6050_data_t`

## Requirements

- STM32 project using HAL
- Configured I2C peripheral
- Configured DMA for cyclic measurement data handling
- MPU6050 connected via I2C:
  - 7-bit address: `0x68` (default) or `0x69` (AD0 high)
  - This implementation uses `0x68`

> Note: header includes `stm32l4xx_hal.h`, but the driver is generally portable across STM32 families using HAL.

## Files

- `mpu-6050.h` — public API, types, documentation
- `mpu-6050.c` — implementation (register map, helpers, HAL calls)

## Quick start

### 1) Add files to your project
Add `mpu-6050.c` to sources and `mpu-6050.h` to include paths.

### 2) Create handle + buffers

```c
#include "mpu-6050.h"

static uint8_t tx_buf[32];      // optional (reserved for future extensions)
static uint8_t rx_buf[1024];    // must be >= max burst_count you use

extern I2C_HandleTypeDef hi2c1;

/* Handle to delay function is optional though recomended. 
You can use HAL_Delay or your own implementation of millisecond delay function */
static void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

MPU6050_t imu = {
    .hi2c = &hi2c1,
    .tx_buffer = tx_buf,
    .rx_buffer = rx_buf,
    .delay_ms_wrapper = delay_ms,
    .burst_count = 0,   // count of data bytes you want to read within a single burst read
};
```

### 3) Init + basic config

```c
HAL_StatusTypeDef st;

st = MPU_6050_Init(&imu);
if(st != HAL_OK) Error_Handler();

MPU_6050_Set_Accel_Range(&imu, G_4);
MPU_6050_Set_Gyro_Range(&imu, DPS_500);

MPU_6050_Set_Mode(&imu, MPU_SINGLE_MODE);
```

## Reading data

### Single measurement read (DMA)

Reads 14 bytes starting at `ACCEL_XOUT_H`:
- accel (6B), temp (2B), gyro (6B)

```c
MPU_6050_Single_Read(&imu);  // starts DMA read into imu.rx_buffer
```

In your I2C DMA complete callback (or after completion), parse and scale:

```c
int16_t raw7[7];
MPU_6050_parse_payload(imu.rx_buffer, raw7);

MPU6050_data_t meas = MPU_6050_payload_to_readable(&imu, raw7);
/* meas.accel_* [g], meas.gyro_* [deg/s], meas.temp [°C] */
```

> The library doesn’t implement callbacks yet; use standard HAL callbacks and your application logic.

## FIFO burst workflow

### 1) Configure FIFO content + burst mode

```c
MPU_6050_Set_FIFO_Content(&imu, FIFO_ACCEL, MPU_ENABLE);
MPU_6050_Set_FIFO_Content(&imu, FIFO_TEMP,  MPU_ENABLE);
MPU_6050_Set_FIFO_Content(&imu, FIFO_GYRO_X | FIFO_GYRO_Y | FIFO_GYRO_Z, MPU_ENABLE);

MPU_6050_Set_Mode(&imu, MPU_BURST_MODE);
```

### 2) Read FIFO count (DMA) + convert

```c
MPU_6050_Read_FIFO_Cnt(&imu);      // DMA reads 2 bytes into imu.fifo_counter_raw[]
MPU_6050_Process_Burst_Cnt(&imu);  // converts to imu.fifo_counter (uint16_t)
```

### 3) Set `burst_count` and read FIFO (DMA)

```c
imu.burst_count = imu.fifo_counter;   // or clamp/align to payload size
MPU_6050_Burst_Read(&imu);            // DMA reads burst_count bytes into imu.rx_buffer
```

### FIFO overflow handling

In `MPU_BURST_MODE` the interrupt source is FIFO overflow. On overflow event:

```c
MPU_6050_FIFO_Reset(&imu);
```

## Low power accel-only cycle mode

Configures:
- FIFO off
- temp disabled
- gyros in standby
- accel enabled
- cycle enabled
- Data Ready interrupt enabled

```c
MPU_6050_Set_Mode(&imu, MPU_LOWPOWER_CYCLE_MODE);
MPU_6050_Set_Lp_Wakeup_Freq(&imu, F_20HZ); // example
```

## Self-test

```c
MPU6050_selftest_t st_res;
MPU_6050_Self_Test(&imu, &st_res);
/* st_res: accel_x/y/z, gyro_x/y/z (% response ratio) */
```

Device should remain stationary. A delay callback (`delay_ms_wrapper`) is recommended.

## API overview

- `MPU_6050_Init()`
- `MPU_6050_Set_Mode()`
- `MPU_6050_Set_Sleep()`
- `MPU_6050_Set_Lp_Wakeup_Freq()`
- `MPU_6050_Set_Channel_State()`
- `MPU_6050_FIFO_Reset()`
- `MPU_6050_Self_Test()`
- `MPU_6050_Set_Gyro_Range()`
- `MPU_6050_Set_Accel_Range()`
- `MPU_6050_Set_FIFO_Content()`
- `MPU_6050_Single_Read()` (DMA)
- `MPU_6050_Read_FIFO_Cnt()` (DMA)
- `MPU_6050_Process_Burst_Cnt()`
- `MPU_6050_Burst_Read()` (DMA)
- `MPU_6050_parse_payload()`
- `MPU_6050_payload_to_readable()`

## Notes

- Driver stores selected ranges (`handles->gyro_scale`, `handles->accel_scale`) for scaling.
- FIFO count is read into `fifo_counter_raw[2]` and then converted to `fifo_counter`.
- Default I2C timeout is short (10 ms) — adjust if needed.

