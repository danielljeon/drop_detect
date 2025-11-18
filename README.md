# drop_detect

![arm_gcc_build](https://github.com/danielljeon/drop_detect/actions/workflows/arm_gcc_build.yaml/badge.svg)

Soldering iron drop-triggered shield for real time embedded systems university
course (firmware).

---

<details markdown="1">
  <summary>Table of Contents</summary>

<!-- TOC -->
* [drop_detect](#drop_detect)
  * [1 Overview](#1-overview)
    * [1.1 Bill of Materials (BOM)](#11-bill-of-materials-bom)
    * [1.2 Block Diagram](#12-block-diagram)
    * [1.3 Pin Configurations](#13-pin-configurations)
    * [1.4 Clock Configurations](#14-clock-configurations)
  * [2 FreeRTOS](#2-freertos)
<!-- TOC -->

</details>

---

## 1 Overview

### 1.1 Bill of Materials (BOM)

| Manufacturer Part Number | Manufacturer                         | Description                 | Quantity | Notes |
|--------------------------|--------------------------------------|-----------------------------|---------:|-------|
| NUCLEO-L432KC            | STMicroelectronics                   | 32-bit MCU Nucleo Dev Board |        1 |       |
| Adafruit BNO085 board    | CEVA Technologies, Inc. via Adafruit | 9-DOF IMU Adafruit board    |        1 |       |

### 1.2 Block Diagram

![drop_detect.drawio.png](docs/drop_detect.drawio.png)

> Drawio file here: [drop_detect.drawio](docs/drop_detect.drawio).

### 1.3 Pin Configurations

<details markdown="1">
  <summary>CubeMX Pinout</summary>

![CubeMX Pinout.png](docs/CubeMX%20Pinout.png)

</details>

<details markdown="1">
  <summary>Pin & Peripherals Table</summary>

| STM32L432KC | Peripheral              | Config                | Connection                       | Notes                                 |
|-------------|-------------------------|-----------------------|----------------------------------|---------------------------------------|
| PB3         | `SYS_JTDO-SWO`          |                       | Onboard ST-Link                  |                                       |
| PA14        | `SYS_JTCK-SWCLK`        |                       | Onboard ST-Link                  |                                       |
| PA13        | `SYS_JTMS-SWDIO`        |                       | Onboard ST-Link                  |                                       |
|             | `TIM2_CH1`              | PWM no output         |                                  | BNO085 SH2 driver timer.              |
| PA5         | `SPI1_SCK`              |                       | BNO085 Pin 19: `H_SCL/SCK/RX`    |                                       |
| PA4         | `GPIO_Output` (SPI1 CS) | Set high              | BNO085 Pin 18: `H_CSN`           |                                       |
| PA6         | `SPI1_MISO`             |                       | BNO085 Pin 20: `H_SDA/H_MISO/TX` |                                       |
| PA7         | `SPI1_MOSI`             |                       | BNO085 Pin 17: `SA0/H_MOSI`      |                                       |
| PB0         | `GPIO_EXTI0`            | Pull-up, falling edge | BNO085 Pin 14: `H_INTN`          |                                       |
| PB1         | `GPIO_Output`           | Set high              | BNO085 Pin 6: `PS0/Wake`         | Pull low to trigger wake.             |
|             |                         | Hardware pull-up      | BNO085 Pin 5: `PS1`              |                                       |
| PA1         | `GPIO_Output`           | Set high              | BNO085 Pin 11: `NRST`            | Pull low to reset.                    |
| PA8         | `TIM1_CH1`              | PWM Generation CH1    | WS2812B Pin: `DIN`               | DIN pin number depends on IC variant. |
| PC15        | `GPIO_Output`           | Pull-down, set low    | GPIO                             | General trigger pin, active high.     |

</details>

### 1.4 Clock Configurations

```
16 MHz High Speed External (HSI)
↓
Phase-Locked Loop Main (PLLM)
↓
80 MHz SYSCLK
↓
80 MHz HCLK
↓
 → 80 MHz APB1 (Maxed) → 80 MHz APB1 Timer
 → 80 MHz APB2 (Maxed) → 80 MHz APB2 Timer
```

---

## 2 FreeRTOS

The `SYS` Timebase Source is set to `TIM16` in order to free `SysTick` for
the FreeRTOS kernal. 
