# STM32WBA Bluetooth® LE AI Fan Control

The objective is to showcase the capabilities of the STM32 Ecosystem for WBA customers. This demonstration features a BLE link, an AI algorithm, and motor control operating concurrently.

The selected use case is a fan equipped with anomaly detection, which is reported via BLE.

### __Keywords__

Connectivity, BLE, BLE protocol, BLE pairing, BLE profile, FreeRTOS

### __Directory contents__

  - BLE_AI_Control_Fan/Core/Inc/app_common.h                                             App Common application configuration file for STM32WPAN Middleware
  - BLE_AI_Control_Fan/Core/Inc/app_conf.h                                               Application configuration file for STM32WPAN Middleware
  - BLE_AI_Control_Fan/Core/Inc/app_entry.h                                              Interface to the application 
  - BLE_AI_Control_Fan/Core/Inc/main.h                                                   Header for main.c file. This file contains the common defines of the application
  - BLE_AI_Control_Fan/Core/Inc/NanoEdgeAI.h                                             This file contains the interface of NanoEdge Algorithm
  - BLE_AI_Control_Fan/Core/Inc/stm32wba55g_discovery_conf.h                             STM32WBA55G_DK1 board configuration file. This file should be copied to the application folder and renamed to tm32wba55g_discovery_conf.h
  - BLE_AI_Control_Fan/Core/Inc/stm32wbaxx_hal_conf.h                                    HAL configuration file
  - BLE_AI_Control_Fan/Core/Inc/stm32wbaxx_it.h                                          This file contains the headers of the interrupt handlers
  - BLE_AI_Control_Fan/Core/Inc/utilities_conf.h                                         Header for configuration file for STM32 Utilities
  - BLE_AI_Control_Fan/STM32_WPAN/App/app_ble.h                                          Header for ble application 
  - BLE_AI_Control_Fan/STM32_WPAN/App/ble_conf.h                                         Configuration file for BLE Middleware
  - BLE_AI_Control_Fan/STM32_WPAN/App/ble_dbg_conf.h                                     Debug configuration file for BLE Middleware
  - BLE_AI_Control_Fan/STM32_WPAN/App/dis.h                                              Header for service2.c 
  - BLE_AI_Control_Fan/STM32_WPAN/App/dis_app.h                                          Header for service2_app.c 
  - BLE_AI_Control_Fan/STM32_WPAN/App/fc.h                                               Header for service1.c 
  - BLE_AI_Control_Fan/STM32_WPAN/App/fc_app.h                                           Header for service1_app.c 
  - BLE_AI_Control_Fan/STM32_WPAN/Target/bpka.h                                          This file contains the interface of the BLE PKA module
  - BLE_AI_Control_Fan/STM32_WPAN/Target/host_stack_if.h                                 This file contains the interface for the stack tasks 
  - BLE_AI_Control_Fan/STM32_WPAN/Target/ll_sys_if.h                                     Header file for ll_sys_if.c
  - BLE_AI_Control_Fan/System/Config/CRC_Ctrl/crc_ctrl_conf.h                            Configuration Header for CRC controller module 
  - BLE_AI_Control_Fan/System/Config/Debug_GPIO/app_debug.h                              Real Time Debug module application APIs and signal table 
  - BLE_AI_Control_Fan/System/Config/Debug_GPIO/app_debug_signal_def.h                   Real Time Debug module application signal definition 
  - BLE_AI_Control_Fan/System/Config/Debug_GPIO/debug_config.h                           Real Time Debug module general configuration file 
  - BLE_AI_Control_Fan/System/Config/Flash/simple_nvm_arbiter_conf.h                     Configuration header for simple_nvm_arbiter.c module 
  - BLE_AI_Control_Fan/System/Config/Log/log_module.h                                    Configuration Header for log module 
  - BLE_AI_Control_Fan/System/Config/LowPower/app_sys.h                                  Header for app_sys.c 
  - BLE_AI_Control_Fan/System/Config/LowPower/peripheral_init.h                          Header for peripheral init module 
  - BLE_AI_Control_Fan/System/Config/LowPower/user_low_power_config.h                    Header for user_low_power_config.c
  - BLE_AI_Control_Fan/System/Interfaces/hw.h                                            This file contains the interface of STM32 HW drivers
  - BLE_AI_Control_Fan/System/Interfaces/hw_if.h                                         Hardware Interface 
  - BLE_AI_Control_Fan/System/Interfaces/stm32_lpm_if.h                                  Header for stm32_lpm_if.c module (device specific LP management) 
  - BLE_AI_Control_Fan/System/Interfaces/timer_if.h                                      configuration of the timer_if.c instances 
  - BLE_AI_Control_Fan/System/Interfaces/usart_if.h                                      Header file for stm32_adv_trace interface file 
  - BLE_AI_Control_Fan/System/Modules/ble_timer.h                                        This header defines the timer functions used by the BLE stack 
  - BLE_AI_Control_Fan/System/Modules/crc_ctrl.h                                         Header for CRC client manager module 
  - BLE_AI_Control_Fan/System/Modules/dbg_trace.h                                        Header for dbg_trace.c 
  - BLE_AI_Control_Fan/System/Modules/otp.h                                              Header file for One Time Programmable (OTP) area 
  - BLE_AI_Control_Fan/System/Modules/scm.h                                              Header for scm.c module 
  - BLE_AI_Control_Fan/System/Modules/stm_list.h                                         Header file for linked list library
  - BLE_AI_Control_Fan/System/Modules/utilities_common.h                                 Common file to utilities 
  - BLE_AI_Control_Fan/System/Modules/baes/baes.h                                        This file contains the interface of the basic AES software module
  - BLE_AI_Control_Fan/System/Modules/baes/baes_global.h                                 This file contains the internal definitions of the AES software module
  - BLE_AI_Control_Fan/System/Modules/Flash/flash_driver.h                               Header for flash_driver.c module 
  - BLE_AI_Control_Fan/System/Modules/Flash/flash_manager.h                              Header for flash_manager.c module 
  - BLE_AI_Control_Fan/System/Modules/Flash/rf_timing_synchro.h                          Header for rf_timing_synchro.c module 
  - BLE_AI_Control_Fan/System/Modules/Flash/simple_nvm_arbiter.h                         Header for simple_nvm_arbiter.c module 
  - BLE_AI_Control_Fan/System/Modules/Flash/simple_nvm_arbiter_common.h                  Common header of simple_nvm_arbiter.c module 
  - BLE_AI_Control_Fan/System/Modules/MemoryManager/advanced_memory_manager.h            Header for advance_memory_manager.c module 
  - BLE_AI_Control_Fan/System/Modules/MemoryManager/stm32_mm.h                           Header for stm32_mm.c module 
  - BLE_AI_Control_Fan/System/Modules/Nvm/nvm.h                                          This file contains the interface of the NVM manager
  - BLE_AI_Control_Fan/System/Modules/RFControl/rf_antenna_switch.h                      RF related module to handle dedictated GPIOs for antenna switch
  - BLE_AI_Control_Fan/System/Modules/RTDebug/debug_signals.h                            Real Time Debug module System and Link Layer signal definition 
  - BLE_AI_Control_Fan/System/Modules/RTDebug/local_debug_tables.h                       Real Time Debug module System and Link Layer signal 
  - BLE_AI_Control_Fan/System/Modules/RTDebug/RTDebug.h                                  Real Time Debug module API declaration 
  - BLE_AI_Control_Fan/System/Modules/RTDebug/RTDebug_dtb.h                              Real Time Debug module API declaration for DTB usage
  - BLE_AI_Control_Fan/System/Modules/SerialCmdInterpreter/serial_cmd_interpreter.h      Header file for the serial commands interpreter module
  - BLE_AI_Control_Fan/Core/Src/app_entry.c                                              Entry point of the application 
  - BLE_AI_Control_Fan/Core/Src/main.c                                                   Main program body 
  - BLE_AI_Control_Fan/Core/Src/stm32wbaxx_hal_msp.c                                     This file provides code for the MSP Initialization and de-Initialization codes
  - BLE_AI_Control_Fan/Core/Src/stm32wbaxx_it.c                                          Interrupt Service Routines
  - BLE_AI_Control_Fan/Core/Src/system_stm32wbaxx.c                                      CMSIS Cortex-M33 Device Peripheral Access Layer System Source File 
  - BLE_AI_Control_Fan/STM32_WPAN/App/app_ble.c                                          BLE Application 
  - BLE_AI_Control_Fan/STM32_WPAN/App/dis.c                                              service2 definition 
  - BLE_AI_Control_Fan/STM32_WPAN/App/dis_app.c                                          service2_app application definition
  - BLE_AI_Control_Fan/STM32_WPAN/App/fc.c                                               service1 definition
  - BLE_AI_Control_Fan/STM32_WPAN/App/fc_app.c                                           service1_app application definition
  - BLE_AI_Control_Fan/STM32_WPAN/Target/bleplat.c                                       This file implements the platform functions for BLE stack library
  - BLE_AI_Control_Fan/STM32_WPAN/Target/bpka.c                                          This file implements the BLE PKA module
  - BLE_AI_Control_Fan/STM32_WPAN/Target/host_stack_if.c                                 Source file for the stack tasks 
  - BLE_AI_Control_Fan/STM32_WPAN/Target/linklayer_plat.c                                Source file for the linklayer plateform adaptation layer 
  - BLE_AI_Control_Fan/STM32_WPAN/Target/ll_sys_if.c                                     Source file for initiating the system sequencer 
  - BLE_AI_Control_Fan/STM32_WPAN/Target/power_table.c                                   This file contains supported power tables 
  - BLE_AI_Control_Fan/System/Config/CRC_Ctrl/crc_ctrl_conf.c                            Source for CRC client controller module configuration file 
  - BLE_AI_Control_Fan/System/Config/Debug_GPIO/app_debug.c                              Real Time Debug module application side APIs 
  - BLE_AI_Control_Fan/System/Config/Flash/simple_nvm_arbiter_conf.c                     Source file of configuration of NMV aritrer
  - BLE_AI_Control_Fan/System/Config/Log/log_module.c                                    Source file of the log module 
  - BLE_AI_Control_Fan/System/Config/LowPower/user_low_power_config.c                    Low power related user configuration
  - BLE_AI_Control_Fan/System/Config/LowPower/peripheral_init.c                          Source for peripheral init module 
  - BLE_AI_Control_Fan/System/Interfaces/hw_aes.c                                        This file contains the AES driver for STM32WBA 
  - BLE_AI_Control_Fan/System/Interfaces/hw_otp.c                                        This file contains the OTP driver
  - BLE_AI_Control_Fan/System/Interfaces/hw_pka.c                                        This file contains the PKA driver for STM32WBA 
  - BLE_AI_Control_Fan/System/Interfaces/hw_rng.c                                        This file contains the RNG driver for STM32WBA 
  - BLE_AI_Control_Fan/System/Interfaces/pka_p256.c                                      This file is an optional part of the PKA driver for STM32WBA. It is dedicated to the P256 elliptic curve
  - BLE_AI_Control_Fan/System/Interfaces/stm32_lpm_if.c                                  Low layer function to enter/exit low power modes (stop, sleep) 
  - BLE_AI_Control_Fan/System/Interfaces/timer_if.c                                      Configure RTC Alarm, Tick and Calendar manager 
  - BLE_AI_Control_Fan/System/Interfaces/usart_if.c                                      Source file for interfacing the stm32_adv_trace to hardware 
  - BLE_AI_Control_Fan/System/Modules/app_sys.c                                          Application system for STM32WPAN Middleware
  - BLE_AI_Control_Fan/System/Modules/ble_timer.c                                        This module implements the timer core functions 
  - BLE_AI_Control_Fan/System/Modules/crc_ctrl.c                                         Source for CRC client controller module 
  - BLE_AI_Control_Fan/System/Modules/otp.c                                              Source file for One Time Programmable (OTP) area 
  - BLE_AI_Control_Fan/System/Modules/scm.c                                              Functions for the System Clock Manager
  - BLE_AI_Control_Fan/System/Modules/stm_list.c                                         TCircular Linked List Implementation
  - BLE_AI_Control_Fan/System/Modules/baes/baes_cmac.c                                   This file contains the AES CMAC implementation
  - BLE_AI_Control_Fan/System/Modules/baes/baes_ecb.c                                    This file contains the AES ECB functions implementation
  - BLE_AI_Control_Fan/System/Modules/Flash/flash_driver.c                               The Flash Driver module is the interface layer between Flash management modules and HAL Flash drivers
  - BLE_AI_Control_Fan/System/Modules/Flash/flash_manager.c                              The Flash Manager module provides an interface to write raw data from SRAM to FLASH
  - BLE_AI_Control_Fan/System/Modules/Flash/rf_timing_synchro.c                          The RF Timing Synchronization module provides an interface to synchronize the flash processing versus the RF activity to make sure the RF timing is not broken
  - BLE_AI_Control_Fan/System/Modules/Flash/simple_nvm_arbiter.c                         The Simple NVM arbiter module provides an interface to write and/or restore data from SRAM to FLASH with use of NVMs
  - BLE_AI_Control_Fan/System/Modules/MemoryManager/advanced_memory_manager.c            Memory Manager 
  - BLE_AI_Control_Fan/System/Modules/MemoryManager/stm32_mm.c                           Memory Manager 
  - BLE_AI_Control_Fan/System/Modules/Nvm/nvm_emul.c                                     This file implements the RAM version of the NVM manager for STM32WBX. It is made for test purpose
  - BLE_AI_Control_Fan/System/Modules/RFControl/rf_antenna_switch.c                      RF related module to handle dedictated GPIOs for antenna switch
  - BLE_AI_Control_Fan/System/Modules/RTDebug/RTDebug.c                                  Real Time Debug module API definition 
  - BLE_AI_Control_Fan/System/Modules/RTDebug/RTDebug_dtb.c                              Real Time Debug module API definition for DTB usage

## Setup

This example runs on **STM32WBAxx DK board**.

You need hardware to do the same demo:
 * A Fan
 * [Arduino Motor Shield rev3](https://docs.arduino.cc/hardware/motor-shield-rev3/)
 * 4 * [LEDs Bar](https://shop.pimoroni.com/products/blinkt?variant=22408658695) (Not mandatory)
 * 3D Model that you can impress on yourself with the 3D files
 * A phone or a PC

And also software:
 * IDE (CubeIDE or IAR)
 * [NanoEdge AI](https://stm32ai.st.com/nanoedge-ai/) 





## Schematic 


To use the system without the LED and the screen:
1. Connect the Arduino shield to the WBA Development Kit (DK) using the ST Morpho connectors.
2. Attach the fan's positive (+) and negative (-) terminals to the "Motor A" outputs.
3. Connect a 12V power supply to the Vin and GND terminals.

To incorporate the LED bar, or use the LCD screen, look the instructions on the Hotspot.


## Application description

The application contains two modes that can be selected in the `app_conf.h` file.


### **The Data Logger**


The mode allows you to generate your own AI library using NanoEdge AI. It is recommended not to use the pre-existing library, as any discrepancies between your model and the provided model could significantly alter the data input to the AI.

To begin logging data for your AI model:

1. **Open** the firmware project in your preferred Integrated Development Environment (IDE).
2. **Modify** the `app_conf.h` file by setting `DATA_LOGGER` to `1`:

```c
#define DATA_LOGGER 1
```

3. **Save** the changes to the `app_conf.h` file.
4. **Compile** the project to build the firmware with the new configuration.
5. **Flash** the compiled firmware onto the board.

For NanoEdge AI Studio:

1. **Start** a new Anomaly Detection project.
2. **Generate** both regular and abnormal signals, which will be sent via USB with a baud rate of 921,600 bps.

For more information, refer to the [NanoEdge AI documentation](https://wiki.st.com/stm32mcu/wiki/AI:NanoEdge_AI_Studio#Designing_a_relevant_sampling_methodology)



### **The Anomaly detection**

After generating your library, you can proceed to use the main application for anomaly detection.

1. **Open** the firmware project in your preferred Integrated Development Environment (IDE).
2. **Edit** the `app_conf.h` file by setting `DATA_LOGGER` to `0`:

```c
#define DATA_LOGGER 0
```

3. **Save** the changes to the `app_conf.h` file.
4. **Compile** the project to build the firmware with the updated configuration.
5. **Flash** the compiled firmware onto the board.

If you wish to use the LEDs and the screen:

- Ensure the corresponding connections are made and the software is configured to interact with these components:

```c
#define CFG_LED_SUPPORTED                       (1)
#define CFG_LCD_SUPPORTED                       (1)
```

To switch to the shield's power supply:

1. **Change** the jumper JP4 to the `5VVIN` position to use an external power supply.

To use the Web App:

1. **Open** the provided Web App link on a PC or Android smartphone.
2. **Connect** to the Fan Control, which will likely be listed as "FC_XX".
3. **Select** the desired fan speed through the app.
4. **Monitor** for notifications from the app indicating when an anomaly is detected.

This setup allows for remote monitoring and control of the fan, with the added functionality of receiving notifications in case of detected anomalies.

## Troubleshooting

**Caution** : Issues and the pull-requests are **not supported** to submit problems or suggestions related to the software delivered in this repository. The STM32WBA-BLE-AI-Fan-Control example is being delivered as-is, and not necessarily supported by ST.

**For any other question** related to the product, the hardware performance or characteristics, the tools, the environment, you can submit it to the **ST Community** on the STM32 MCUs related [page](https://community.st.com/s/topic/0TO0X000000BSqSWAW/stm32-mcus).
