# STM32WB55ADV-Tester

[![Thumbnail - click to watch demo](https://img.youtube.com/vi/a61Kv0bly2U/hqdefault.jpg)](https://www.youtube.com/embed/a61Kv0bly2U)

* The STM32WB55ADV-Tester project uses the NUCLEO-WB55RG board to simulate multiple Bluetooth devices nearby.

  Every 0.2s, the STM32WB55 increments its public Bluetooth address

  00:00:00:00:00:01

  00:00:00:00:00:02

  00:00:00:00:00:03

  ...

  You can use the STM32WB55 to test the Bluetooth scanner side.

## Hardware Needed

  * One [NUCLEO-WB55RG](https://www.st.com/en/evaluation-tools/nucleo-wb55rg.html)
  * One Bluetooth scanner such as smartphones or laptops

## Software Needed

  * IDE, choose your preferred one:

    * [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) - please use high code optimization level

    * [IAR EWARM](https://www.iar.com/products/architectures/arm/iar-embedded-workbench-for-arm/)

    * [Keil MDK-ARM](https://developer.arm.com/Tools%20and%20Software/Keil%20MDK) - please use high code optimization level

  * Bluetooth scanner side
    * Bluetooth smartphone apps like [ST BLE Toolbox](https://play.google.com/store/apps/details?id=com.st.dit.stbletoolbox&hl=en_US&pli=1)
    * Another NUCLEO-WB55RG with P2P Client app
    * [STM32CubeMon-RF](https://www.st.com/en/development-tools/stm32cubemonrf.html)

## User's Guide

1) Build this STM32WB55ADV-Tester project using one of the IDEs.

2) Flash this code on the NUCLEO-WB55RG board and check the LEDs.
    LED1: Briefly turns off when stopping BLE advertising.
          Turns on after reconfiguring BLE public address and ADV packet.
    LED2: Toggles at the end of Radio activities.

3) Use the Bluetooth scanning app to test like the ST BLE Toolbox mobile app

## Troubleshooting

**Caution** : Issues and the pull-requests are **not supported** to submit problems or suggestions related to the software delivered in this repository. The motor parameters are not optimized in this firmware. The STM32WB55ADV-Tester example is being delivered as-is, and not necessarily supported by ST.



**For any other question** related to the product, the hardware performance or characteristics, the tools, the environment, you can submit it to the **ST Community** on the STM32 MCUs related [page](https://community.st.com/s/topic/0TO0X000000BSqSWAW/stm32-mcus).