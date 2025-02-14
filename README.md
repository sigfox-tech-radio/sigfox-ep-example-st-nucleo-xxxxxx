
# Sigfox End-Point library implementation example on Nucleo board

## Description

This repository provides an implementation example of the Sigfox End-Point Library on ST-Microelectronics Nucleo boards and various radio shields. The library, addons, and drivers are embedded as submodules to facilitate compatibility management between dependencies.

### Supported Nucleo boards

| **Reference** | **MCU** | `cmake_mcu_board` | **Limitations** |
|:---:|:---:|:---:|:---:|
| [Nucleo-L053R8](https://www.st.com/en/evaluation-tools/nucleo-l053r8.html) | STM32L053R8T6 | `NucleoL053R8` | None |
| [Nucleo-WL33CC1](https://www.st.com/en/evaluation-tools/nucleo-wl33cc1.html) | STM32WL33CCV6 | `NucleoWL33CC1` | None |

### Supported radio shields

| **Reference** | **Radio chip** | `cmake_radio_shield` | **Limitations** | **RSA reports**|
|:---:|:---:|:---:|:---:|:---:|
| [STEVAL-FKI868V2](https://www.st.com/en/evaluation-tools/steval-fki868v2.html) | S2LP | `steval-fki868v2` | Band 868 only  | [RC1_100](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/wiki/rfp/steval-fki868v2_RC1_100.pdf), [RC1_100](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/wiki/rfp/steval-fki868v2_RC1_100.pdf), |
| [LR1110MB1DIS](https://www.semtech.fr/products/wireless-rf/lora-edge/lr1110mb1lbks) | LR1110 | `lr1110mb1dis` | None | [RC1_100](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/wiki/rfp/lr1110mb1dis_RC1_100.pdf), [RC1_600](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/wiki/rfp/lr1110mb1dis_RC1_600.pdf), [RC2](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/wiki/rfp/lr1110mb1dis_RC2.pdf), |
| [LR1110MB1DJS](https://www.semtech.fr/products/wireless-rf/lora-edge/lr1110dvk1tcks) *(shield not sold alone)* | LR1110 | `lr1110mb1djs` | None | |
| [LR1121MB1DIS](https://www.semtech.fr/products/wireless-rf/lora-connect/lr1121dvk1tcks) *(shield not sold alone)* | LR1121 | `lr1121mb1dis` | None | [RC1_100](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/wiki/rfp/lr1121mb1dis_RC1_100.pdf), [RC1_600](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/wiki/rfp/lr1121mb1dis_RC1_600.pdf), [RC2](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/wiki/rfp/lr1121mb1dis_RC2.pdf) |
| [SX1261MB1BAS](https://www.semtech.com/products/wireless-rf/lora-connect/sx1261dvk1bas) *(shield not sold alone)* | SX1261 | `sx1261mb1bas` | Band 868 only  | [RC1_100](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/wiki/rfp/sx1261mb1bas_RC1_100.pdf), [RC1_600](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/wiki/rfp/sx1261mb1bas_RC1_600.pdf)  |
| [SX1261MB2BAS](https://www.semtech.fr/products/wireless-rf/lora-connect/sx1261mb2bas) | SX1261 | `sx1261mb2bas` | Band 868 only  | |
| [Nucleo-WL33CC1](https://www.st.com/en/evaluation-tools/nucleo-wl33cc1.html) | STM32WL33CCV6 | | None | [RC1_100](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/wiki/rfp/nucleo-wl33cc1_RC1_100.pdf), [RC1_600](https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/wiki/rfp/nucleo-wl33cc1_RC1_600.pdf)

## Applications

* The `button` defines a simple application where a Sigfox message is sent when the user button is pressed.

* The `modem` defines a more complex application to send Sigfox messages and execute the RF & Protocol test modes with a set of AT commands.

Other applications will come soon, such as a full AT command modem exposing all Sigfox features and test modes.

## Architecture

From lower to upper layers, the project is structured as follows:

* `drivers` : hardware drivers split in 5 categories:
    * `cmsis` : **MCU core** drivers.
    * `peripherals` : MCU peripherals abstraction layer (**MCAL**) and specific MCU drivers (**STM32 HAL drivers** embedded as submodules).
    * `components` : External components drivers (**radio chips drivers** embedded as submodules).
    * `shields` : Radio shields drivers (**HW_API** implementation required by the RF_API).
    * `utils` : Pure software utility functions such as **AES encryption**.
* `middleware` : this folder contains all the **Sigfox** submodules, the **MCU API** implementation and the **AT commands** parser.
* `application` : Main application.

<p align="center">
<img src="https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx/wiki/images/sigfox_ep_example_st_nucleo_xxxxxx_architecture.drawio.png" width="600"/>
</p>

## Compilation flags for optimization

This example inherits all the [Sigfox End-Point library flags](https://github.com/sigfox-tech-radio/sigfox-ep-lib/wiki/compilation-flags-for-optimization) and can be optimized accordingly.

Some dependencies require one or multiple flags to be activated, please refer to the README of the corresponding repository.

## Getting started

### Cloning the repository and its submodules

```bash
git clone https://github.com/sigfox-tech-radio/sigfox-ep-example-st-nucleo-xxxxxx.git
cd sigfox-ep-example-st-nucleo-xxxxxx
git submodule update --init
```

### How to build the project

#### Command line 

Command line requires tools to be installed :

* Linux:

    * make : ```sudo apt-get install build-essential```
    * cmake (min 3.21) : ```sudo apt-get install cmake```
    * [Arm GNU Toolchain arm-none-eabi-gcc](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain).
 
* Windows:

    * [make](https://gnuwin32.sourceforge.net/packages/make.htm) add bin installation folder to PATH environment variable.
    * [cmake(min 3.21)](https://cmake.org/) and check "add CMake to the system PATH for all user".
    * [Arm GNU Toolchain arm-none-eabi-gcc](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain) and check "add to PATH to environment variable" during installation.

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE="Release" \
      -DCMAKE_TOOLCHAIN_FILE="cmake/toolchain-arm-none-eabi.cmake" \
      -DTOOLCHAIN_PATH="<REPLACE BY ROOT TOOLCHAIN PATH PREVIOUSLY INSTALLED>" \
      -DMCU_BOARD="<cmake_mcu_board>" \
      -DRADIO_SHIELD="<cmake_radio_shield>" \
      -DSIGFOX_EP_RC1_ZONE=ON \
      -DSIGFOX_EP_RC2_ZONE=ON \
      -DSIGFOX_EP_RC3_LBT_ZONE=ON \
      -DSIGFOX_EP_RC3_LDC_ZONE=ON \
      -DSIGFOX_EP_RC4_ZONE=ON \
      -DSIGFOX_EP_RC5_ZONE=ON \
      -DSIGFOX_EP_RC6_ZONE=ON \
      -DSIGFOX_EP_RC7_ZONE=ON \
      -DSIGFOX_EP_APPLICATION_MESSAGES=ON \
      -DSIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE=ON \
      -DSIGFOX_EP_BIDIRECTIONAL=ON \
      -DSIGFOX_EP_ASYNCHRONOUS=ON \
      -DSIGFOX_EP_LOW_LEVEL_OPEN_CLOSE=ON \
      -DSIGFOX_EP_REGULATORY=ON \
      -DSIGFOX_EP_LATENCY_COMPENSATION=ON \
      -DSIGFOX_EP_SINGLE_FRAME=OFF \
      -DSIGFOX_EP_UL_BIT_RATE_BPS=OFF \
      -DSIGFOX_EP_TX_POWER_DBM_EIRP=OFF \
      -DSIGFOX_EP_T_IFU_MS=OFF \
      -DSIGFOX_EP_T_CONF_MS=OFF \
      -DSIGFOX_EP_UL_PAYLOAD_SIZE=OFF \
      -DSIGFOX_EP_AES_HW=ON \
      -DSIGFOX_EP_CRC_HW=OFF \
      -DSIGFOX_EP_MESSAGE_COUNTER_ROLLOVER=OFF \
      -DSIGFOX_EP_PARAMETERS_CHECK=ON \
      -DSIGFOX_EP_CERTIFICATION=ON \
      -DSIGFOX_EP_PUBLIC_KEY_CAPABLE=ON \
      -DSIGFOX_EP_VERBOSE=ON \
      -DSIGFOX_EP_ERROR_CODES=ON \
      -DSIGFOX_EP_ERROR_STACK=32 \
      -G "Unix Makefiles" ..
make all
```

All binary files produced are available in `build/application/"application Name"/` folder. 

### How to program Nucleo Board

If nucleo board is still in STLink you can directly click and drop the `.bin` previous produced file in the mounted disk by Nucleo board. 

### Replace Sigfox credential 

By default this project integrate Sigfox test credentials typically used for RF&Protocol test under RSA: 

```
EP ID  : FEDCBA98
Authentication KEY : 0123456789ABCDEF0123456789ACBDEF
```

EP ID and Authentication KEY are stored respectively in memory Page 511 at 0x0800FF80 and 0x0800FF84 addresses. It is possible to modify them by directly patching the output ```<application>.bin``` file or by modifying **sigfoxID** and **sigfoxKEY** variables in ```middleware/sigfox/mcu_api/src/mcu_api.c``` file.
