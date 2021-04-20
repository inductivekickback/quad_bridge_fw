Firmware for the Nordic Semiconductor nRF51 SOC that allows it to control a Syma X4 (218 BKT) quadcopter using an SNES gamepad or via Bluetooth Low Energy (BLE) with my [QuadBridge app](https://github.com/inductivekickback/QuadBridge) for Android. More information about the project is [located here](http://inductivekickback.blogspot.com/2015/11/ble-to-shockburst-bridge-for-syma-x4.html).

## About
The Syma X4 quadcopter uses Nordic's ShockBurst RF protocol. This protocol is supported by all contemporary Nordic devices because it uses the same radio as BLE. Furthermore, Nordic's BLE stack allows the application to access the radio whenever the radio is idle so it's relatively easy to build a device that acts as a bridge between BLE and ShockBurst.

If BLE is not required then the quadcopter can also be driven using a Trenro SNES gamepad. These gamepads are inexpensive and easy to work with.

## Usage
The project is configured for use with the [nRF51-DK](http://www.digikey.com/product-detail/en/NRF51-DK/1490-1038-ND/5022449) and [nRF51 SDK v8.0](http://developer.nordicsemi.com/nRF51_SDK/nRF51_SDK_v8.x.x/). Note that the [S110 SoftDevice](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.110.sds.v1.0.0%2Fs110.html&cp=2_7_0) is also required.

Place the project into "{SDK ROOT}/examples/ble_peripheral/" to compile it. The firmware will send BLE advertisements with the Nordic UART Service UUID and "SYMA_218" as the name.

If the gamepad is used then its VCC and GND pins should be plugged into VDD (NOT 5V) and GND on the nRF51-DK. Additionally, the gamepad's LATCH pin should be plugged in to P01, CLOCK should be plugged in to P02, and DATA should be plugged in to P03. The button mapping is defined in [gamepad_ctl.h](https://github.com/inductivekickback/quad_bridge_fw/blob/master/gamepad_ctl.h).
