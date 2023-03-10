# Control Board IoT DevKit

The IoT DevKit Control Board is a development board for studying cases or professional applications. The board uses a powerful dual-core microcontroller with WiFi, Classic Bluetooth, and Low-Energy Bluetooth (BLE). There are many possibilities of applications that go from the smart home to industrial environment monitor.
## Features

* 4 Relays for output up to 10A.
* 4 Digital inputs (12V) and AC 127/220V.   
* 1 Analog output up to 10V.
* 1 Dimmer AC output
* 2 NTC inputs
* WiFi and Bluetooth
* IR external board conector or expansion board.

## Applications

* Environment monitor
* Smart Home Control
* Motor speed control
* ThingsBoard application
* Thermostat

## Block Diagram

![block-diagram](imagens/block-diagram.png?raw=true "Title")

## Board main components

![control-board](imagens/des-control-board.png?raw=true "Title")

OBS: The max power input is 12V!

## Examples

* [ESP - Rain Maker](examples/controlboard-rainmaker)
* [ThingsBoard](examples/controlboard-rainmaker)

## Get Started

1. All examples were developed using  [Espressif IDE](https://dl.espressif.com/dl/esp-idf/?idf=4.4),  but you can choose the IDE you like.
2. clone this repositoty `git clone https://github.com/IndustriasWilliam/control-board-iot-devkit.git`
3. Open the IDE and go to "File" > Open projects from file system... 