# Dynamixel AX-12A Robot Actuator

[![AX-12A](https://static.generation-robots.com/3053/dynamixel-ax-12a-servomotor.jpg)](http://www.trossenrobotics.com/images/productdownloads/AX-12(English).pdf)

## How to install the library
- Download zip file
- Extract zip
- Rename the folder : AX-12A-servo-library-master --> AX-12A-servo-library
- Put the folder into your library folder (usually Documents/Arduino/libraries)

## How to use the library
To use the library, include header file :
```
#include "DynamixelSerial.h"
```

The library is provided with 4 Arduino examples :
- **Blink** : simpliest example to blink the built-in LED
- **Move** : example showing how to set a goal position
- **EndlessTurn** : example of endlessTurn mode (wheel mode)
- **readRegister** : debug AX-12A by printing all its registers

You can change AX-12A settings by connecting your servo to a PC via [USB2Dynamixel][USB2DXL].

To do so, you also have to install [Dynamixel Wizard][RoboPlus] included in the RoboPlus software suite.

Follow [these instructions][Dynamixel Wizard] to connect and access the AX-12A settings.

## Baud rate tests

We tested the library on differents boards at the most common baud rates.

Some boards cannot reach the desired baud rate :

| Address 0x04 | Baud rate | Feather M0 | Feather 32u4 | nRF52 | STM32L4 |
| ------------ | --------- | ---------- | ------------ | ----- | ------- | 
| 1	           | 1000000   | ok         | ok           | ok    | fail    |
| 3            | 500000    | ok         | ok           | fail  | ok      |
| 4            | 400000    | ok         | fail         | fail  | ok      |
| 7            | 250000    | ok         | ok           | ok    | ok      |
| 9            | 200000    | ok         | ok           | fail  | ok      |
| 16           | 115200    | ok         | ok           | ok    | ok      |
| 34           | 57600     | ok         | fail         | ok    | ok      |
| 103          | 19200     | ok         | ok           | ok    | ok      |
| 207          | 9600      | ok         | ok           | ok    | ok      |



[USB2DXL]: <http://support.robotis.com/en/product/auxdevice/interface/usb2dxl_manual.htm>
[RoboPlus]: <http://en.robotis.com/BlueAD/board.php?bbs_id=downloads&scate=SOFTWARE>
[Dynamixel Wizard]: <http://support.robotis.com/en/software/roboplus/dynamixel_monitor.htm>
