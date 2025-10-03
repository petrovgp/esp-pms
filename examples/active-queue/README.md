# PMS Active Queue Example

This example demonstrates using the PMS sensor in active mode, with constat stream of data in either stable submode (every ~2.3s) or fast submode (every ~200 - 800ms). Submodes are automatic and depend on the environment the sensor is in.

Data is being sent to a queue, in which the example processes the data if the event type indicates received data over UART, and prints the PM concentration levels.

This example can be done without connecting any control pins (SET and/or RESET pins).

## Recommended hardware setup

 ```
    -------------                 -------------
    |           |----> 5V         |           |
    |           |----> GND        |           |
    |           |                 |           |
    |           | SET       GPIO9 |           |
    |           |---<>---+---<>---|           |
    |           | RX           TX |           |
    |  PMSX003  |----<---+---<----|   ESP32   |
    |           | TX          RX  |           |
    |           |---->---+--->----|           |
    |           | RESET    GPIO10 |           |
    |           |---<>---+---<>---|           |
    |           |                 |           |
    |           |                 |           |
    -------------                 -------------
```
