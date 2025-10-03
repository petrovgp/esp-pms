# PMS Passive Sleep Example

This example demonstrates using the PMS sensor in passive mode, with data being streamed only after the sensor receives a *passive mode read commmand*. 

Sensor will be put into passive mode with a *set mode command* sent over UART. Example will log the state of the sensor that is currently in every 3 seconds. It will read one data frame once the sensor reaches active state, it wil parse the data, print the PM concentration and put the sensor to sleep for 5 seconds. This cycle runs indefinitely.

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
