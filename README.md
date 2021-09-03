| Supported Targets | ESP32 |
| ----------------- | ----- |

# MCPWM brushed dc motor control Example

The Objective of this project is to control the DC motor with High Precision. This project have been done for the Line following robot. For the motion of the robot, the DC motor has been used. The goal basically is to precisely control the speed of the motor. Thus, this project gives a brief overview of the precise control using control feedback algorithm. For this task, ESP32 is used as the hardware controller and esp-idf framework is used to flash the program into the ESP32.

* GPIO19 is assigned as the enable/input 1 for motor driver
* GPIO18 is assigned as the enable/input 2 for motor driver
* GPIO34 is assigned as the enable/input 1 for motor encoder
* GPIO35 is assigned as the enable/input 2 for motor encoder

MCPWM documentation: (https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html)

PCNT documentation: (https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html)

Proper documentation of the project: Look for the documentation folder.

# Note: PID feedback control loop is used to control the speed of the DC motor and thus control over the revolutions is achieved through rotary encoder.
