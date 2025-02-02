# Line Follower Robot using STM32F411

This project is a high-speed, high-accuracy line follower robot based on the STM32F411 microcontroller. The robot utilizes an [LSA08](https://robu.in/wp-content/uploads/2015/11/LSA08-Users-Manual-Jun12.pdf) sensor for line detection via UART communication and is driven by two [N20 300RPM](https://robu.in/product/n20-12v-300-rpm-micro-metal-gear-motor/?gad_source=1&gclid=Cj0KCQiAqL28BhCrARIsACYJvkfTXdTgoVehS03U7q7NQAJsIl6GadeOPt4iYMQUSQA3hKH54x4fSKAaAoOtEALw_wcB) motors controlled through an [MDD3A](https://docs.google.com/document/d/1ax3gSo0srTzoSr2bo8ETLym0OqrYjlk_JCZK4kxtfXg/edit?tab=t.0) motor driver. A PID control algorithm is implemented to achieve precise and efficient line tracking.

## Control Algorithm:
PID (Proportional-Integral-Derivative) for High-Speed and Accurate Line Following

## Hardware Components
1. **STM32F411 Development Board** - Controls the entire system
2. **LSA08 Line Sensor Module** - Detects the line and provides positional data over UART
3. **N20 300RPM Motors** - Provides motion to the robot
4. **MDD3A Motor Driver** - Controls the motor speed and direction
5. **Power Supply** - 3S 11.1V Lipo battery
6. **Custom designed PCB**

## Software Implementation
The firmware is written in C using the STM32 HAL (Hardware Abstraction Layer) libraries. The main features include:

- **UART Communication:** To receive data from the LSA08 sensor
- **PID Control:** Fine-tuned for optimal performance to minimize error and achieve stable tracking
- **PWM Motor Control:** Speed adjustments based on sensor feedback

## Future Improvements
- Adding an IMU sensor for better stability
- Wireless communication for remote monitoring

## Acknowledgments
Special thanks to [Team Robomanipal](http://www.robomanipal.com/), MIT, Manipal for their valuable support and resources.
