# rm_imu

RoboMaster C board IMU serial-print task.

## Task

Read the onboard BMI088 IMU data through SPI and print the three-axis IMU data to upper-computer serial software through USART1.

## CubeMX Resources

- SPI1: BMI088 communication bus.
- PA4: accelerometer chip select, GPIO output, default high.
- PB0: gyroscope chip select, GPIO output, default high.
- USART1: serial output, 115200-8-N-1.
- PB6: USART1_TX.
- PB7: USART1_RX.
- TIM10_CH1 / PF6: optional BMI088 heater PWM.

## Notes

- BMI088 chip-select pins are active low.
- Keep the SPI baud rate below 10 MHz. With PCLK2 at 84 MHz, SPI prescaler 16 gives 5.25 MHz.
- Do not commit `build/`; regenerate build files locally with CMake/CubeMX.