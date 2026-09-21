# STM32 Rover Motor Control

This project is an STM32-based motor control firmware for a 4-wheel rover. It features PWM motor control, quadrature encoder reading, and basic kinematics calculation (RPM, angular velocity, and linear velocity) for each wheel.

## Hardware Specifications
- **Microcontroller**: STM32F446RET6 (NUCLEO-F446RE)
- **Motor Driver**: Cytron MDD3A (2-PWM mode)
- **Motors**: 4 x SPG30E-GR131 (12V 37MM DC Geared Motor)
- **Encoders**: Magnetic Quadrature Encoders (Hall Effect)
  - Gear Ratio: 131:1
  - Encoder PPR: 13 (52 CPR with 4x decoding)
  - Ticks per Revolution: 6812.0
- **Wheel Radius**: 48mm (96mm Diameter)

## Features
- **Motor Control**: Forward, reverse, and stop using PWM (TIM2, TIM3, TIM12).
- **Encoder Reading**: Hardware timer-based encoder reading (TIM1, TIM4, TIM5, TIM8).
- **Kinematics Calculation**:
  - Calculates wheel RPM
  - Calculates angular velocity (rad/s)
  - Calculates linear velocity (m/s)
- **UART Debugging**: Prints detailed status every 100ms over USART2 (115200 bps).
- **Automated Test Sequence**: Built-in routine to test each motor forward and backward automatically on startup.

## Requirements
- STM32CubeIDE (or compatible ARM GCC toolchain)
- ST-LINK Utility or OpenOCD for flashing

## How to Build and Run
1. Clone the repository.
2. Open the project in STM32CubeIDE using the `.project` and `.cproject` files.
3. Build the project (`Project -> Build All`).
4. Connect the STM32 board via ST-LINK.
5. Flash the firmware to the board.
6. Open a Serial Monitor (e.g., TeraTerm, PuTTY) with `115200 bps, 8 data bits, no parity, 1 stop bit` to view the debug output.

## Pin Configuration
*(Refer to `rover_motor.ioc` in STM32CubeMX for exact pinout)*
- **USART2 TX/RX**: PA2 / PA3 (Connected to ST-LINK VCP)
- **Encoders**: TIM1, TIM4, TIM5, TIM8
- **PWM Outputs**: TIM2, TIM3, TIM12
