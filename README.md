# STM32 Rover Motor Control

This project is an STM32-based motor control firmware for a 4-wheel rover. It features PWM DC motor control, quadrature encoder reading, basic kinematics calculation, and **Feetech STS3215 Serial Bus Servo control** for steering.

## Hardware Specifications
- **Microcontroller**: STM32F446RET6 (NUCLEO-F446RE)
- **Motor Driver**: Cytron MDD3A (4-channel, 2-PWM mode)
- **Drive Motors**: 4 x SPG30E-GR131 (12V DC Geared Motor)
  - Gear Ratio: 131:1 / Ticks per Revolution: 6812.0
- **Steering Servos**: 4 x Feetech STS3215 (Serial Bus Servo)
  - ID 1~4, Daisy-chained
  - 1Mbps Single-Wire Half-Duplex Communication

## Features
- **Drive Motor Control**: Forward, reverse, and stop using hardware PWM.
- **Encoder Reading**: Hardware timer-based x4 quadrature decoding.
- **Servo Motor Control (NEW)**:
  - Custom UART protocol implementation without HAL overhead.
  - **Single-Wire Half-Duplex (HDSEL) mode**: Direct connection without external diode/adapter board!
  - Sync-write multiple servos simultaneously.
- **Interactive Terminal Interface**: Control servo angles in real-time via Serial Monitor.

## Terminal Commands (115200 bps, LF or CRLF)
| Command | Description |
| :--- | :--- |
| `PING` | Check connection status for all 4 servos |
| `T <a1> <a2> <a3> <a4>` | Move servos to target angles (e.g., `T 45 -45 45 -45`) |
| `P` | Print current angles of all servos |
| `ON` / `OFF` | Enable or disable servo torque |

## Pin Configuration (Wiring Guide)

### 1. Drive Motors (Cytron MDD3A)
| Motor | STM32 Pin | Timer/Channel | Function |
| :--- | :--- | :--- | :--- |
| **M1** | PA5 / PB3 | `TIM2_CH1` / `CH2` | Motor 1 Forward / Reverse |
| **M2** | PB14 / PB15 | `TIM12_CH1` / `CH2`| Motor 2 Forward / Reverse |
| **M3** | PA6 / PA7 | `TIM3_CH1` / `CH2` | Motor 3 Forward / Reverse |
| **M4** | PB0 / PB1 | `TIM3_CH3` / `CH2` | Motor 4 Forward / Reverse |

### 2. Encoders (Quadrature x4)
| Encoder | STM32 Pin | Timer | Note |
| :--- | :--- | :--- | :--- |
| **Enc 1** | PA8 / PA9 | `TIM1` | 16-bit Timer |
| **Enc 2** | PB6 / PB7 | `TIM4` | 16-bit Timer |
| **Enc 3** | PA0 / PA1 | `TIM5` | **32-bit Timer** |
| **Enc 4** | PC6 / PC7 | `TIM8` | 16-bit Timer |

### 3. Steering Servos (Feetech STS3215) & Power
> **Direct Single-Wire Connection** (No external adapter needed)

| STM32 Pin | Target Pin | Note |
| :--- | :--- | :--- |
| **PB10** (`USART3_TX`) | Servo **BUS (Data)** | Single-Wire HDSEL mode (Open-Drain). Connect directly to servo signal wire. |
| **GND** | 12V Power **GND** | **MUST be connected to Common Ground!** |
| **Not Connected** | Servo **VCC** | Connect servo VCC directly to external 12V supply (+). |

### 4. Debug & Terminal Interface
| STM32 Pin | Target Pin | Note |
| :--- | :--- | :--- |
| **PA2** (`USART2_TX`) | ST-LINK VCP | 115200 bps |
| **PA3** (`USART2_RX`) | ST-LINK VCP | 115200 bps, Terminal Command Input |

## How to Build
This project uses a standard Makefile. 
```bash
make clean
make -j4
```
Flash the resulting `build/rover_motor.elf` or `.hex` file using STM32CubeIDE or STM32CubeProgrammer.
