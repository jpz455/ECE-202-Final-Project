# ARM Cortex-M4 Elevator Controller (STM32L476)

This project implements a simulated **4-floor elevator system** using the STM32L476 microcontroller programmed entirely in **ARM Cortex-M4 assembly**. The system is controlled by **push buttons** and a **keypad**, and uses **stepper motors**, **LED indicators**, and a **seven-segment display (SSD)** to simulate elevator floor transitions and user feedback.

##  Features

- **Keypad Floor Selection** (PA0, PA1, PA4, PA6 inputs with PB4, PB9-PB11 outputs)
- **Physical Push Button Controls** for direct floor access (PB0, PB1, PB5, PB8)
- **Stepper Motor Control** to simulate elevator movement
- **LED Indicators** (PC0–PC3) for floor status
- **Seven-Segment Display** output for current floor
- **Door Opening and Closing Simulation**
- **Tera Term UART Output** for event feedback (floor reached, door status, etc.)
- **Reset Button Support** (PC10)

##  File Structure

main.s # Main program: system initialization, button/keypad handling, motor logic
core_cm4_constants.s # ARM Cortex-M4 specific register definitions (included)
stm32l476xx_constants.s # STM32-specific register and peripheral addresses (included)


##  Key Concepts

- **SysTick Timer**: For timed delays and smooth motor stepping
- **GPIO Configuration**: Proper bitmasking and manipulation for input/output modes
- **NVIC & Interrupts**: Setup for system-level timing and reset handling
- **Modular Movement Logic**: Elevator transitions using forward/backward step sequences
- **Angle-Based Floor Calculation**: Each floor corresponds to a multiple of a base motor angle
- **Feedback via UART**: User receives text updates in Tera Term via USART2

## How to Use

1. **Assemble and Flash**:
   - Assemble `main.s` using Keil uVision, STM32CubeIDE, or an ARM-compatible assembler.
   - Flash the binary to an STM32L476-based board.

2. **Wiring Setup**:
   - Connect LEDs to PC0–PC3.
   - Connect SSD segments to PC4–PC7.
   - Connect stepper motor control pins to PB2, PB3, PB6, PB7, PB12–PB15.
   - Connect keypad inputs to PA0, PA1, PA4, PA6.
   - Connect keypad outputs to PB4, PB9–PB11.
   - Setup UART on USART2 for terminal output (Tera Term recommended).

3. **Run Simulation**:
   - Use the keypad or push buttons to select a floor.
   - Watch the stepper motor simulate elevator motion.
   - View status updates on Tera Term and LEDs.

## Assembly Highlights

- Each floor has a specific motor angle:
  - Floor 1: 0
  - Floor 2: 514
  - Floor 3: 1028
  - Floor 4: 1542

- Door opening and closing is handled with a precise sequence of motor steps.
- UART messages (e.g., `"Floor 1 Called"`, `"Arrived"`, `"Door Opening"`) help in debugging and simulation.

## Dependencies

- STM32L476 Nucleo or similar development board
- External stepper motor driver (ULN2003 or similar)
- 4x4 keypad
- LEDs and SSD
- Tera Term (for serial terminal output)


