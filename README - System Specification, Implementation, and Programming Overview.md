Secure Low-Power Bus Encoder & Attack-Resistant Architecture | Final Project 
• Architected and implemented a novel Bus Encoding algorithm on Xilinx Artix-7 FPGA to minimize switching activity and reduce dynamic power dissipation on high-capacitance off-chip buses.. 
• Developed a Hardware-Software Co-design verification environment, interfacing a Python-based controller with the FPGA via UART for real-time performance validation. 
• Conducted rigorous power analysis using hands – on hardware experiments and Vivado Power Analyzer,demonstrating a measurable reduction in Hamming Distance and switching noise. 
• Optimized RTL code for resource utilization (LUTs/Flip-Flops) and achieved successful timing closure at target operating frequencies. 
System Architecture
Microcontroller: ADuC841 (High-Performance 8052 Core).

Language: Embedded C.

IDE: Keil uVision.

Key Modules:

main.c: Core system logic and task scheduling.

bus_encoder.c: Implementation of the encoding algorithms.

peripherals.c: Hardware abstraction layer for UART, Timers, and I/O initialization.

header.h: Global definitions, macros, and function prototypes.

Implementation Details
The firmware is structured to ensure efficient real-time performance:

Hardware Initialization: Configures the internal clock, UART for serial communication, and GPIO pins.

Data Acquisition: Reads input signals from the bus or sensors.

Encoding Logic: Processes the raw data into an encoded format suitable for transmission.

Transmission: Sends the processed data via UART/Bus interface with parity and error control.

Flashing and Execution Instructions (How to Burn the Code)
To run this project on your hardware, follow these steps:

1. Prerequisites
Install/Open Keil uVision IDE.


2. Building the Project
Open the project file: Input + Transmitter.uvproj in Keil.

Click on Build/Rebuild (F7) to compile the source files.

3. Flashing (Burning) the Microcontroller
Connect the ADuC841 to your computer via the serial interface.

Set the microcontroller to Download Mode (by pushing the reset and load buttons simultaneously then relese reset button while pusjing the load button).

Open the keli software.

Click load to flash the firmware onto the chip.

Reset the board to start execution.

