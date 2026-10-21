# Design and Control of a 2D Cable-Driven Parallel Robot

## 1. Project Overview

This project focuses on the design, simulation, and control of a low-cost 2D Cable-Driven Parallel Robot (CDPR).

The robot uses four cables to control the position of a moving platform in a planar workspace.

## 2. System Specifications

- Robot type: Planar Cable-Driven Parallel Robot
- Degrees of freedom: 2 (X-Y)
- Number of cables: 4
- Controller: ESP32
- Motor drivers: DRV8825
- Tension measurement: Load cells and HX711

## 3. Hardware

- ESP32 development board
- NEMA 17 stepper motors
- DRV8825 motor drivers
- Load cells and HX711 modules
- 2020 aluminum extrusion frame
- Custom-designed cable winches

## 4. Control and Simulation

- Forward and inverse kinematics
- Workspace analysis
- Cable tension distribution using Quadratic Programming (QP)
- Closed-loop cable tension control using load cells
- Motor control using ESP32

## 5. Project Structure

- `docs/`: System diagram
- `matlab/`: MATLAB simulation and analysis
- `firmware/`: ESP32 firmware
- `mechanical/`: Mechanical design files
- `results/`: Experimental results and images

## 6. Results
<img width="2048" height="1152" alt="CDPR" src="https://github.com/user-attachments/assets/f7a7efbd-7a32-48a6-9b8e-b57b0e60bad9" />
<img width="1537" height="857" alt="CDPR_3D" src="https://github.com/user-attachments/assets/f5e72a13-9be1-4c9e-b3b1-0d66fc64b07d" />
https://github.com/user-attachments/assets/3d3e3154-1d26-4343-8fc7-3aaefd6ffb5f
https://github.com/user-attachments/assets/fea0d65c-c6a9-494e-9823-e97bd7a5128
<img width="711" height="454" alt="Results_of_coordinates_Experiment" src="https://github.com/user-attachments/assets/5f26468d-1df4-46d7-a0be-541b73222f21" />
<img width="520" height="398" alt="The compare between the desired position and the real position" src="https://github.com/user-attachments/assets/aa16c0b3-213e-4b44-bd16-137b6ab7bb5b" />
<img width="2779" height="1242" alt="ket_qua_square" src="https://github.com/user-attachments/assets/1c4c854c-e5a2-4e27-bd64-f3b985972302" />
<img width="2779" height="1242" alt="ket_qua_circle" src="https://github.com/user-attachments/assets/95902ea1-16b3-4fd9-8b63-b552cb76482d" />

## 7. Tools

MATLAB | SolidWorks | ESP32 | C/C++ | Python


## 8. Author

Viet Anh
