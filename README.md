## OneDuino
![Arduino](https://img.shields.io/badge/-Arduino/Teensy-00979D?style=flat&logo=arduino&logoColor=white)&nbsp;
![C++](https://img.shields.io/badge/-C++-00599CA?style=flat&logo=C%2B%2B&logoColor=white)&nbsp;
![Maple](https://img.shields.io/badge/-Maple-3A6693?style=flat&logo=maple&logoColor=white)&nbsp;
![Matlab](https://img.shields.io/badge/-Matlab-007ACC?style=flat&logo=matlab&logoColor=white)&nbsp;
![Solidworks](https://img.shields.io/badge/-Solidworks-BD2C22?style=flat&logo=solidworks&logoColor=white)&nbsp;
![KiCad](https://img.shields.io/badge/-KiCad-B68E29?style=flat&logo=kicad&logoColor=white)&nbsp;

### :clipboard: Summary
The goal of this project is to build a one-wheeled robot from scratch. This includes mechanical design of the robot, component selection, electrical design, control system design and embedded programming. The MCU at the hearth of the robot is a Teensy 4.0, packing enough computational power to experiment with optimisation based control strategies. I started this project with two goals in mind: rehearsing a number of important aspects of control theory and the Arduino C/C++ languages and generally improving my embedded programming skills. 

### :construction: Progress
- [x] Model derivation using Euler-Lagrange equations
- [x] 3D design of robot in Solidworks
- [x] Component selection
- [x] LQR control system design
- [x] Circuit design
- [ ] 3D printing parts and assembly
- [ ] Coding
- [ ] Countless hours of debugging untill this thing will work

### :computer: Simulation results
Simulation of the LQR control responsible for balance in the forward direction: with initial conditions different from balance, its target state is at x = 1 in a balanced state. The LQR controller calculates the optimal input for an infinite optimisation horizon as this doesn't require trajectory tracking and corresponding optimisation.

![Forward LQR Control](https://github.com/samvangysegem/onewheel/blob/main/Matlab/ForwardControl_Animation.gif)
 
### :battery: Circuit Design
The circuit is built on a prototyping PCB given the low complexity of the circuit, which wasn't worth the long waiting time when ordering PCBs online. The board features female headers for connecting the Teensy 4.0 and interfacing with all its pins, 2 male header rows for driving the motors and interfacing with their respective encoders and an MPU6050 module. The red-black wires connect to the Cytron MDD3A dual channel DC motor driver board, in addition to a number of male-female headers for control. As the main focus of this project isn't brushed DC motor control, I didn't bother designing the H-bridges and corresponding driving circuits myself (an idea for an upcoming project, albeit a bit more challenging by replacing the brushed DC motors with brushless DC motors).

| PCB Front | PCB Back |
| --------- | -------- |
![](https://github.com/samvangysegem/onewheel/blob/main/Circuit/Front.JPG) | ![](https://github.com/samvangysegem/onewheel/blob/main/Circuit/Back.JPG)

### Mistakes
Short circuit occured on the prototyping PCB during a motor test with battery supply. Upon closer inspection, the isolation of one of the motor wires melted during soldering. Solder from a wire next to it entered the isolation through this hole, nearly making contact with the other wire. Even though no short circuit was measured during testing, the higher current required for driving the motor in combination with slight wire movement (possibly) caused a short circuit between these two wires. As a result, a voltage of 12 V was applied to the GND output of the Teensy, resulting in a fried voltage regulator, IMU (also connected to this GND) in addition to internal damage of the Teensy. 

**Conclusion** 
- When wire isolation melted and unsure whether a short circuit will occur, assume the worst
- Test motor interfaces first, without Teensy

