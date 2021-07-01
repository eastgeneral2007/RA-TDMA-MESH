ABOUT

Drone-RK is an open-source real-time distributed UAV development infrastructure from the Electrical and Computer Engineering Department at Carnegie Mellon University and Faculdade de Engenharia da Universidade do Porto.

This project focuses on the software infrastructure required for self-contained autonomous UAV application development. Drone-RK currently runs on the Parrot AR.Drone 2.0 hardware platform. Drone-RK provides Resource Kernel (RK) extensions to the standard Linux kernel that provide real-time scheduling extensions such that tasks in the system can specify their resource demands such that the operating system can provide timely, guaranteed and controlled access to system resources (CPU, network, sensors and actuators). Drone-RK also includes a network framework - TDMA, routing multi-hop.

The Drone-RK development platform provides APIs for local sensing, control and processing as well as various demonstration applications. In order to support rich autonomous behaviors, the platform provides hooks to incorporate additional hardware components (GPS, digital compasses, ultrasonic ranging, etc)
BUILDING

An ARM compiler is needed. 
We have been developing with the AR.Drone 2 
libdrk.so a shared library that can be used with any user binaries.
hello_world is probably the most useful binary as it will perform a basic library loading, takeoff, and land , move, etc.

TUTORIALS

http://wise.ece.cmu.edu/redmine/projects/drone-rk/wiki/Hello_World
