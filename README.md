# RC-Plane
An Remote Controlled Air Plane using Audurino-NANO as transmitter and STM32NUCLEO as receiver.
the diagram below show the system :  
![System Diagram](https://github.com/user-attachments/assets/7faf200a-7a93-4734-a8ca-663de5d3313a)

The system mainly consist with 2 parts:
-  The transmitter processes the joystick input and sends the data out using radio frequency
-  The receiver is responsible for receiving the data at a defined frequency

Library used:
[Arduino-STM32-nRF24L01](https://github.com/nopnop2002/Arduino-STM32-nRF24L01)
