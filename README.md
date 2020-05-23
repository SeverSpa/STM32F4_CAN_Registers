1. Description:

Two STM32F4 Discovery boards communicating by CAN interface. Each board increment an own variable (as an example) and
sends it by CAN to the other board, which displays its 4 lower bits on user leds. 

2. How to use:

Dowload the whole content of the project, run the CubeMX file C12_CAN_Reg.ioc and generate project for IAR EWARM. 
Open the generated project, Download and Degug (Ctrl+R) in the one board, then Go (F5). 
Invert the identifiers: 0x02 as "Own ID" and 0x01 for "Accepted ID", Download and Degug (Ctrl+R with the first board decoupled) in the second board, then Go (F5). 
In the Live watch field of each board we may see the Tx and Rx variables and the lower bits of the corresponding variable may be seen on user leds. 

3. Software context:

The project was verified using:

-STM32 Cube MX version 5.6.0

-Firmware package STM32Cube FW_F4 V1.25.0

-IAR-EWARM v 8.50.1.

Notice: For other software context, some modifications may be necessary, as the future evolution of these products is unknown.

4. Hardware context:
-Two STM32F4 -Discovery boards, each of then using a CAN transceiver on PB8 and PB9, and two bus wires between the transceivers

5. Youtube classroom: https://www.youtube.com/watch?v=rx85uFiPxu8&t=3148s
