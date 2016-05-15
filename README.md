# Not yet Functional Work In Progress.
Using two Winbond 16MB flash chips, each in 4 bit QPI mode
Running in parallel for 8 bit access each command.


# BiParallelFlash :: Using this pin configuration to match the board Frank Made
Compatible to SerialFlash by Paul Stoffregen

Winbond SPI Flash only.
T_PIN= 2; //PTD0 Flash_1 Pin 5 // First Par Flash Start
T_PIN=14; //PTD1 Flash_1 Pin 2
T_PIN= 7; //PTD2 Flash_1 Pin 3
T_PIN= 8; //PTD3 Flash_1 Pin 7
T_PIN= 6; //PTD4 Flash_2 Pin 5 // Second Par Flash Start
T_PIN=20; //PTD5 Flash_2 Pin 2
T_PIN=21; //PTD6 Flash_2 Pin 3
T_PIN= 5; //PTD7 Flash_2 Pin 7
 
T_PIN=23; //PTC2 FlashPin 6      // wire FROM teensy PIN 17
T_PIN=15; //PTC0 Flash1 Pin 1
T_PIN=22; //PTC1 Flash2 Pin 1    // wire FROM teensy PIN 16


# ParallelFlash -------------------

Compatible to SerialFlash by Paul Stoffregen
Winbond SPI Flash only.

Connections:

Pin 2 (Flash) : Pin 14(Teensy)

Pin 3 (Flash) : Pin 7(Teensy)

Pin 4 (Flash) : GND

Pin 5 (Flash) : Pin 2(Teensy)

Pin 6 (Flash) : Pin 20(Teensy)

Pin 7 (Flash) : Pin 8(Teensy)

Pin 8 (Flash) : 3.3Volt
