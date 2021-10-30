#ifndef __CONFIG_H
#define __CONFIG_H

#define DEBUG   1
#define USE_GY85_IMU

#define K_P    0.1 // P constant
#define K_I    0.1 // I constant
#define K_D    0.1// D constant

/** motor param **/
#define PWM_BITS        8
#define MAX_RPM         366 //motor's maximum RPM
#define COUNTS_PER_REV  1560 //wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
#define WHEEL_DIAMETER  0.206 //wheel's diameter in meters

#define LR_WHEELS_DISTANCE 0.70 // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.68
#define IMU_PUBLISH_RATE 100 //hz
#define BAT_PUBLISH_RATE 0.2 //hz
#define COMMAND_RATE 100 //hz
#define DEBUG_RATE 1

//#define CALIB
//SCK-PB3 MISO-PB4 MOSI-PB5

#define LED_Pin PC13

#define MOTOR1_PWM_Pin PB6
#define MOTOR2_PWM_Pin PB7
#define MOTOR3_PWM_Pin PB0
#define MOTOR4_PWM_Pin PB1

#define MOTOR1_A_PIN   PC14
#define MOTOR2_A_PIN   PC15
#define MOTOR3_A_PIN   PA11
#define MOTOR4_A_PIN   PA12

#define MOTOR1_B_PIN   PB12
#define MOTOR2_B_PIN   PB13
#define MOTOR3_B_PIN   PB14
#define MOTOR4_B_PIN   PB15

#define ENCODE1_A PA1
#define ENCODE1_B PA2

#define ENCODE2_A PA4
#define ENCODE2_B PA3

#define ENCODE3_A PA5
#define ENCODE3_B PA6

#define ENCODE4_A PA8
#define ENCODE4_B PA7

#endif
