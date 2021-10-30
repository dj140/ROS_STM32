#ifdef __cplusplus
extern "C" {
#endif
#ifndef _ENCODER_H_
#define _ENCODER_H_
 
#include "Arduino.h"
#include "config.h"



//#define HIGH 1
//#define LOW  0

void encoder_init(void);
extern int en_pos1;
extern int en_pos2;
extern int en_pos3;
extern int en_pos4;
 

#endif  

#ifdef __cplusplus
}
#endif
