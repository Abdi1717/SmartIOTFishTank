
#ifndef servo_H_
#define servo_H_
#include <asf.h>



void configure_tcc_for_pwm(void);
void configure_pwm_pin(void);
void servo_set(uint32_t degrees);
/*int32_t servoWriteDataWait(uint32_t data, const TickType_t xMaxBlockTime);*/
void configure_clocks(void);




#endif