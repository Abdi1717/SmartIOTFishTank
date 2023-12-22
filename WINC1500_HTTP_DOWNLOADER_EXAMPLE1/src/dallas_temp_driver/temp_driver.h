/*
 * temp_driver.h
 *
 * Created: 12/17/2023 5:51:36 PM
 *  Author: Owner
 */ 


#ifndef TEMP_DRIVER_H_
#define TEMP_DRIVER_H_

void ds18b20_start_conversion(void);
uint16_t ds18b20_read_temp(void);

#endif /* TEMP_DRIVER_H_ */