/*
 * temp_driver.c
 *
 * Created: 12/17/2023 5:51:55 PM
 *  Author: Owner
 */ 
#include <asf.h>
#include "dallas_temp_driver/temp_driver.h"
#include "dallas_temp_driver/OW.h"
#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t OWSemaphore;

void ds18b20_start_conversion(void) {
	
	reset_detect();
	OWWriteDataWait(DS18B20_CMD_SKIPROM,portMAX_DELAY);
	OWWriteDataWait(DS18B20_CMD_CONVTEMP,portMAX_DELAY);
// 	write_byte(DS18B20_CMD_SKIPROM);
// 	write_byte(DS18B20_CMD_CONVTEMP);
	
}

uint16_t ds18b20_read_temp(void) {
	reset_detect();
	OWWriteDataWait(DS18B20_CMD_SKIPROM,portMAX_DELAY);
	OWWriteDataWait(DS18B20_CMD_READSCRATCH,portMAX_DELAY);
	
	uint8_t lsb = OWReadDataWait(0,portMAX_DELAY);
	uint8_t msb = OWReadDataWait(0,portMAX_DELAY);

	uint16_t raw_temp = (msb << 8) | lsb;
	
	return raw_temp;
}