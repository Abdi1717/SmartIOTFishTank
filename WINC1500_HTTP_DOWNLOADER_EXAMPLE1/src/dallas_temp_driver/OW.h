/*
 * IncFile1.h
 *
 * Created: 12/17/2023 4:38:25 PM
 *  Author: Roberto Milan
 */ 


#ifndef OW_H_
#define OW_H_

# include <FreeRTOS.h>
# include <semphr.h>
# include <task.h>

#define DS18B20_PIN PIN_PA20
#define DS18B20_CMD_CONVTEMP 0x44
#define DS18B20_CMD_READSCRATCH 0xBE
#define DS18B20_CMD_WRITESCRATCH 0x4E
#define DS18B20_CMD_SKIPROM 0xCC

#define DELAYA ((uint32_t) 6)
#define DELAYB ((uint32_t) 64)
#define DELAYC ((uint32_t) 60)
#define DELAYD ((uint32_t) 10)
#define DELAYE ((uint32_t) 9)
#define DELAYF ((uint32_t) 55)
#define DELAYG ((uint32_t) 0)
#define DELAYH ((uint32_t) 480)
#define DELAYI ((uint32_t) 70)
#define DELAYJ ((uint32_t) 410)

void __delay_us(uint32_t microseconds);
void ds18b20_init(void);
void writebit(uint8_t val);
void release_bus(void);
void configure_out(void);
uint8_t readbit(void);
bool reset_detect(void);
// void write_byte(uint8_t byte);
// uint8_t read_byte(void);
int32_t OWWriteDataWait(uint8_t data, const TickType_t xMaxBlockTime);
int8_t OWReadDataWait(const TickType_t delay, const TickType_t xMaxBlockTime);









#endif /* INCFILE1_H_ */