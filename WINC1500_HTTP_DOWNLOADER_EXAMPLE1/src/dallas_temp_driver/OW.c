// #include <asf.h>
// #include "dallas_temp_driver/OW.h"
// 
// //based on https://onlinedocs.microchip.com/pr/GUID-1618003F-992B-4E48-9411-5E5D5D952C06-en-US-3/index.html documents about Dallas One Wire
// //assembly and tick based delay, actually allowed everything to work

// owner: Author: Roberto Milan 

#include <asf.h>
#include "dallas_temp_driver/OW.h"

SemaphoreHandle_t sensorOWMutexHandle;						 ///<Mutex to handle the sensor I2C bus thread access.
SemaphoreHandle_t sensorOWSemaphoreHandle;		///<Binary semaphore to notify task that we have received an I2C interrupt on the Sensor bus
static volatile TaskHandle_t xTaskToNotifySensorDone = NULL; ///< Stores the handle of the task that will be notified when the SENSOR transmission is complete. */
static uint8_t sensorTransmitError = false;					 ///<Flag used to indicate that there was an I2C transmission error on the SENSOR bus.

//based on https://onlinedocs.microchip.com/pr/GUID-1618003F-992B-4E48-9411-5E5D5D952C06-en-US-3/index.html documents about Dallas One Wire
//FreeRTOS integration based on A08 I2C driver
//assembly and tick based delay, actually allowed everything to work

void OWSensorsTxComplete(void){
				
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	xSemaphoreGiveFromISR( sensorOWSemaphoreHandle, &xHigherPriorityTaskWoken );
	sensorTransmitError = false;
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void OWSensorsRxComplete(void){
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	xSemaphoreGiveFromISR( sensorOWSemaphoreHandle, &xHigherPriorityTaskWoken );
	sensorTransmitError = false;
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

int32_t OWFreeMutex(void){
	if (xSemaphoreGive(sensorOWMutexHandle) != pdTRUE) {
		//error
		return 1; 
	}
	//no error
	return 0;
}

int32_t OWGetMutex(TickType_t waitTime){
    if (xSemaphoreTake(sensorOWMutexHandle, waitTime) == pdTRUE) {
        // Mutex acquired successfully
        return 0;
    } else {
        // Failed to acquire mutex within the wait time
        return 1;
    }
}

static int32_t OWGetSemaphoreHandle(SemaphoreHandle_t *handle){
	int32_t error = 0;
	*handle = sensorOWSemaphoreHandle;
	return error;
}

static uint8_t OWGetTaskErrorStatus(void){
	 return sensorTransmitError;
 }

static void OWSetTaskErrorStatus(uint8_t value){
	sensorTransmitError = value;
 }

void __delay_us(uint32_t microseconds) {

	uint32_t iterations = (microseconds * (8000000 / 1000000)) / 2;

	for (uint32_t i = 0; i < iterations; ++i) {
		asm volatile ("nop");  // No operation
	}
}

void ds18b20_init(void) {
	// Configure DS18B20 pin as input with pull-up
	PORT->Group[0].PINCFG[DS18B20_PIN & 0x1F].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	PORT->Group[0].DIRCLR.reg = 1U << (DS18B20_PIN & 0x1F);

	sensorOWMutexHandle = xSemaphoreCreateMutex();
	sensorOWSemaphoreHandle = xSemaphoreCreateBinary();
}

void configure_out(void) {
	// Configure DS18B20 pin as output
	PORT->Group[0].DIRSET.reg = 1U << (DS18B20_PIN & 0x1F);
}

void release_bus(void) {
	// Configure DS18B20 pin as input without pull-up (allow DS18B20 to pull on the line)
	PORT->Group[0].PINCFG[DS18B20_PIN & 0x1F].reg = PORT_PINCFG_INEN;
	PORT->Group[0].OUTSET.reg = 1U << (DS18B20_PIN & 0x1F);
}

//Probably could've implemented HAL after realizing that the problem was the delay function
//but better to have lowest delay using masking
void writebit(uint8_t val) {
	switch(val) {
		case 0:
		// Disable interrupts
		__disable_irq();
		configure_out();
		PORT->Group[0].OUTCLR.reg = 1U << (DS18B20_PIN & 0x1F);
		__delay_us(DELAYC);
		release_bus();
		__delay_us(DELAYD);
		// Enable interrupts
		__enable_irq();
		PORT->Group[0].PINCFG[DS18B20_PIN & 0x1F].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
		PORT->Group[0].DIRCLR.reg = 1U << (DS18B20_PIN & 0x1F);
		break;

		case 1:
		// Disable interrupts
		__disable_irq();
		configure_out();
		PORT->Group[0].OUTCLR.reg = 1U << (DS18B20_PIN & 0x1F);
		__delay_us(DELAYA);
		release_bus();
		__delay_us(DELAYB);
		// Enable interrupts
		__enable_irq();
		PORT->Group[0].PINCFG[DS18B20_PIN & 0x1F].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
		PORT->Group[0].DIRCLR.reg = 1U << (DS18B20_PIN & 0x1F);
		break;

		default:
		return;
	}
}

uint8_t readbit(void) {
	uint8_t bit;
	// Enable interrupts
	__enable_irq();
	configure_out();
	PORT->Group[0].OUTCLR.reg = 1U << (DS18B20_PIN & 0x1F);
	__delay_us(DELAYA);
	release_bus();
	__delay_us(DELAYE);
	bit = (PORT->Group[0].IN.reg >> (DS18B20_PIN & 0x1F)) & 1U;
	__delay_us(DELAYF);
	// Enable interrupts
	__enable_irq();
	PORT->Group[0].PINCFG[DS18B20_PIN & 0x1F].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	PORT->Group[0].DIRCLR.reg = 1U << (DS18B20_PIN & 0x1F);
	return bit;
}

bool reset_detect(void) {
	bool exist;
	// Disable interrupts
	__disable_irq();
	configure_out();
	PORT->Group[0].OUTCLR.reg = 1U << (DS18B20_PIN & 0x1F);
	__delay_us(DELAYH);
	release_bus();
	__delay_us(DELAYI);
	exist = (PORT->Group[0].IN.reg >> (DS18B20_PIN & 0x1F)) & 1U;
	__delay_us(DELAYJ);
	// Enable interrupts
	__enable_irq();
	PORT->Group[0].PINCFG[DS18B20_PIN & 0x1F].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
	PORT->Group[0].DIRCLR.reg = 1U << (DS18B20_PIN & 0x1F);
	return exist;
}

void write_byte(uint8_t byte) {
	for(int i = 0; i < 8; i++) {
		writebit(byte & 0x01);
		byte >>= 1;
	}
	OWSensorsTxComplete();
}

uint8_t read_byte(void) {
	uint8_t byte = 0;
	for(int i = 0; i < 8; i++) {
		if(readbit()) {
			//simple shift and or bit into byte
			byte |= (0x01 << i);
		}
	}
	OWSensorsRxComplete();
	return byte;
}

/**************************************************************************//**
 * @fn			int32_t I2cWriteDataWait(I2C_Data *data, const TickType_t delay, const TickType_t xMaxBlockTime)
 * @brief       This is the main function to use to write data from an I2C device on a given I2C Bus. This function is blocking.
 * @details     This function writes data from an I2C device, by writing the requested bytes.This function is blocking (bare-metal) or it
				makes the current thread sleep until the I2C bus has finished the transaction (FREERTOS version).
				On FreeRtos, this function gets the mutex for the respective I2C bus.
 * @param[in]   data Pointer to I2C data structure which has all the information needed to send an I2C message
 * @param[in]   delay Delay that the I2C device needs to return the response. Can be 0 if the response is ready instantly. It can be 
				the delay an I2C device needs to make a measurement.
 * @param[in]   xMaxBlockTime Maximum time for the thread to wait until the I2C mutex is free.
 * @return      Returns an error message in case of error.
 * @note        
 *****************************************************************************/
int32_t OWWriteDataWait(uint8_t data, const TickType_t xMaxBlockTime){
	int32_t error = 0;
	int32_t ERROR_NONE = 0;

	SemaphoreHandle_t semHandle = NULL;

	//---0. Get Mutex
	error = OWGetMutex(xMaxBlockTime);
	if(ERROR_NONE != error) goto exit;

	//---1. Get Semaphore Handle
	error = OWGetSemaphoreHandle(&semHandle);
	if(ERROR_NONE != error) goto exit;

	//---2. Initiate sending data
	write_byte(data);

	//---3. Wait for binary semaphore to tell us that we are done!
	if(xSemaphoreTake(semHandle, xMaxBlockTime) == pdTRUE){
		/* The transmission ended as expected. We now delay until the I2C sensor is finished */
		if(OWGetTaskErrorStatus()){
			OWSetTaskErrorStatus(false);
			if(error != ERROR_NONE){
				error = 1;
				}else{
				error = 1;
			}
			goto exitError0;
		}
		}else{
		/* The call to xSemaphoreTake() timed out. */
		error = 1;
		goto exitError0;
	}

	//---4. Release Mutex
	error |= OWFreeMutex();

	exit:
	return error;

	exitError0:
	OWFreeMutex();

	return error;
}


/**************************************************************************//**
 * @fn			int32_t I2cReadDataWait(I2C_Data *data, const TickType_t delay, const TickType_t xMaxBlockTime)
 * @brief       This is the main function to use to read data from an I2C device on a given I2C Bus. This function is blocking.
 * @details     This function reads data from an I2C device, by first writing to the address (I2C device address + register) and then reading the requested bytes. This
				function is blocking (bare-metal) or it makes the current thread sleep until the I2C bus has finished the transaction (FREERTOS version).
				On FreeRtos, this function gets the mutex for the respective I2C bus.
 * @param[in]   data Pointer to I2C data structure which has all the information needed to send an I2C message
 * @param[in]   delay Delay that the I2C device needs to return the response. Can be 0 if the response is ready instantly. It can be the delay an I2C device needs to make a measurement.
 * @param[in]   xMaxBlockTime Maximum time for the thread to wait until the I2C mutex is free.
 * @return      Returns an error message in case of error. See ErrCodes.h
 * @note        THIS IS THE FREERTOS VERSION! DO NOT Declare #define USE_FREERTOS if you wish to use the baremetal version!      
				students to fill!
 *****************************************************************************/
int8_t OWReadDataWait(const TickType_t delay, const TickType_t xMaxBlockTime){
    int32_t error = 0;
	int32_t ERROR_NONE = 0;
	uint8_t byte = 0;
    SemaphoreHandle_t semHandle = NULL;

    //---0. Get Mutex
    error = OWGetMutex(xMaxBlockTime);
    if(ERROR_NONE != error) goto exit;

    //---1. Get Semaphore Handle
    error = OWGetSemaphoreHandle(&semHandle);
    if(ERROR_NONE != error) goto exit;

    //---5. Initiate Read data
    byte = read_byte();

    //---6. Wait for notification that the read is done!
    if( xSemaphoreTake( semHandle, xMaxBlockTime ) != pdTRUE ){
        // The call to xSemaphoreTake() timed out.
        error = 1;
        goto exitError0;
    }

    // Check for transmission error after read
    if(OWGetTaskErrorStatus()){
        OWSetTaskErrorStatus(false);
        error = 1;
        goto exitError0;
    }

    //---7. Release Mutex
    OWFreeMutex();
    goto exit;

exit:
    return byte;

exitError0:
    OWFreeMutex();
    return error;
}