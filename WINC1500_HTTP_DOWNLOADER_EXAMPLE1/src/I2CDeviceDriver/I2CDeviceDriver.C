
/*
 * I2CDeviceDriver.C
 *
 * Created: 11/6/2023 8:31:22 PM
 *  Author: AbdiMPC
 */ 


// This is your C file where you define the functions declared in the header.
#include "I2CDeviceDriver.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "i2c_master.h"
#include "FreeRTOS_CLI.h"    // Include FreeRTOS CLI header


#define NAU7802_ADDRESS 0x2A // Replace with the actual I2C address for your device


#define NAU7802_ADC_RESULT_REG_MSB 0x12
#define NAU7802_ADC_RESULT_REG_MIDB 0x13
#define NAU7802_ADC_RESULT_REG_LSB 0x14

// Declare the semaphore handle as a static variable, which means it will be private to this file.
static SemaphoreHandle_t i2cSemaphore;

void I2CDevice_Init() {
	 i2c_master_init();

	// Create the semaphore for I2C bus access
	i2cSemaphore = xSemaphoreCreateMutex();
}



bool I2CDevice_Write(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data) {
	if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE) {
		// Start the I2C write transaction
		i2c_master_write_to_register(deviceAddress, registerAddress, &data, 1);
		
		// Release the semaphore after the operation
		xSemaphoreGive(i2cSemaphore);
		
		return true; // Assume success for now
	}
	return false; // Semaphore not obtained
}

bool I2CDevice_Read(uint8_t deviceAddress, uint8_t registerAddress, uint8_t *data) {
	if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE) {
		// Start the I2C read transaction
		i2c_master_read_from_register(deviceAddress, registerAddress, data, 1);
		
		// Release the semaphore after the operation
		xSemaphoreGive(i2cSemaphore);
		
		return true; // Assume success for now
	}
	return false; // Semaphore not obtained
}




// Helper function to convert ADC result to a 24-bit integer
static int32_t convert_adc_result(uint8_t msb, uint8_t midb, uint8_t lsb) {
	// Shift and combine bytes to form a 24-bit integer
	return ((int32_t)msb << 16) | ((int32_t)midb << 8) | lsb;
}

// Example CLI command to read a register
static BaseType_t prvI2CReadCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	(void)pcCommandString; 
	
	// Perform I2C read operation for ADC data
	uint8_t msb, midb, lsb;
	if (I2CDevice_Read(NAU7802_ADDRESS, NAU7802_ADC_RESULT_REG_MSB, &msb) &&
	I2CDevice_Read(NAU7802_ADDRESS, NAU7802_ADC_RESULT_REG_MIDB, &midb) &&
	I2CDevice_Read(NAU7802_ADDRESS, NAU7802_ADC_RESULT_REG_LSB, &lsb)) {
		
		// Convert ADC result to 24-bit integer
		int32_t adcResult = convert_adc_result(msb, midb, lsb);
		
		// Format the output string
		snprintf(pcWriteBuffer, xWriteBufferLen, "ADC Result: %ld\r\n", adcResult);
		
		return pdFALSE; // No more data to return after this point, hence pdFALSE
		} else {
		// I2C read failed, output an error message
		snprintf(pcWriteBuffer, xWriteBufferLen, "Error reading ADC result\r\n");
		return pdFALSE; // No more data to return after this point, hence pdFALSE
	}
}
static BaseType_t prvI2CInitCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	I2CDevice_Init();
	snprintf(pcWriteBuffer, xWriteBufferLen, "NAU7802 Initialized.\r\n");
	return pdFALSE;
}

static BaseType_t prvI2CWriteCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	uint8_t regAddress = 0;
	uint8_t regValue = 0;

	// Parse the command string for register address and value
	const char *param = FreeRTOS_CLIGetParameter(pcCommandString, 1, NULL); // First parameter: register address
	regAddress = (uint8_t) strtol(param, NULL, 16);

	param = FreeRTOS_CLIGetParameter(pcCommandString, 2, NULL); // Second parameter: register value
	regValue = (uint8_t) strtol(param, NULL, 16);

	if (I2CDevice_Write(NAU7802_ADDRESS, regAddress, regValue)) {
		snprintf(pcWriteBuffer, xWriteBufferLen, "Write to reg 0x%02X: 0x%02X\r\n", regAddress, regValue);
		} else {
		snprintf(pcWriteBuffer, xWriteBufferLen, "Failed to write to reg 0x%02X\r\n", regAddress);
	}
	
	return pdFALSE;
}

static const CLI_Command_Definition_t xI2CInit = {
	"i2c_init",
	"i2c_init: Initialize the I2C device.\r\n",
	prvI2CInitCommand,
	0
};

static const CLI_Command_Definition_t xI2CReadADC = {
	"i2c_read_adc",
	"i2c_read_adc: Read ADC value.\r\n",
	prvI2CReadCommand,
	0
};

static const CLI_Command_Definition_t xI2CWriteReg = {
	"i2c_write_reg",
	"i2c_write_reg <reg> <val>: Write a value to a register.\r\n",
	prvI2CWriteCommand,
	2 // Number of expected parameters
};

void vRegisterI2CCommands(void) {
	FreeRTOS_CLIRegisterCommand(&xI2CInit);
	FreeRTOS_CLIRegisterCommand(&xI2CReadADC);
	FreeRTOS_CLIRegisterCommand(&xI2CWriteReg);
}
