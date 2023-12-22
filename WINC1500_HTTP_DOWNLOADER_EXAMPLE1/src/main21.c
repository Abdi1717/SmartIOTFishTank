/**************************************************************************/ /**
* @file      main.c
* @brief     Main application entry point
* @author    Eduardo Garcia
* @date      2020-02-15
* @copyright Copyright Bresslergroup\n
*            This file is proprietary to Bresslergroup.
*            All rights reserved. Reproduction or distribution, in whole
*            or in part, is forbidden except by express written permission
*            of Bresslergroup.
******************************************************************************/


/******************************************************************************
* Includes
******************************************************************************/
#include <errno.h>
#include "asf.h"
#include "main.h"
#include "stdio_serial.h"
#include "SerialConsole.h"
#include "FreeRTOS.h"
#include "driver/include/m2m_wifi.h"
#include "CliThread/CliThread.h"
#include "SeesawDriver/Seesaw.h"
#include "IMU\lsm6dso_reg.h"
#include "UiHandlerThread\UiHandlerThread.h"
#include "LidarSensor\Lidar.h"
#include "DistanceDriver\DistanceSensor.h"
#include "servo/servo.h"
#include "dallas_temp_driver/OW.h"
#include "ScreenI2cDriver\SSD1306.h"
#include "WifiHandlerThread/WifiHandler.h"

/******************************************************************************
* Defines and Types
******************************************************************************/
#define APP_TASK_ID 0 /**< @brief ID for the application task */
#define CLI_TASK_ID 1 /**< @brief ID for the command line interface task */
//#define BOOT_TEST	1 //Uncomment me to compile boot test.

/******************************************************************************
* Local Function Declaration
******************************************************************************/
void vApplicationIdleHook(void);
//!< Initial task used to initialize HW before other tasks are initialized
static void StartTasks(void);
void vApplicationDaemonTaskStartupHook(void);


void initializeHCSR04(void);
uint16_t getDistance(void);
static void vDistanceMeasurementTask(void *pvParameters);
static void vDistance100MeasurementTask(void *pvParameters);
static void vSSD1306Task(void *pvParameters);
static void vDistance100MeasurementTempeartureTask(void *pvParameters);
static TaskHandle_t wifiTaskHandle = NULL;     //!< Wifi task handle
/******************************************************************************
* Variables
******************************************************************************/
static TaskHandle_t cliTaskHandle    = NULL; //!< CLI task handle
static TaskHandle_t daemonTaskHandle    = NULL; //!< Daemon task handle
static TaskHandle_t uiTaskHandle    = NULL; //!< UI task handle
// Define the pins for the HC-SR04 sensor
#define TRIG_PIN PIN_PB02   // TRIG pin connected to GPIO1
#define ECHO_PIN PIN_PB03   // ECHO pin connected to GPIO2

//#define TRIG_PIN PIN_PB10   // TRIG pin connected to GPIO1
//#define ECHO_PIN PIN_PB11   // ECHO pin connected to GPIO2

char bufferPrint[64]; //Buffer for daemon task

/**
 * \brief Main application function.
 *
 * Application entry point.
 *
 * \return program return value.
 */
int main(void)
{
	/* Initialize the board. */
	system_init();

	/* Initialize the UART console. */
	InitializeSerialConsole();

	//Initialize trace capabilities
	 vTraceEnable(TRC_START);

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

	return 0; //Will not get here
}

/**************************************************************************/ /**
* function          vApplicationDaemonTaskStartupHook
* @brief            Initialization code for all subsystems that require FreeRToS
* @details			This function is called from the FreeRToS timer task. Any code
*					here will be called before other tasks are initilized.
* @param[in]        None
* @return           None
*****************************************************************************/
volatile uint8_t data[256];
void vApplicationDaemonTaskStartupHook(void)
{

SerialConsoleWriteString("\r\n\r\n-----ESE516 Main Program-----\r\n");

//Initialize HW that needs FreeRTOS Initialization
SerialConsoleWriteString("\r\n\r\nInitialize HW...\r\n");
 // initializeHCSR04();

	if (I2cInitializeDriver() != STATUS_OK)
	{
		SerialConsoleWriteString("Error initializing I2C Driver!\r\n");
	}
	else
	{
		SerialConsoleWriteString("Initialized I2C Driver!\r\n");
	}
	
	   SerialConsoleWriteString("\r\nInitializing US-100 Distance Sensor...\r\n");
	   InitializeDistanceSensor();
	
/*
	if(0 != InitializeSeesaw())
	{
		SerialConsoleWriteString("Error initializing Seesaw!\r\n");
	}	
	else
	{
		SerialConsoleWriteString("Initialized Seesaw!\r\n");
	}

	uint8_t whoamI = 0;
	(lsm6dso_device_id_get(GetImuStruct(), &whoamI));
	
	if (whoamI != LSM6DSO_ID){
		SerialConsoleWriteString("Cannot find IMU!\r\n");
	}
	else
	{
		SerialConsoleWriteString("IMU found!\r\n");
		if(InitImu() == 0)
		{
			SerialConsoleWriteString("IMU initialized!\r\n");
		}
		else
		{
			SerialConsoleWriteString("Could not initialize IMU\r\n");
		}
	}
*/

	   	i2c_begin(); // Initialize SSD1306
	   	SerialConsoleWriteString("SSD1306 initialized!\r\n");
	   	



	StartTasks();

	vTaskSuspend(daemonTaskHandle);
}

/**************************************************************************//**
* function          StartTasks
* @brief            Initialize application tasks
* @details
* @param[in]        None
* @return           None
*****************************************************************************/




static void StartTasks(void)
{

snprintf(bufferPrint, 64, "Heap before starting tasks: %d\r\n", xPortGetFreeHeapSize());
SerialConsoleWriteString(bufferPrint);

//Initialize Tasks here

if (xTaskCreate(vCommandConsoleTask, "CLI_TASK", CLI_TASK_SIZE, NULL, CLI_PRIORITY, &cliTaskHandle) != pdPASS) {
	SerialConsoleWriteString("ERR: CLI task could not be initialized!\r\n");
}

snprintf(bufferPrint, 64, "Heap after starting CLI: %d\r\n", xPortGetFreeHeapSize());
SerialConsoleWriteString(bufferPrint);

    if (xTaskCreate(vWifiTask, "WIFI_TASK", WIFI_TASK_SIZE, NULL, WIFI_PRIORITY, &wifiTaskHandle) != pdPASS) {
	    SerialConsoleWriteString("ERR: WIFI task could not be initialized!\r\n");
    }
    snprintf(bufferPrint, 64, "Heap after starting WIFI: %d\r\n", xPortGetFreeHeapSize());
    SerialConsoleWriteString(bufferPrint);




i2c_begin(); // Initialize SSD1306
SerialConsoleWriteString("SSD1306 initialized!\r\n");
/*
if (xTaskCreate(vSSD1306Task, "SSD1306 Task", 500, NULL, 2, NULL) != pdPASS) {
	SerialConsoleWriteString("ERR: SSD1306 task could not be initialized!\r\n");
}*/

/*
 if (xTaskCreate(vDistanceMeasurementTask, "DistanceMeasurementTask", 1024, NULL, 2, NULL) != pdPASS) {
	   SerialConsoleWriteString("Error creating Distance Measurement Task!\r\n");
   }
   snprintf(bufferPrint, 64, "Heap after starting Distance Measurement Task: %d\r\n", xPortGetFreeHeapSize());
   SerialConsoleWriteString(bufferPrint);
*/

/*
   if (xTaskCreate(vDistance100MeasurementTask, "US100DistanceTask", 1024, NULL, 2, NULL) != pdPASS) {
	    SerialConsoleWriteString("Error creating US-100 Distance Measurement Task!\r\n");
    }

    snprintf(bufferPrint, 64, "Heap after starting all tasks: %d\r\n", xPortGetFreeHeapSize());
    SerialConsoleWriteString(bufferPrint);*/


if (xTaskCreate(vDistance100MeasurementTempeartureTask, "Distance100MeasurementTempeartureTask", 500, NULL, 2, NULL) != pdPASS) {
	SerialConsoleWriteString("Error creating Distance100MeasurementTempeartureTask!\r\n");
}

snprintf(bufferPrint, 64, "Heap after starting all tasks: %d\r\n", xPortGetFreeHeapSize());
SerialConsoleWriteString(bufferPrint);

	
}





static void configure_console(void)
{

	stdio_base = (void *)GetUsartModule();
	ptr_put = (int (*)(void volatile*,char))&usart_serial_putchar;
	ptr_get = (void (*)(void volatile*,char*))&usart_serial_getchar;


	# if defined(__GNUC__)
	// Specify that stdout and stdin should not be buffered.
	setbuf(stdout, NULL);
	setbuf(stdin, NULL);
	// Note: Already the case in IAR's Normal DLIB default configuration
	// and AVR GCC library:
	// - printf() emits one character at a time.
	// - getchar() requests only 1 byte to exit.
	#  endif
	//stdio_serial_init(GetUsartModule(), EDBG_CDC_MODULE, &usart_conf);
	//usart_enable(&cdc_uart_module);
}





void vApplicationMallocFailedHook(void)
{
SerialConsoleWriteString("Error on memory allocation on FREERTOS!\r\n");
while(1);
}

void vApplicationStackOverflowHook(void)
{
SerialConsoleWriteString("Error on stack overflow on FREERTOS!\r\n");
while(1);
}

static void vSSD1306Task(void *pvParameters) {
	while(1) {

		i2c_clear(); // Clear the display
		i2c_setCursor(0, 0); // Set cursor at the top-left corner
		i2c_print("Hello, World!"); // Print a message
		vTaskDelay(pdMS_TO_TICKS(1000)); // Refresh every second
	}
}


static void vDistance100MeasurementTask(void *pvParameters) {
    (void) pvParameters;  // Unused parameter
    char buffer[50];
    const uint16_t maxDistance = 170;  // 170 mm is considered as 100% food

    while (1) {
        uint16_t distance;
        int32_t error = DistanceSensorGetDistance(&distance, pdMS_TO_TICKS(1000)); // 1000 ms as max block time

        if (error == ERROR_NONE) {
            // Calculate food remaining as a percentage
            int foodRemaining = (distance > maxDistance) ? 0 : (100 - (distance * 100 / maxDistance));

            snprintf(buffer, sizeof(buffer), "Food remaining: %d%%", foodRemaining);
            SerialConsoleWriteString(buffer); // Print on serial console

            i2c_clear(); // Clear the SSD1306 display
            i2c_setCursor(0, 0); // Set cursor at the top-left corner
            i2c_print(buffer); // Print the food remaining on the SSD1306 display
        } else {
            SerialConsoleWriteString("Error reading distance\r\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay before next measurement
    }
}


static void vDistance100MeasurementTempeartureTask(void *pvParameters) {
	(void) pvParameters;  // Unused parameter
	char buffer[100];  // Buffer size increased to accommodate both readings
	const uint16_t maxDistance = 170;  // 170 mm is 100% food
	uint16_t distance, rawTemperature;
	int32_t error;
	static uint16_t temperature;
	static float temp_celc;

	while (1) {
		// Measure distance
		error = DistanceSensorGetDistance(&distance, pdMS_TO_TICKS(1000)); // 1000 ms as max block time
		int foodRemaining = 0;

		if (error == ERROR_NONE) {
			// Calculate food remaining as a percentage
			foodRemaining = (distance > maxDistance) ? 0 : (100 - (distance * 100 / maxDistance));
			} else {
			SerialConsoleWriteString("Error reading distance\r\n");
		}

/*
		// Measure temperature
		ds18b20_init();
		ds18b20_start_conversion();
		delay_ms(750);
		//vTaskDelay(pdMS_TO_TICKS(750)); // Delay for conversion time
		rawTemperature = ds18b20_read_temp();
		delay_ms(1000);
		temp_celc = rawTemperature / 16.0;
		*/
		
		

		// Update the SSD1306 display
		i2c_clear(); // Clear the display
		i2c_setCursor(0, 0); // Set cursor at the top-left corner
		snprintf(buffer, sizeof(buffer), "Food: %d%%", foodRemaining);
		i2c_print(buffer); // Print the food remaining
		vTaskDelay(pdMS_TO_TICKS(1000)); 
/*
		i2c_setCursor(0, 1); // Move cursor to the next line
		snprintf(buffer, sizeof(buffer), "Temperature:  %u C", (uint16_t)temp_celc);
		i2c_print(buffer); // Print the temperature
	 // snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Temperature: %u \r\n", (uint16_t)temp_celc);
		vTaskDelay(pdMS_TO_TICKS(1000)); // Delay before next measurement
		
		*/
		
	
	}
}



/*

static void vDistance100MeasurementTask(void *pvParameters) {
    (void) pvParameters;  // Unused parameter
    char buffer[50];
    uint16_t distance;
    uint8_t temperature;
    int32_t error;

    while (1) {
        // Get distance
        error = DistanceSensorGetDistance(&distance, pdMS_TO_TICKS(1000));
        if (error == ERROR_NONE) {
            snprintf(buffer, sizeof(buffer), "Distance: %u mm", distance);
            SerialConsoleWriteString(buffer);
        } else {
            SerialConsoleWriteString("Error reading distance\r\n");
        }

        // Get temperature
        error = DistanceSensorGetTemperature(&temperature, pdMS_TO_TICKS(1000));
        if (error == ERROR_NONE) {
            snprintf(buffer, sizeof(buffer), "Temp: %u C", temperature);
            SerialConsoleWriteString(buffer);
        } else {
            SerialConsoleWriteString("Error reading temperature\r\n");
        }

        // Update SSD1306
        i2c_clear(); // Clear the display
        i2c_setCursor(0, 0); // Set cursor to top-left corner
        i2c_print("Distance: "); // Print label
        i2c_printNum(distance); // Print distance
        i2c_setCursor(0, 1); // Move to next line
        i2c_print("Temp: "); // Print label
        i2c_printNum(temperature); // Print temperature

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay before next measurement
    }
	
	
}
*/
