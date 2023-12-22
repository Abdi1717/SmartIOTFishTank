/**************************************************************************//**
* @file      CliThread.c
* @brief     File for the CLI Thread handler. Uses FREERTOS + CLI
* @author    Eduardo Garcia
* @date      2020-02-15

******************************************************************************/


/******************************************************************************
* Includes
******************************************************************************/
#include "CliThread.h"
#include "IMU\lsm6dso_reg.h"
#include "SeesawDriver/Seesaw.h"
#include "dallas_temp_driver/temp_driver.h"
#include "dallas_temp_driver/OW.h"
#include "servo/servo.h"
#include "DistanceDriver/DistanceSensor.h"
#include "WifiHandlerThread/WifiHandler.h"
#include <asf.h>

/******************************************************************************
* Defines
******************************************************************************/


/******************************************************************************
* Variables
******************************************************************************/
static const int8_t * const pcWelcomeMessage =
"FreeRTOS CLI.\r\nType Help to view a list of registered commands.\r\n";

SemaphoreHandle_t cliCharReadySemaphore;		///<Semaphore to indicate that a character has been received


static const CLI_Command_Definition_t xImuGetCommand =
{
	"imu",
	"imu: Returns a value from the IMU\r\n",
	CLI_GetImuData,
	0
};


static const CLI_Command_Definition_t xResetCommand =
{
	"reset",
	"reset: Resets the device\r\n",
	CLI_ResetDevice,
	0
};

static const CLI_Command_Definition_t xNeotrellisTurnLEDCommand =
{
	"led",
	"led [keynum][R][G][B]: Sets the given LED to the given R,G,B values.\r\n",
	CLI_NeotrellisSetLed,
	4
};



//Clear screen command
const CLI_Command_Definition_t xClearScreen =
{
	CLI_COMMAND_CLEAR_SCREEN,
	CLI_HELP_CLEAR_SCREEN,
	CLI_CALLBACK_CLEAR_SCREEN,
	CLI_PARAMS_CLEAR_SCREEN
};


static const CLI_Command_Definition_t xTempGetCommand = {"temp", "temp: Returns a value from the temp sensor\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_GetTemp, 0};
	
static const CLI_Command_Definition_t xFeedCommand = {"feed", "feed: Feed the fish\r\n", (const pdCOMMAND_LINE_CALLBACK)CLI_Feed, 0};	


static const CLI_Command_Definition_t xDistanceSensorGetDistance = {"getdistance",
                                                                    "getdistance: Returns the distance from the US-100 Sensor.\r\n",
                                                                    (const pdCOMMAND_LINE_CALLBACK)CLI_DistanceSensorGetDistance,
                                                                    0};

/******************************************************************************
* Forward Declarations
******************************************************************************/
static void FreeRTOS_read(char* character);

/******************************************************************************
* Callback Functions
******************************************************************************/


/******************************************************************************
* CLI Thread
******************************************************************************/

void vCommandConsoleTask( void *pvParameters )
{
//REGISTER COMMANDS HERE

FreeRTOS_CLIRegisterCommand( &xImuGetCommand );
FreeRTOS_CLIRegisterCommand( &xClearScreen );
FreeRTOS_CLIRegisterCommand( &xResetCommand );
FreeRTOS_CLIRegisterCommand( &xNeotrellisTurnLEDCommand );
FreeRTOS_CLIRegisterCommand(&xFeedCommand);
FreeRTOS_CLIRegisterCommand(&xTempGetCommand);
FreeRTOS_CLIRegisterCommand(&xDistanceSensorGetDistance);

uint8_t cRxedChar[2], cInputIndex = 0;
BaseType_t xMoreDataToFollow;
/* The input and output buffers are declared static to keep them off the stack. */
static int8_t pcOutputString[ MAX_OUTPUT_LENGTH_CLI  ], pcInputString[ MAX_INPUT_LENGTH_CLI ];
static char pcLastCommand[ MAX_INPUT_LENGTH_CLI ];
static bool isEscapeCode = false;
static char pcEscapeCodes [4];
static uint8_t pcEscapeCodePos = 0;

    /* This code assumes the peripheral being used as the console has already
    been opened and configured, and is passed into the task as the task
    parameter.  Cast the task parameter to the correct type. */

    /* Send a welcome message to the user knows they are connected. */
    SerialConsoleWriteString( pcWelcomeMessage);

	//Any semaphores/mutexes/etc you needed to be initialized, you can do them here
	cliCharReadySemaphore = xSemaphoreCreateBinary();
	if(cliCharReadySemaphore == NULL)
	{
		LogMessage(LOG_ERROR_LVL, "Could not allocate semaphore\r\n");
		vTaskSuspend( NULL );
	}


    for( ;; )
    {

	FreeRTOS_read(&cRxedChar[0]);

	if( cRxedChar[0] == '\n' || cRxedChar[0] == '\r'  )
        {
            /* A newline character was received, so the input command string is
            complete and can be processed.  Transmit a line separator, just to
            make the output easier to read. */
            SerialConsoleWriteString("\r\n");
			//Copy for last command
			isEscapeCode = false; pcEscapeCodePos = 0;
			strncpy(pcLastCommand, pcInputString, MAX_INPUT_LENGTH_CLI-1);
			pcLastCommand[MAX_INPUT_LENGTH_CLI-1] = 0;	//Ensure null termination

            /* The command interpreter is called repeatedly until it returns
            pdFALSE.  See the "Implementing a command" documentation for an
            explanation of why this is. */
            do
            {
                /* Send the command string to the command interpreter.  Any
                output generated by the command interpreter will be placed in the
                pcOutputString buffer. */
                xMoreDataToFollow = FreeRTOS_CLIProcessCommand
                              (
                                  pcInputString,   /* The command string.*/
                                  pcOutputString,  /* The output buffer. */
                                  MAX_OUTPUT_LENGTH_CLI/* The size of the output buffer. */
                              );

                /* Write the output generated by the command interpreter to the
                console. */
				//Ensure it is null terminated
				pcOutputString[MAX_OUTPUT_LENGTH_CLI - 1] = 0;
                SerialConsoleWriteString(pcOutputString);

            } while( xMoreDataToFollow != pdFALSE );

            /* All the strings generated by the input command have been sent.
            Processing of the command is complete.  Clear the input string ready
            to receive the next command. */
            cInputIndex = 0;
            memset( pcInputString, 0x00, MAX_INPUT_LENGTH_CLI );
			memset( pcOutputString, 0, MAX_OUTPUT_LENGTH_CLI);
        }
        else
        {
		            /* The if() clause performs the processing after a newline character
            is received.  This else clause performs the processing if any other
            character is received. */
		
			if (true == isEscapeCode) {

				if(pcEscapeCodePos < CLI_PC_ESCAPE_CODE_SIZE) {
					pcEscapeCodes[pcEscapeCodePos++] = cRxedChar[0];
				}
				else {
					isEscapeCode = false; pcEscapeCodePos = 0;
				}
			
				if (pcEscapeCodePos >= CLI_PC_MIN_ESCAPE_CODE_SIZE) {
				
					// UP ARROW SHOW LAST COMMAND
					if(strcasecmp(pcEscapeCodes, "oa"))	{
                            /// Delete current line and add prompt (">")
                            sprintf(pcInputString, "%c[2K\r>", 27);
				            SerialConsoleWriteString((char*)pcInputString);
                            /// Clear input buffer
                            cInputIndex = 0;
                            memset( pcInputString, 0x00, MAX_INPUT_LENGTH_CLI );
                        /// Send last command
						strncpy(pcInputString, pcLastCommand, MAX_INPUT_LENGTH_CLI - 1); 	
                        cInputIndex = (strlen(pcInputString) < MAX_INPUT_LENGTH_CLI - 1) ? strlen(pcLastCommand) : MAX_INPUT_LENGTH_CLI - 1;
						SerialConsoleWriteString(pcInputString);
					}
				
					isEscapeCode = false; pcEscapeCodePos = 0;
				}			
			}
            /* The if() clause performs the processing after a newline character
            is received.  This else clause performs the processing if any other
            character is received. */

            else if( cRxedChar[0] == '\r' )
            {
                /* Ignore carriage returns. */
            }
            else if( cRxedChar[0] == ASCII_BACKSPACE || cRxedChar[0] == ASCII_DELETE )
            {
				char erase[4] = {0x08, 0x20, 0x08, 0x00};
				SerialConsoleWriteString(erase);
                /* Backspace was pressed.  Erase the last character in the input
                buffer - if there are any. */
                if( cInputIndex > 0 )
                {
                    cInputIndex--;
                    pcInputString[ cInputIndex ] = 0;
                }
            }
			// ESC
			else if( cRxedChar[0] == ASCII_ESC) {
				isEscapeCode = true; //Next characters will be code arguments
				pcEscapeCodePos = 0;
			}
            else
            {
                /* A character was entered.  It was not a new line, backspace
                or carriage return, so it is accepted as part of the input and
                placed into the input buffer.  When a n is entered the complete
                string will be passed to the command interpreter. */
                if( cInputIndex < MAX_INPUT_LENGTH_CLI )
                {
                    pcInputString[ cInputIndex ] = cRxedChar[0];
                    cInputIndex++;
                }

					//Order Echo
					cRxedChar[1] = 0;
					SerialConsoleWriteString(&cRxedChar[0]);
            }
        }
    }
}



/**************************************************************************//**
* @fn			void FreeRTOS_read(char* character)
* @brief		This function block the thread unless we received a character
* @details		This function blocks until UartSemaphoreHandle is released to continue reading characters in CLI
* @note
*****************************************************************************/
static void FreeRTOS_read(char* character)
{


//We check if there are more characters in the buffer that arrived since the last time
//This function returns -1 if the buffer is empty, other value otherwise
int ret = SerialConsoleReadCharacter(character);


if(ret == -1)
{
	//there are no more characters - block the thread until we receive a semaphore indicating reception of at least 1 character
	xSemaphoreTake(cliCharReadySemaphore, portMAX_DELAY);

	//If we are here it means there are characters in the buffer - we re-read from the buffer to get the newly acquired character
	SerialConsoleReadCharacter(character);
}


}


/**************************************************************************//**
* @fn			void CliCharReadySemaphoreGiveFromISR(void)
* @brief		Give cliCharReadySemaphore binary semaphore from an ISR
* @details		
* @note
*****************************************************************************/
void CliCharReadySemaphoreGiveFromISR(void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( cliCharReadySemaphore, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/******************************************************************************
* CLI Functions - Define here
******************************************************************************/
//Example CLI Command. Reads from the IMU and returns data.
BaseType_t CLI_GetImuData( int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString )
{
static int16_t  data_raw_acceleration[3];
static int16_t  data_raw_angular_rate;
static float acceleration_mg[3];
uint8_t reg;
stmdev_ctx_t *dev_ctx = GetImuStruct();


/* Read output only if new xl value is available */
lsm6dso_xl_flag_data_ready_get(dev_ctx, &reg);

if(reg){
	memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	lsm6dso_acceleration_raw_get(dev_ctx, data_raw_acceleration);
	acceleration_mg[0] =
	lsm6dso_from_fs2_to_mg(data_raw_acceleration[0]);
	acceleration_mg[1] =
	lsm6dso_from_fs2_to_mg(data_raw_acceleration[1]);
	acceleration_mg[2] =
	lsm6dso_from_fs2_to_mg(data_raw_acceleration[2]);

	snprintf(pcWriteBuffer,xWriteBufferLen, "Acceleration [mg]:X %d\tY %d\tZ %d\r\n",
	(int)acceleration_mg[0], (int)acceleration_mg[1], (int)acceleration_mg[2]);
}else
{
	snprintf(pcWriteBuffer,xWriteBufferLen, "No data ready! \r\n");
}
return pdFALSE;
}


//THIS COMMAND USES vt100 TERMINAL COMMANDS TO CLEAR THE SCREEN ON A TERMINAL PROGRAM LIKE TERA TERM
//SEE http://www.csie.ntu.edu.tw/~r92094/c++/VT100.html for more info
//CLI SPECIFIC COMMANDS
static char bufCli[CLI_MSG_LEN];
BaseType_t xCliClearTerminalScreen( char *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString )
{
	char clearScreen = ASCII_ESC;
	snprintf(bufCli, CLI_MSG_LEN - 1, "%c[2J", clearScreen);
	snprintf(pcWriteBuffer, xWriteBufferLen, bufCli);
	return pdFALSE;
}


//Example CLI Command. Resets system.
BaseType_t CLI_ResetDevice( int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString )
{
	system_reset();
	return pdFALSE;
}


BaseType_t CLI_GetTemp(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	//delay_init();
    static uint16_t temperature;
    static float temp_celc;
	ds18b20_init();
	ds18b20_start_conversion();
	delay_ms(750);
	temperature = ds18b20_read_temp();
	delay_ms(1000);
	temp_celc = temperature / 16;
	

	//does not print for some reason
    snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Temperature: %u \r\n", (uint16_t)temp_celc);
    return pdFALSE;
}




BaseType_t CLI_Feed(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{

	//delay_init();
	configure_clocks();
	configure_tcc_for_pwm();
	configure_pwm_pin();
	
	servo_set(50000);
	delay_ms(1000);
	servo_set(100000);
	//does not print for some reason
	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Servo Actuating...\r\n");
	return pdFALSE;
}


BaseType_t CLI_DistanceSensorGetDistance(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
    uint16_t distance = 0;
    int error = DistanceSensorGetDistance(&distance, 100);
    if (0 != error) {
        snprintf((char *) pcWriteBuffer, xWriteBufferLen, "Sensor Error %d!\r\n", error);
    } else {
        snprintf((char *) pcWriteBuffer, xWriteBufferLen, "Distance: %d mm\r\n", distance);
    }

    error = WifiAddDistanceDataToQueue(&distance);
    if (error == pdTRUE) {
        strcat((char *) pcWriteBuffer, "Distance Data MQTT Post\r\n");
    }
    return pdFALSE;
}

/**************************************************************************//**
BaseType_t CLI_NeotrellisSetLed( int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString )
* @brief	CLI command to turn on a given LED to a given R,G,B, value
* @param[out] *pcWriteBuffer. Buffer we can use to write the CLI command response to! See other CLI examples on how we use this to write back!
* @param[in] xWriteBufferLen. How much we can write into the buffer
* @param[in] *pcCommandString. Buffer that contains the complete input. You will find the additional arguments, if needed. Please see 
https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_CLI/FreeRTOS_Plus_CLI_Implementing_A_Command.html#Example_Of_Using_FreeRTOS_CLIGetParameter
Example 3
                				
* @return		Returns pdFALSE if the CLI command finished.
* @note         Please see https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_CLI/FreeRTOS_Plus_CLI_Accessing_Command_Line_Parameters.html
				for more information on how to use the FreeRTOS CLI.

*****************************************************************************/
BaseType_t CLI_NeotrellisSetLed( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString ) {
	int8_t *pcParameter;
	BaseType_t xParameterStringLength;
	uint32_t ulParamValue;
	uint8_t keynum, red, green, blue;

	/* Getting the first parameter: Keynum */
	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
	ulParamValue = strtoul((char *)pcParameter, NULL, 10);
	if (ulParamValue < 16) {
		keynum = (uint8_t)ulParamValue;
		} else {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Keynum parameter out of range (0-15).\r\n");
		return pdFALSE;
	}

	/* Getting the second parameter: Red */
	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
	ulParamValue = strtoul((char *)pcParameter, NULL, 10);
	if (ulParamValue < 256) {
		red = (uint8_t)ulParamValue;
		} else {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Red parameter out of range (0-255).\r\n");
		return pdFALSE;
	}

	/* Getting the third parameter: Green */
	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 3, &xParameterStringLength);
	ulParamValue = strtoul((char *)pcParameter, NULL, 10);
	if (ulParamValue < 256) {
		green = (uint8_t)ulParamValue;
		} else {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Green parameter out of range (0-255).\r\n");
		return pdFALSE;
	}

	/* Getting the fourth parameter: Blue */
	pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 4, &xParameterStringLength);
	ulParamValue = strtoul((char *)pcParameter, NULL, 10);
	if (ulParamValue < 256) {
		blue = (uint8_t)ulParamValue;
		} else {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Blue parameter out of range (0-255).\r\n");
		return pdFALSE;
	}

	/* If we reach this point, all parameters are correct and we can set the LED */
	int32_t error = SeesawSetLed(keynum, red, green, blue);
	if (error != ERROR_NONE) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error setting LED: %d\r\n", error);
		return pdFALSE;
	}

	/* Update the LEDs with the new settings */
	error = SeesawOrderLedUpdate();
	if (error != ERROR_NONE) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error updating LED: %d\r\n", error);
		return pdFALSE;
	}

	/* show to screen */
	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "LED %d set to R:%d G:%d B:%d\r\n", keynum, red, green, blue);


	return pdFALSE;
}

