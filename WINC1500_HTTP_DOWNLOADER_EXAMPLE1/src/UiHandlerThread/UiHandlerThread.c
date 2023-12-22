 /**************************************************************************//**
* @file      UiHandlerThread.c
* @brief     File that contains the task code and supporting code for the UI Thread for ESE516 Spring (Online) Edition
* @author    You! :)
* @date      2020-04-09 

******************************************************************************/


/******************************************************************************
* Includes
******************************************************************************/
#include <errno.h>
#include "asf.h"
#include "UiHandlerThread/UiHandlerThread.h"
#include "SeesawDriver/Seesaw.h"
#include "SerialConsole.h"
#include "main.h"

/******************************************************************************
* Defines
******************************************************************************/

/******************************************************************************
* Variables
******************************************************************************/
uiStateMachine_state uiState;
/******************************************************************************
* Forward Declarations
******************************************************************************/

/******************************************************************************
* Callback Functions
******************************************************************************/


/******************************************************************************
* Task Function
******************************************************************************/

/**************************************************************************//**
* @fn		void vUiHandlerTask( void *pvParameters )
* @brief	STUDENT TO FILL THIS
* @details 	student to fill this
                				
* @param[in]	Parameters passed when task is initialized. In this case we can ignore them!
* @return		Should not return! This is a task defining function.
* @note         
*****************************************************************************/
void vUiHandlerTask( void *pvParameters )
{
//Do initialization code here
SerialConsoleWriteString("UI Task Started!");
uiState = UI_STATE_HANDLE_BUTTONS;

//Here we start the loop for the UI State Machine
while(1)
{
	switch(uiState)
	{
		case(UI_STATE_HANDLE_BUTTONS):
		{
			    // Step 1: Query the number of button events
			    uint8_t count = SeesawGetKeypadCount();
			    if(count != 0) {
				    // Allocate buffer to read button events
				    uint8_t buffer[count]; 

				    // Step 2: Read the button events
				    SeesawReadKeypad(buffer, count);

				    // Step 3: Handle each button event
				    for(uint8_t i = 0; i < count; i++) {
					    uint8_t event = buffer[i];
					    uint8_t keyNumber = (event >> 2) & 0x3F; // Extract key number
					    uint8_t action = event & 0x03;           // Extract action

					    // Convert to 0-15 range
					    keyNumber = NEO_TRELLIS_SEESAW_KEY(keyNumber);

					    // Check action and set LED accordingly
					    if(action == SEESAW_KEYPAD_EDGE_RISING) { // Button press
						    SeesawSetLed(keyNumber, 255, 0, 0); 
						    } else if(action == SEESAW_KEYPAD_EDGE_FALLING) { // Button release
						    SeesawSetLed(keyNumber, 0, 0, 0); // Turn off LED
					    }
				    }

				    // Step 4: Update the LEDs
				    SeesawOrderLedUpdate();
			    }


		break;
		}

		case(UI_STATE_IGNORE_PRESSES):
		{
		//Ignore me for now
			break;
		}

		case(UI_STATE_SHOW_MOVES):
		{
		//Ignore me as well
			break;
		}

		default: //In case of unforseen error, it is always good to sent state machine to an initial state.
			uiState = UI_STATE_HANDLE_BUTTONS;
		break;
	}

	//After execution, you can put a thread to sleep for some time.
	vTaskDelay(50);
}



}




/******************************************************************************
* Functions
******************************************************************************/