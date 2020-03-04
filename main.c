/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

/*char readBuffer[1] = {};


void USART0_Callback(uintptr_t context) {
    if(USART0_ErrorGet() != USART_ERROR_NONE) {
        //Handle error case
    } else {
        //USART0_Write(readBuffer, 1);
        LED_AH_Toggle();
        
        USART0_Read(&readBuffer, 1);
    }
}*/

/** Callback function for TC0 CH0 */
/*void TC0_CH0_TimerInterruptHandler(TC_TIMER_STATUS status, uintptr_t context) {
    char character = 'a';
    USART0_Write(&character, 1);
}*/

int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    /*USART0_ReadCallbackRegister(USART0_Callback, (uintptr_t)NULL);
    USART0_Read(&readBuffer, 1);*/
    
    /** Initialze timer0 channel0 for testing purposes */
    /*TC0_CH0_TimerCallbackRegister(TC0_CH0_TimerInterruptHandler, (uintptr_t)NULL);
    TC0_CH0_TimerStart();*/

    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

