/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    usb.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "USB_Initialize" and "USB_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "USB_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _USB_H
#define _USB_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "configuration.h"
#include "definitions.h"
#include "FreeRTOS.h"
#include "task.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END
    
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
#define USB_HOST_CDC_BAUDRATE_SUPPORTED 9600UL
#define USB_HOST_CDC_PARITY_TYPE        0
#define USB_HOST_CDC_STOP_BITS          0
#define USB_HOST_CDC_NO_OF_DATA_BITS    8

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    /* Application enabled the bus*/
    USB_STATE_BUS_ENABLE,
            
    /* Application waits for bus to be enabled */
    USB_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE,

    /* Application waits for CDC Device Attach */
    USB_STATE_WAIT_FOR_DEVICE_ATTACH,

    /* CDC Device is Attached */
    USB_STATE_OPEN_DEVICE,
            
    /* Set the Line Coding */
    USB_STATE_SET_LINE_CODING,

    /* Application waits to get the device line coding */
    USB_STATE_WAIT_FOR_GET_LINE_CODING,

    /* Application sets the line coding */
    USB_STATE_SEND_SET_LINE_CODING,

    /* Appliction waits till set line coding is done */
    USB_STATE_WAIT_FOR_SET_LINE_CODING,

    /* Application sets the contol line state */
    USB_STATE_SEND_SET_CONTROL_LINE_STATE,

    /* Application waits for the set control line state to complete */
    USB_STATE_WAIT_FOR_SET_CONTROL_LINE_STATE,
            
    /* Application sends the prompt to the device */
    USB_STATE_SEND_PROMPT_TO_DEVICE,

    /* Application waits for prompt send complete */
    USB_STATE_WAIT_FOR_PROMPT_SEND_COMPLETE,

    /* Application request to get data from device */
    USB_STATE_GET_DATA_FROM_DEVICE,

    /* Application waits for data from device */
    USB_STATE_WAIT_FOR_DATA_FROM_DEVICE,

    /* Application has received data from the device */
    USB_STATE_DATA_RECEIVED_FROM_DEVICE,

    /* Application is in error state */
    USB_STATE_ERROR,
            
    USB_STATE_INIT_MODEM_AT,
            
    USB_STATE_INIT_MODEM_ATE0,
            
    USB_STATE_INIT_MODEM_CMEE   

} USB_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* First place to be aligned. 
     * Array to hold read data */
    char inDataArray[64];
    
    /* The application's current state */
    USB_STATES state;
    
    /* The application's previous state */
    USB_STATES previousSenderState;

    /* CDC Object */
    USB_HOST_CDC_OBJ cdcObj;
    
    /* True if a device is attached */
    bool deviceIsAttached;
    
    /* True if control request is done */
    bool controlRequestDone;
    
    /* Control Request Result */
    USB_HOST_CDC_RESULT controlRequestResult;

    /* A CDC Line Coding object */
    USB_CDC_LINE_CODING cdcHostLineCoding;
    
    /* A Control Line State object*/
    USB_CDC_CONTROL_LINE_STATE controlLineState;
    
    /* Handle to the CDC device. */
    USB_HOST_CDC_HANDLE cdcHostHandle;
    
    USB_HOST_CDC_REQUEST_HANDLE  requestHandle;
    
    /* True when a write transfer has complete */
    bool writeTransferDone;
    
    /* Write Transfer Result */
    USB_HOST_CDC_RESULT writeTransferResult;
    
     /* True when a read transfer has complete */
    bool readTransferDone;
    
    /* Read Transfer Result */
    USB_HOST_CDC_RESULT readTransferResult;
    
    /* True if device was detached */
    bool deviceWasDetached;

} USB_DATA;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

USB_HOST_EVENT_RESPONSE USB_USBHostEventHandler (USB_HOST_EVENT event, void * eventData, uintptr_t context);

void USB_USBHostCDCAttachEventListener(USB_HOST_CDC_OBJ cdcObj, uintptr_t context);

USB_HOST_CDC_EVENT_RESPONSE USB_USBHostCDCEventHandler(USB_HOST_CDC_HANDLE cdcHandle,USB_HOST_CDC_EVENT event,void * eventData,uintptr_t context);

void USART0_log(char* sender, char* msg);

void USART0_log_size(char* sender, char* msg, int msg_size);

void clear_array(char* array, size_t size);

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void USB_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    USB_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    USB_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void USB_Initialize ( void );


/*******************************************************************************
  Function:
    void USB_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    USB_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void USB_Tasks( void );



#endif /* _USB_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

