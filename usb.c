/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    usb.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 ******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "usb.h"
#include <string.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the USB_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

USB_DATA USB_ALIGN usbData;
 
 int log_msg_sent = 0;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

USB_HOST_EVENT_RESPONSE USB_USBHostEventHandler (USB_HOST_EVENT event, void * eventData, uintptr_t context) {
    /* This function is called by the USB Host whenever a USB Host Layer event
     * has occurred. In this example we only handle the device unsupported event
     * */

    switch (event)
    {
        case USB_HOST_EVENT_DEVICE_UNSUPPORTED:
            
            /* The attached device is not supported for some reason */
            break;
            
        default:
            break;
                    
    }
    
    return(USB_HOST_EVENT_RESPONSE_NONE);
}

void USB_USBHostCDCAttachEventListener(USB_HOST_CDC_OBJ cdcObj, uintptr_t context) {
    /* This function gets called when the CDC device is attached. Update the
     * application data structure to let the application know that this device
     * is attached */
    
    usbData.deviceIsAttached = true;
    usbData.cdcObj = cdcObj;
}

USB_HOST_CDC_EVENT_RESPONSE USB_USBHostCDCEventHandler(USB_HOST_CDC_HANDLE cdcHandle,USB_HOST_CDC_EVENT event,void * eventData,uintptr_t context) {
    /* This function is called when a CDC Host event has occurred. A pointer to
     * this function is registered after opening the device. See the call to
     * USB_HOST_CDC_EventHandlerSet() function. */

    USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE_DATA * setLineCodingEventData;
    USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE_DATA * setControlLineStateEventData;
    USB_HOST_CDC_EVENT_WRITE_COMPLETE_DATA * writeCompleteEventData;
    USB_HOST_CDC_EVENT_READ_COMPLETE_DATA * readCompleteEventData;
    
    switch(event)
    {
        case USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE:
            
            /* This means the application requested Set Line Coding request is
             * complete. */
            setLineCodingEventData = (USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE_DATA *)(eventData);
            usbData.controlRequestDone = true;
            usbData.controlRequestResult = setLineCodingEventData->result;
            break;
            
        case USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE:
            
            /* This means the application requested Set Control Line State 
             * request has completed. */
            setControlLineStateEventData = (USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE_DATA *)(eventData);
            usbData.controlRequestDone = true;
            usbData.controlRequestResult = setControlLineStateEventData->result;
            break;
            
        case USB_HOST_CDC_EVENT_WRITE_COMPLETE:
            
            /* This means an application requested write has completed */
            usbData.writeTransferDone = true;
            writeCompleteEventData = (USB_HOST_CDC_EVENT_WRITE_COMPLETE_DATA *)(eventData);
            usbData.writeTransferResult = writeCompleteEventData->result;
            break;
            
        case USB_HOST_CDC_EVENT_READ_COMPLETE:
            
            /* This means an application requested write has completed */
            usbData.readTransferDone = true;
            readCompleteEventData = (USB_HOST_CDC_EVENT_READ_COMPLETE_DATA *)(eventData);
            usbData.readTransferResult = readCompleteEventData->result;
            break;
            
        case USB_HOST_CDC_EVENT_DEVICE_DETACHED:
            
            /* The device was detached */
            usbData.deviceWasDetached = true;
            break;
            
        default:
            break;
    }
    
    return(USB_HOST_CDC_EVENT_RESPONE_NONE);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/

void USART0_log(char* sender, char* msg) {
    char message[100];
    sprintf(message, "[%s] %s\n\r", sender, msg);
    USART0_Write(message, strlen(sender) + strlen(msg) + 5);
    
    vTaskDelay(1000 * configTICK_RATE_HZ / 1000);
}

void USART0_log_size(char* sender, char* msg, int msg_size) {
    char message[100];
    sprintf(message, "[%s] %s\n\r", sender, msg);
    USART0_Write(message, strlen(sender) + msg_size + 5);
    
    vTaskDelay(1000 * configTICK_RATE_HZ / 1000);
}

void clear_array(char* array, size_t size) {
    char new_array[1] = {};
    for (int i = 0; i < size; i++) {
        array[i] = new_array[0];
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void USB_Initialize ( void )

  Remarks:
    See prototype in usb.h.
 */

void USB_Initialize ( void )
{
    /* Initialize the application state machine */
    
    usbData.state =  USB_STATE_BUS_ENABLE;
    usbData.cdcHostLineCoding.dwDTERate     = USB_HOST_CDC_BAUDRATE_SUPPORTED;
    usbData.cdcHostLineCoding.bDataBits     = (uint8_t)USB_HOST_CDC_NO_OF_DATA_BITS;
    usbData.cdcHostLineCoding.bParityType   = (uint8_t)USB_HOST_CDC_PARITY_TYPE;
    usbData.cdcHostLineCoding.bCharFormat   = (uint8_t)USB_HOST_CDC_STOP_BITS;
    usbData.controlLineState.dtr = 0;
    usbData.controlLineState.carrier = 0;
    usbData.deviceIsAttached = false;
    usbData.deviceWasDetached = false;
    usbData.readTransferDone = false;
    usbData.writeTransferDone = false;
    usbData.controlRequestDone = false;
}


/******************************************************************************
  Function:
    void USB_Tasks ( void )

  Remarks:
    See prototype in usb.h.
 */

void USB_Tasks ( void )
{

    /* Check the application's current state. */
   USB_HOST_CDC_RESULT result;
   char send_msg[32];
   
   if(usbData.deviceWasDetached)
   {
       /* This means the device is not attached. Reset the application state */
       
       usbData.state = USB_STATE_WAIT_FOR_DEVICE_ATTACH;
       usbData.readTransferDone = false;
       usbData.writeTransferDone = false;
       usbData.controlRequestDone = false;
       usbData.deviceWasDetached = false;
   }
   
    switch (usbData.state)
    {
        case USB_STATE_BUS_ENABLE:
        
            /* In this state the application enables the USB Host Bus. Note
             * how the CDC Attach event handler are registered before the bus
             * is enabled. */
            if (log_msg_sent == 0) {
                USART0_log("USB", "BUS_ENABLE");
                log_msg_sent = 1;
            }
            
            USB_HOST_EventHandlerSet(USB_USBHostEventHandler, (uintptr_t)0);
            USB_HOST_CDC_AttachEventHandlerSet(USB_USBHostCDCAttachEventListener, (uintptr_t) 0);
            //char stat_usb[100];
            USB_HOST_BusEnable(0);
            //sprintf(stat_usb, "%d", usb_result);
            //USART0_log("USB", stat_usb);
            usbData.state = USB_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE;
            log_msg_sent = 0;
            
            break;
        
        case USB_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE:
            
            /* In this state we wait for the Bus enable to complete */
            
            if (log_msg_sent == 0) {
                USART0_log("USB", "WAIT_FOR_BUS_ENABLE_COMPLETE");
                log_msg_sent = 1;
            }
            
            if(USB_HOST_BusIsEnabled(0))
            {
                usbData.state = USB_STATE_WAIT_FOR_DEVICE_ATTACH;
                log_msg_sent = 0;
            }
            
            break;
            
        case USB_STATE_WAIT_FOR_DEVICE_ATTACH:
            
            /* In this state the application is waiting for the device to be
             * attached */
            
            if (log_msg_sent == 0) {
                USART0_log("USB", "WAIT_FOR_DEVICE_ATTACH");
                log_msg_sent = 1;
            }
            if(usbData.deviceIsAttached)
            {
                /* A device is attached. We can open this device */
                usbData.state = USB_STATE_OPEN_DEVICE;
                log_msg_sent = 0;
                usbData.deviceIsAttached = false;
            }
            break;
            
        case USB_STATE_OPEN_DEVICE:
            
            /* In this state the application opens the attached device */
            
            if (log_msg_sent == 0) {
                USART0_log("USB", "OPEN_DEVICE");
                log_msg_sent = 1;
            }
            
            usbData.cdcHostHandle = USB_HOST_CDC_Open(usbData.cdcObj);
            if(usbData.cdcHostHandle != USB_HOST_CDC_HANDLE_INVALID)
            {
                /* The driver was opened successfully. Set the event handler
                 * and then go to the next state. */
                USB_HOST_CDC_EventHandlerSet(usbData.cdcHostHandle, USB_USBHostCDCEventHandler, (uintptr_t)0);
                usbData.state = USB_STATE_SET_LINE_CODING;
                log_msg_sent = 0;
            }
            break;
            
        case USB_STATE_SET_LINE_CODING:
            
            /* Here we set the Line coding. The control request done flag will
             * be set to true when the control request has completed. */
            
            if (log_msg_sent == 0) {
                USART0_log("USB", "SET_LINE_CODING");
                log_msg_sent = 1;
            }
            
            usbData.controlRequestDone = false;
            result = USB_HOST_CDC_ACM_LineCodingSet(usbData.cdcHostHandle, NULL, &usbData.cdcHostLineCoding);
            
            if(result == USB_HOST_CDC_RESULT_SUCCESS)
            {
                /* We wait for the set line coding to complete */
                usbData.state = USB_STATE_WAIT_FOR_SET_LINE_CODING;
                log_msg_sent = 0;
            }
                            
            break;
            
        case USB_STATE_WAIT_FOR_SET_LINE_CODING:
            
            if (log_msg_sent == 0) {
                USART0_log("USB", "WAIT_FOR_SET_LINE_CODING");
                log_msg_sent = 1;
            }
            
            if(usbData.controlRequestDone)
            {
                if(usbData.controlRequestResult != USB_HOST_CDC_RESULT_SUCCESS)
                {
                    /* The control request was not successful. */
                    usbData.state = USB_STATE_ERROR;
                    log_msg_sent = 0;
                }
                else
                {
                    /* Next we set the Control Line State */
                    vTaskDelay(20000 * configTICK_RATE_HZ / 1000);
                    usbData.state = USB_STATE_SEND_SET_CONTROL_LINE_STATE;
                    log_msg_sent = 0;
                }
            }
            break;
            
        case USB_STATE_SEND_SET_CONTROL_LINE_STATE:
            
            /* Here we set the control line state */
            
            if (log_msg_sent == 0) {
                USART0_log("USB", "SEND_SET_CONTROL_LINE_STATE");
                log_msg_sent = 1;
            }
            
            usbData.controlRequestDone = false;
            result = USB_HOST_CDC_ACM_ControlLineStateSet(usbData.cdcHostHandle, NULL, 
                    &usbData.controlLineState);
            
            if(result == USB_HOST_CDC_RESULT_SUCCESS)
            {
                /* We wait for the set line coding to complete */
                usbData.state = USB_STATE_WAIT_FOR_SET_CONTROL_LINE_STATE;
                log_msg_sent = 0;
            }
            
            break;
            
        case USB_STATE_WAIT_FOR_SET_CONTROL_LINE_STATE:
            
            /* Here we wait for the control line state set request to complete */
            
            if (log_msg_sent == 0) {
                USART0_log("USB", "WAIT_FOR_SET_CONTROL_LINE_STATE");
                log_msg_sent = 1;
            }
            
            if(usbData.controlRequestDone)
            {
                if(usbData.controlRequestResult != USB_HOST_CDC_RESULT_SUCCESS)
                {
                    /* The control request was not successful. */
                    usbData.state = USB_STATE_ERROR;
                    log_msg_sent = 0;
                }
                else
                {
                    /* Next we set the Control Line State */
                    usbData.state = USB_STATE_INIT_MODEM_AT;
                    log_msg_sent = 0;
                }
            }
            
            break;
                
        case USB_STATE_INIT_MODEM_AT:
            
            if (log_msg_sent == 0) {
                USART0_log("USB", "INIT_MODEM_AT");
                log_msg_sent = 1;
            }
            
            usbData.writeTransferDone = false;
            
            clear_array(send_msg, 32);
            
            strcpy(send_msg, "AT\r");
            result = USB_HOST_CDC_Write(usbData.cdcHostHandle, NULL, ( void * )send_msg, strlen(send_msg));
            
            if(result == USB_HOST_CDC_RESULT_SUCCESS)
            {
                usbData.previousSenderState = USB_STATE_INIT_MODEM_AT;
                usbData.state = USB_STATE_WAIT_FOR_PROMPT_SEND_COMPLETE;
                log_msg_sent = 0;
            }
            
            break;
            
           
        case USB_STATE_INIT_MODEM_ATE0:
            if (log_msg_sent == 0) {
                USART0_log("USB", "INIT_MODEM_ATE0");
                log_msg_sent = 1;
            }
            
            usbData.writeTransferDone = false;

            clear_array(send_msg, 32);
            
            strcpy(send_msg, "ATE0\r");
            result = USB_HOST_CDC_Write(usbData.cdcHostHandle, NULL, ( void * )send_msg, strlen(send_msg));
            
            if(result == USB_HOST_CDC_RESULT_SUCCESS)
            {
                usbData.previousSenderState = USB_STATE_INIT_MODEM_ATE0;
                usbData.state = USB_STATE_WAIT_FOR_PROMPT_SEND_COMPLETE;
                log_msg_sent = 0;
            }
            
            break;
            
        case USB_STATE_INIT_MODEM_CMEE:
            if (log_msg_sent == 0) {
                USART0_log("USB", "INIT_MODEM_CMEE");
                log_msg_sent = 1;
            }
            
            usbData.writeTransferDone = false;

            clear_array(send_msg, 32);
            
            strcpy(send_msg, "AT+CMEE=2\r");
            result = USB_HOST_CDC_Write(usbData.cdcHostHandle, NULL, ( void * )send_msg, strlen(send_msg));
            
            if(result == USB_HOST_CDC_RESULT_SUCCESS)
            {
                usbData.previousSenderState = USB_STATE_INIT_MODEM_CMEE;
                usbData.state = USB_STATE_WAIT_FOR_PROMPT_SEND_COMPLETE;
                log_msg_sent = 0;
            }
            break;
            
        case USB_STATE_SEND_PROMPT_TO_DEVICE:
            
            /* The prompt is sent to the device here. The write transfer done
             * flag is updated in the event handler. */
            
            if (log_msg_sent == 0) {
                USART0_log("USB", "SEND_PROMPT_TO_DEVICE");
                log_msg_sent = 1;
            }
            
            usbData.writeTransferDone = false;
            
            
            /**
             * SET UART BAUD: AT+IPR=9600\r
             * TURN OFF ECHO: ATE0\r
             * GET VENDOR NAME: AT+CGMI\r
             * GET SIM ID: AT+CCID\r
             * SET SIM FUNCTIONALITY: AT+CFUN=
             * INITIAL SETUP: ATE0;+CMEE=2;&W0;+CPWROFF\r
             */
            
            clear_array(send_msg, 32);
            
            strcpy(send_msg, "AT+CCID\r");
            result = USB_HOST_CDC_Write(usbData.cdcHostHandle, NULL, ( void * )send_msg, strlen(send_msg));
            
            if(result == USB_HOST_CDC_RESULT_SUCCESS)
            {
                usbData.state = USB_STATE_WAIT_FOR_PROMPT_SEND_COMPLETE;
                log_msg_sent = 0;
            }
            break;
            
        case USB_STATE_WAIT_FOR_PROMPT_SEND_COMPLETE:
            
            /* Here we check if the write transfer is done */
            
            if (log_msg_sent == 0) {
                USART0_log("USB", "WAIT_FOR_PROMPT_SEND_COMPLETE");
                log_msg_sent = 1;
            }
            
            if(usbData.writeTransferDone)
            {
                if(usbData.writeTransferResult == USB_HOST_CDC_RESULT_SUCCESS)
                {
                    /* Now to get data from the device */
                    log_msg_sent = 0;
                    usbData.state = USB_STATE_GET_DATA_FROM_DEVICE;
                }
                else
                {
                    /* Try sending the prompt again. */
                    log_msg_sent = 0;
                    usbData.state = usbData.previousSenderState;                              
                }
            }
            
            break;
            
        case USB_STATE_GET_DATA_FROM_DEVICE:
            
            /* Here we request data from the device */
            
            if (log_msg_sent == 0) {
                USART0_log("USB", "GET_DATA_FROM_DEVICE");
                log_msg_sent = 1;
            }
            
            usbData.readTransferDone = false;
            result = USB_HOST_CDC_Read(usbData.cdcHostHandle, NULL, usbData.inDataArray, 64);
            if(result == USB_HOST_CDC_RESULT_SUCCESS)
            {
                usbData.state = USB_STATE_WAIT_FOR_DATA_FROM_DEVICE;
                log_msg_sent = 0;
            }
            break;
            
        case USB_STATE_WAIT_FOR_DATA_FROM_DEVICE:
            
            /* Wait for data from device. If the data has arrived, then toggle
             * the LED. */
            
            if (log_msg_sent == 0) {
                USART0_log("USB", "WAIT_FOR_DATA_FROM_DEVICE");
                log_msg_sent = 1;
            }
            
            if(usbData.readTransferDone)
            {
                if(usbData.readTransferResult == USB_HOST_CDC_RESULT_SUCCESS)
                {
                    USART0_log_size("USB-INCOMING", usbData.inDataArray, strlen(usbData.inDataArray));
                    
                    char new_array[1] = {};
                    for (int i = 0; i < 64; i++) {
                        usbData.inDataArray[i] = new_array[0];
                    }
                    log_msg_sent = 0;
                    switch (usbData.previousSenderState) {
                        case USB_STATE_INIT_MODEM_AT:
                            usbData.state = USB_STATE_INIT_MODEM_ATE0;
                            
                            break;
                            
                        case USB_STATE_INIT_MODEM_ATE0:
                            usbData.state = USB_STATE_INIT_MODEM_CMEE;
                            
                            break;
                            
                        case USB_STATE_INIT_MODEM_CMEE:
                            usbData.state = USB_STATE_SEND_PROMPT_TO_DEVICE;
                            
                            break;
                            
                        case USB_STATE_SEND_PROMPT_TO_DEVICE:
                            usbData.state = USB_STATE_GET_DATA_FROM_DEVICE;
                            
                            break;
                            
                        default:
                            usbData.state = USB_STATE_GET_DATA_FROM_DEVICE;
                            
                    }  
                }
            }
            
            break;
            
        case USB_STATE_ERROR:
            /* An error has occurred */
            
            if (log_msg_sent == 0) {
                USART0_log("USB", "ERROR");
                log_msg_sent = 1;
            }
            
            break;
            
        default:
            break;
    }
}


/*******************************************************************************
 End of File
 */
