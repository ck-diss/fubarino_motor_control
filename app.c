/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

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
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
//uint8_t APP_MAKE_BUFFER_DMA_READY switchPromptUSB[] = "\r\nPUSH BUTTON PRESSED";

uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
uint8_t APP_MAKE_BUFFER_DMA_READY writeBuffer[4];
uint8_t APP_MAKE_BUFFER_DMA_READY writeBufferTemp[4];
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************




/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
    USB_DEVICE_CDC_INDEX index ,
    USB_DEVICE_CDC_EVENT event ,
    void * pData,
    uintptr_t userData
)
{
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *)userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch ( event )
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *)pData)->breakDuration;
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler ( USB_DEVICE_EVENT event, void * eventData, uintptr_t context )
{
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch ( event )
    {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */
            //BSP_LEDOn ( APP_USB_LED_1 );
            ///BSP_LEDOn ( APP_USB_LED_2 );
            ///BSP_LEDOff ( APP_USB_LED_3 );

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuratio. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData;
            if ( configuredEventData->configurationValue == 1)
            {
                /* Update LED to show configured state */
                ///BSP_LEDOff ( APP_USB_LED_1 );
                ///BSP_LEDOff ( APP_USB_LED_2 );
                ///BSP_LEDOn ( APP_USB_LED_3 );

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t)&appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            ///BSP_LEDOff ( APP_USB_LED_1 );
            ///BSP_LEDOn ( APP_USB_LED_2 );
            ///BSP_LEDOn ( APP_USB_LED_3 );
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void)
{
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if(appData.isConfigured == false)
    {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    }
    else
    {
        retVal = false;
    }

    return(retVal);
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID ;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 115200; //was 9600
    //appData.getLineCodingData.bParityType =  0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Initialize Ignore switch flag */
    appData.ignoreSwitchPress = false;

    /* Reset the switch debounce counter */
    appData.switchDebounceTimer = 0;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];
    appData.writeBuffer = &writeBuffer[0];
    appData.writeBufferTemp = &writeBufferTemp[0];
}



void setCurrPWM_FH (void) {
    LED = LED_ON;
    ENABLE_MOTOR_FH = !LOCKED;
    fh_stopCondition = false;
    if (appData.readBuffer [1]){
        // if 99% -> take max value
        if (appData.readBuffer [1]=='9' && appData.readBuffer [2]=='9'){
            curr_pwm_FH = PWM_MAX_FH;
        }
        else {
            // PWM * percentage received via serial port
            curr_pwm_FH = PWM_MAX_FH * ((appData.readBuffer [1]-'0')*10 + (appData.readBuffer [2]-'0')) / 100;
            if (curr_pwm_FH<PWM_MIN_FH) curr_pwm_FH = PWM_MIN_FH;
        }
        //PLIB_OC_PulseWidth16BitSet(OC_ID_FH, curr_pwm_FH);
    }
    prostActivatedByCommand=true;
}

void setCurrPWM_MAXON (void) {
    LED = LED_ON;
    ENABLE_MOTOR_MAXON = !LOCKED;
    maxon_stopCondition = false;
    if (appData.readBuffer [1]){
        // if 99% -> take max value
        if (appData.readBuffer [1]=='9' && appData.readBuffer [2]=='9'){
            curr_pwm_Maxon = PWM_MAX_MAXON;
        }
        else {
            // PWM * percentage received via serial port
            curr_pwm_Maxon = PWM_MAX_MAXON * ((appData.readBuffer [1]-'0')*10 + (appData.readBuffer [2]-'0')) / 100;
            if (curr_pwm_Maxon<PWM_MIN_MAXON) curr_pwm_Maxon = PWM_MIN_MAXON;
        }
        //PLIB_OC_PulseWidth16BitSet(OC_ID_FH, curr_pwm_FH);
    }
    prostActivatedByCommand=true;
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* Update the application state machine based
     * on the current state */

    switch(appData.state)
    {
        case APP_STATE_INIT:
            DRV_TMR0_Start();   //
            DRV_TMR1_Start();   //
            DRV_TMR2_Start();   //
            DRV_TMR3_Start();   //
            //FAULHABER
            DRV_OC0_Start();    //PWM Faulhaber
            E_FH_now = 0;
            E_FH_old = 0;
            ench_motorFH_start = UPPER_LIMIT_FH; //Lower limit of the motor 1 7.000 (100.000 = 3cm)
            ench_motorFH_end = LOWER_LIMIT_FH; //Upper limit of the motor 1 350.000 (wert war 330000) neu war 280000
            ench_motorFH_slow_down_buffer = 5000;
            motor_FH_initialized = 0;
            curr_pwm_FH = PWM_MIN_FH;
            maxon_stopCondition = true;
            fh_stopCondition = true;
            
            counter_encoder_emergency = 1;
            //MAXON
            DRV_OC1_Start();
            E_Maxon_old = 0;
            E_Maxon_now = 0;
            ench_motorMaxon_start = UPPER_LIMIT_MAXON; //Lower limit of the motor 2
            ench_motorMaxon_end = LOWER_LIMIT_MAXON; //Upper limit of the motor 2
            motor_Maxon_initialized = false;
            
            //GREIFER
            
            GREIFER_OPEN = false;
            GREIFER_CLOSE = false;
            
            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

            if(appData.deviceHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if(appData.isConfigured)
            {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if(APP_StateReset())
            {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if(appData.isReadComplete == true)
            {
                appData.isReadComplete = false;
                appData.readTransferHandle =  USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read (USB_DEVICE_CDC_INDEX_0,
                        &appData.readTransferHandle, appData.readBuffer,
                        APP_READ_BUFFER_SIZE);
                
                if(appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID)
                {
                    appData.state = APP_STATE_ERROR;
                    break;
                }
            }

            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
        case APP_STATE_CHECK_SWITCH_PRESSED:

            if(APP_StateReset())
            {
                break;
            }

            //APP_ProcessSwitchPress();

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

            if(appData.isReadComplete || appData.isSwitchPressed)
            {
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }

            break;


        case APP_STATE_SCHEDULE_WRITE:

            if(APP_StateReset())
            {
                break;
            }

            /* Setup the write */

            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;

//            if(appData.isSwitchPressed)
//            {
//                /* If the switch was pressed, then send the switch prompt*/
//                appData.isSwitchPressed = false;
//                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
//                        &appData.writeTransferHandle, switchPromptUSB, 23,
//                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
//            }
//            else
//            {
                /* Else echo the received character + 1*/
//                appData.readBuffer[0] = appData.readBuffer[0];  //original code: "+ 1"
            
            //NOTWENDIG?
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle,
                        appData.readBuffer, 1,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
//            }        
                
                
                // input from c# program
                if (appData.readBuffer [0] == 'z') {
                    LED = LED_OFF;
                }
                if (appData.readBuffer [0] == 'i') {
                    LED = LED_ON;
                }
                //OPEN Greifer
                if (appData.readBuffer [0] == 'o') {
                    GREIFER_OPEN = true;
                    GREIFER_CLOSE = false;
                }
                //CLOSE Greifer
                if (appData.readBuffer [0] == 'c') {
                    GREIFER_OPEN = false;
                    GREIFER_CLOSE = true;     
                }
                //STOP Greifer
                if (appData.readBuffer [0] == '#') {
                    GREIFER_OPEN = false;
                    GREIFER_CLOSE = false;
                }
                //FAULHABER:
                //stop 's'
                if (motor_FH_initialized == true) {
                    if (appData.readBuffer [0] == 's')
                    { 
                       LED = LED_OFF;
                       fh_stopCondition = true;
                       ENABLE_MOTOR_FH = LOCKED;
                       PLIB_OC_PulseWidth16BitSet(OC_ID_FH, 0);
                       prostActivatedByCommand=true;
                    }
                    // extension 'e'
                    if (appData.readBuffer [0] == 'e' && motorFH_upper_limit_reached == false)
                    { 
                        ROTARY_DIRECTION_MOTOR_FH = DIRECTION_UP_FH;  // to the "emergency button" -> elbow down
                        // PWM based on contraction strength
                        setCurrPWM_FH();
                    } 
                    // flexion 'f'
                    if (appData.readBuffer [0] == 'f' && motorFH_lower_limit_reached == false)
                    { 
                        ROTARY_DIRECTION_MOTOR_FH = DIRECTION_DOWN_FH;  // away from the "proxymity sensor"
                        // PWM based on contraction strength
                        setCurrPWM_FH();
                    }  
                }
                //MAXON
                //stop wrist movement 'w'
                if (motor_Maxon_initialized == true) {
                    if (appData.readBuffer [0] == 'h')
                    {
                       LED = LED_OFF;
                       ENABLE_MOTOR_MAXON = LOCKED; 
                       PLIB_OC_PulseWidth16BitSet(OC_ID_MAXON, 0);
                       maxon_stopCondition = true;
                       prostActivatedByCommand=true;
                    }
                    // pronation 'p'
                    if (appData.readBuffer [0] == 'p' && maxon_upper_limit_reached == false)
                    { 
                        ROTARY_DIRECTION_MOTOR_MAXON = DIRECTION_UP_MAXON;  // to the "emergency button" -> elbow down
                        // PWM based on contraction strength
                        setCurrPWM_MAXON();      
                    }
                    //supination 'u'
                    if (appData.readBuffer [0] == 'u' && maxon_lower_limit_reached == false) 
                    { 
                        ROTARY_DIRECTION_MOTOR_MAXON = DIRECTION_DOWN_MAXON;  // to the "emergency button" -> elbow down
                        // PWM based on contraction strength
                        setCurrPWM_MAXON();
                    }
                }
                if (motor_Maxon_initialized == true && motor_FH_initialized == true) {
                    if (appData.readBuffer [0] == ',') {
                        ROTARY_DIRECTION_MOTOR_MAXON = DIRECTION_UP_MAXON;
                        ROTARY_DIRECTION_MOTOR_FH = DIRECTION_UP_FH;

                        curr_pwm_Maxon = PWM_MAX_MAXON;
                        curr_pwm_FH = PWM_MAX_FH;
                        
                        ENABLE_MOTOR_FH = !LOCKED;
                        ENABLE_MOTOR_MAXON = !LOCKED;
                        fh_stopCondition = false;
                        maxon_stopCondition = false;
                    }
                    if (appData.readBuffer [0] == '.' && maxon_lower_limit_reached == false && motorFH_lower_limit_reached == false) {
                        ROTARY_DIRECTION_MOTOR_MAXON = DIRECTION_DOWN_MAXON;
                        ROTARY_DIRECTION_MOTOR_FH = DIRECTION_DOWN_FH;

                        curr_pwm_Maxon = PWM_MAX_MAXON;
                        curr_pwm_FH = PWM_MAX_FH;
                        
                        ENABLE_MOTOR_FH = !LOCKED;
                        ENABLE_MOTOR_MAXON = !LOCKED;
                        fh_stopCondition = false;
                        maxon_stopCondition = false;                    
                    }   
                }
            break;

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if(APP_StateReset())
            {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if(appData.isWriteComplete == true)
            {
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}
 


/*******************************************************************************
 End of File
 */
