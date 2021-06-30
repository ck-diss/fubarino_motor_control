//#include <xc.h>
//#include <sys/attribs.h>
#include "app.h"
//#include "system_definitions.h"
//#include "peripheral/oc/plib_oc.h"

APP_DATA appData;


void ProxFaulhaberInterruptCallback (void) {
    //INTERRUPT proximity sensor: stops the motor before damage the gearbox
    ENABLE_MOTOR_FH = LOCKED;
    ROTARY_DIRECTION_MOTOR_FH = DIRECTION_DOWN_FH;
    PLIB_OC_PulseWidth16BitSet(OC_ID_FH, 0); 
    motor_FH_initialized = true;
    motorFH_upper_limit_reached = true;
    // evtl problem weil nicht sofort stopped -> wert geht ins negative (-4000?)
    RCount_FH = 0;
    LED = LED_OFF;
}

void ProxMaxonInterruptCallback (void) {
    //INTERRUPT proximity sensor: stops the motor before damage the gearbox
    ENABLE_MOTOR_MAXON = LOCKED;
    ROTARY_DIRECTION_MOTOR_MAXON = DIRECTION_DOWN_MAXON;
    PLIB_OC_PulseWidth16BitSet(OC_ID_MAXON, 0); 
    motor_Maxon_initialized = true;
    maxon_upper_limit_reached = true;
    RCount_Maxon = 0;
    LED = LED_OFF;
}

void PRGButtonInterruptCallback (void) {
    motor_FH_initialized = false;
    motor_Maxon_initialized = false;
    //FAULHABER
    //INTERRUPT BUTTON_PIC: for the initialization of the Encoder press the button on the PIC32
    if (PROX_SENSOR_FH == !PROX_SENSOR_ACTIVE) //PROX_SENSOR_FH NOT activ
    {
        fh_stopCondition = false;
        //edit unnecessary BRAKE_MOTOR_FH = false;
        PLIB_OC_PulseWidth16BitSet(OC_ID_FH, PWM_MIN_FH);
        ROTARY_DIRECTION_MOTOR_FH = DIRECTION_UP_FH;
        ENABLE_MOTOR_FH = !LOCKED;
    } else //falls PROX_SENSOR_FH aktiv (Nut ganz oben) und PRG_BTN gedrückt wird
    {
        ENABLE_MOTOR_FH = LOCKED;
        fh_stopCondition = true;
        LED = LED_OFF;
        PLIB_OC_PulseWidth16BitSet(OC_ID_FH, 0); 
        motor_FH_initialized = true;
        RCount_FH = 0;
        ROTARY_DIRECTION_MOTOR_FH = DIRECTION_DOWN_FH;
    }
    //MAXON
    if (PROX_SENSOR_MAXON == !PROX_SENSOR_ACTIVE) //PROX_SENSOR NOT activ
    {
        maxon_stopCondition = false;
        PLIB_OC_PulseWidth16BitSet(OC_ID_MAXON, PWM_MIN_MAXON);
        ROTARY_DIRECTION_MOTOR_MAXON = DIRECTION_UP_MAXON;
        ENABLE_MOTOR_MAXON = !LOCKED;
    } else //falls PROX_SENSOR_FH aktiv (Nut ganz oben) und PRG_BTN gedrückt wird
    {
        ENABLE_MOTOR_MAXON = LOCKED;
        maxon_stopCondition = true;
        LED = LED_OFF;
        PLIB_OC_PulseWidth16BitSet(OC_ID_MAXON, 0); 
        motor_Maxon_initialized = true;
        RCount_Maxon = 0;
        ROTARY_DIRECTION_MOTOR_MAXON = DIRECTION_DOWN_MAXON;
    }
    prostActivatedByCommand=false;
    sprintf(appData.writeBufferTemp,"-1-1");
}

void CNInterruptCallback (void) {
    //    The CNSTATx register indicates whether a change
//    occurred on the corresponding pin since the last read
//    of the PORTx bit.
//    Faulhaber A = RC6
//    Faulhaber B = RB9
//    Maxon A     = RC1
//    Maxon B     = RC2
//     if (CNSTATCbits.CNSTATC1 == 1 || CNSTATCbits.CNSTATC2 == 1) {
//        E_Maxon_now = (PORTC & 0b00000110); //(0-0-0-0-0-0-RC2-RC1-0)
//        switch ((E_Maxon_old << 1) | (E_Maxon_now >> 1)) {
//            //0b00(old)00(now)
//            case 0b0000:    
//            case 0b0101:    
//            case 0b1010:    
//            case 0b1111:    break;   //nothing changed
//            case 0b0001:    
//            case 0b0111:    
//            case 0b1110:    
//            case 0b1000:    RCount_Maxon--;
//                            break;
//            case 0b0100:    
//            case 0b0010:    
//            case 0b1011:    
//            case 0b1101:    RCount_Maxon++;
//                            break;
// 
//            default:    //more than one bit changed
//                break;        
//        }
//        E_Maxon_old = E_Maxon_now;
//    }
//    if (CNSTATCbits.CNSTATC6 == 1 || CNSTATBbits.CNSTATB9 == 1) {
//        E_FH_now = (((PORTC & 0b01000000) >> 5) | ((PORTB >> 9) & 0b00000001)); //(0-RC6-0-0-0-0-0-0)
//
//        switch ((E_FH_old << 2) | E_FH_now) {
//            //0b00(old)00(now)
//            case 0b0000:    
//            case 0b0101:    
//            case 0b1010:    
//            case 0b1111:    break;   //nothing changed
//            case 0b0001:    
//            case 0b0111:    
//            case 0b1110:    
//            case 0b1000:    RCount_FH++;
//                            break;
//            case 0b0100:    
//            case 0b0010:    
//            case 0b1011:    
//            case 0b1101:    RCount_FH--;
//                            break;
// 
//            default:    //more than one bit changed
//                break;        
//        }
//        E_FH_old = E_FH_now;
//    }
}
void Timer5usCallback (void) {
    //FAULHABER ENCODER
        E_FH_now = (((PORTC & 0b01000000) >> 5) | ((PORTB >> 9) & 0b00000001)); //(0-RC6-0-0-0-0-0-0)

        switch ((E_FH_old << 2) | E_FH_now) {
            //0b00(old)00(now)
            case 0b0000:    
            case 0b0101:    
            case 0b1010:    
            case 0b1111:    break;   //nothing changed
            case 0b0001:    
            case 0b0111:    
            case 0b1110:    
            case 0b1000:    RCount_FH++;
                            break;
            case 0b0100:    
            case 0b0010:    
            case 0b1011:    
            case 0b1101:    RCount_FH--;
                            break;
 
            default:    //more than one bit changed
                break;        
        }
        E_FH_old = E_FH_now;
    //MAXON ENCODER
        E_Maxon_now = (PORTC & 0b00000110); //(0-0-0-0-0-0-RC2-RC1-0)
        switch ((E_Maxon_old << 1) | (E_Maxon_now >> 1)) {
            //0b00(old)00(now)
            case 0b0000:    
            case 0b0101:    
            case 0b1010:    
            case 0b1111:    break;   //nothing changed
            case 0b0001:    
            case 0b0111:    
            case 0b1110:    
            case 0b1000:    RCount_Maxon--;
                            break;
            case 0b0100:    
            case 0b0010:    
            case 0b1011:    
            case 0b1101:    RCount_Maxon++;
                            break;
 
            default:    //more than one bit changed
                break;        
        }
        E_Maxon_old = E_Maxon_now;
    
    
}
void Timer1msCallback (void) {
    // if initialized and PROX_SENSOR_FH not active
   if (motor_FH_initialized) 
    {
        // if PROX_SENSOR_FH active or count value smaller than limit -> go down
        if (RCount_FH < ench_motorFH_start) {
                ROTARY_DIRECTION_MOTOR_FH = DIRECTION_DOWN_FH;
                ENABLE_MOTOR_FH = !LOCKED;
                PLIB_OC_PulseWidth16BitSet(OC_ID_FH, PWM_MIN_FH); 
                fh_stopCondition = true;
                motorFH_upper_limit_reached = true;
        }
        // if PROX_SENSOR_FH not active and limit bigger than start_limit
        else {
            if (fh_stopCondition) {
                ENABLE_MOTOR_FH = LOCKED;
                LED = LED_OFF;
                PLIB_OC_PulseWidth16BitSet(OC_ID_FH, 0); 
            }
            else {
                // downwards
                if (ROTARY_DIRECTION_MOTOR_FH == DIRECTION_DOWN_FH) {
                    // away from the "proximity sensor"
                    if (RCount_FH < ench_motorFH_end) {
                        LED = LED_ON;
                        ENABLE_MOTOR_FH = !LOCKED;
                        motorFH_upper_limit_reached = false;

                        // reduce speed if near end
                        if (RCount_FH > (ench_motorFH_end-ench_motorFH_slow_down_buffer)) {
                            //PLIB_OC_PulseWidth16BitSet(OC_ID_FH, curr_pwm_FH - (RCount_FH-(ench_motorFH_end - ench_motorFH_slow_down_buffer))*1.4);
                            PLIB_OC_PulseWidth16BitSet(OC_ID_FH, PWM_MIN_FH);
                        }
                        else {
                            PLIB_OC_PulseWidth16BitSet(OC_ID_FH, curr_pwm_FH);
                        }
                    }
                    else {
                        LED = LED_OFF;
                        //RCount_FH = ench_motorFH_end; //test
                        fh_stopCondition = true;
                        ENABLE_MOTOR_FH = LOCKED;
                        PLIB_OC_PulseWidth16BitSet(OC_ID_FH, 0);
                        motorFH_lower_limit_reached = true;
                    }
                }
                // upwards
                if (ROTARY_DIRECTION_MOTOR_FH == DIRECTION_UP_FH) {
                    //ROTARY_DIRECTION_MOTOR_FH = 0;  // to the "proximity sensor"               
                    if (RCount_FH > ench_motorFH_start) {
                        LED = LED_ON;
                        ENABLE_MOTOR_FH = !LOCKED;
                        motorFH_lower_limit_reached = false;

                        if (RCount_FH < (ench_motorFH_start + ench_motorFH_slow_down_buffer)) {
                            //PLIB_OC_PulseWidth16BitSet(OC_ID_FH, curr_pwm_FH - ((ench_motorFH_start + ench_motorFH_slow_down_buffer) - RCount_FH)*1.4); // Faktor 1.4 für Höhere "Bremswirkung"
                            PLIB_OC_PulseWidth16BitSet(OC_ID_FH, PWM_MIN_FH);
                        } 
                        else {
                            PLIB_OC_PulseWidth16BitSet(OC_ID_FH, curr_pwm_FH); 
                        }
                    }
                    else {
                        LED = LED_OFF;
                       // RCount_FH = ench_motorFH_start; //test
                        ENABLE_MOTOR_FH = LOCKED;
                        PLIB_OC_PulseWidth16BitSet(OC_ID_FH, 0);
                        fh_stopCondition = true;
                        motorFH_upper_limit_reached = true;
                    }
                }
            }
        }    
    }  
    //MAXON
    if (motor_Maxon_initialized) 
    {
        // if PROX_SENSOR_MAXON active or count value smaller than limit -> go down
        if (RCount_Maxon < ench_motorMaxon_start) {
                ROTARY_DIRECTION_MOTOR_MAXON = DIRECTION_DOWN_MAXON;
                ENABLE_MOTOR_MAXON = !LOCKED;
                PLIB_OC_PulseWidth16BitSet(OC_ID_MAXON, PWM_MIN_MAXON); 
                maxon_stopCondition = true;
                maxon_upper_limit_reached = true;
        }
        // if PROX_SENSOR_MAXON not active and limit bigger than start_limit
        else {
            if (maxon_stopCondition) {
                ENABLE_MOTOR_MAXON = LOCKED;
                LED = LED_OFF;
                PLIB_OC_PulseWidth16BitSet(OC_ID_MAXON, 0); 
            }
            else {
                // downwards
                if (ROTARY_DIRECTION_MOTOR_MAXON == DIRECTION_DOWN_MAXON) {
                    // away from the "proximity sensor"
                    if (RCount_Maxon < ench_motorMaxon_end) {
                        LED = LED_ON;
                        ENABLE_MOTOR_MAXON = !LOCKED;
                        PLIB_OC_PulseWidth16BitSet(OC_ID_MAXON, curr_pwm_Maxon);
                        maxon_upper_limit_reached = false;
                    }
                    else {
                        LED = LED_OFF;
                        ENABLE_MOTOR_MAXON = LOCKED;
                        PLIB_OC_PulseWidth16BitSet(OC_ID_MAXON, 0);
                        maxon_stopCondition = true;
                        maxon_lower_limit_reached = true;
                    }
                }
                // upwards
                if (ROTARY_DIRECTION_MOTOR_MAXON == DIRECTION_UP_MAXON) {
                    //ROTARY_DIRECTION_MOTOR_MAXON = 0;  // to the "proximity sensor"               
                    if (RCount_Maxon > ench_motorMaxon_start) {
                        LED = LED_ON;
                        ENABLE_MOTOR_MAXON = !LOCKED;
                        PLIB_OC_PulseWidth16BitSet(OC_ID_MAXON, curr_pwm_Maxon); 
                        maxon_lower_limit_reached = false;
                    }
                    else {
                        LED = LED_OFF;
                        ENABLE_MOTOR_MAXON = LOCKED;
                        PLIB_OC_PulseWidth16BitSet(OC_ID_MAXON, 0);
                        maxon_stopCondition = true;
                        maxon_upper_limit_reached = true;
                    }
                }
            }
        }    
    }  
}

void Timer100msCallback (void) {
//    if (prostActivatedByCommand) {
//        angleFH = (int)((double)(RCount_FH-UPPER_LIMIT_FH)/(LOWER_LIMIT_FH-UPPER_LIMIT_FH)*100);
//        if (angleFH < 10) {
//            sprintf(appData.writeBuffer,"0%d",angleFH);
//        } else {
//            if (angleFH >= 100) {
//                angleFH = 99;
//            }
//            sprintf(appData.writeBuffer,"%d",angleFH);
//        }
//        angleMaxon = (int)((double)(RCount_Maxon-UPPER_LIMIT_MAXON)/(LOWER_LIMIT_MAXON-UPPER_LIMIT_MAXON)*100);
//        if (angleMaxon < 10) {
//            sprintf(appData.writeBuffer+2,"0%d",angleMaxon);
//        } else {
//            if (angleMaxon >= 100) {
//                angleMaxon = 99;
//            }
//            sprintf(appData.writeBuffer+2,"%d",angleMaxon);
//        }
//        if (*appData.writeBuffer != *appData.writeBufferTemp) {
//            *appData.writeBufferTemp = *appData.writeBuffer;
//            USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
//                    &appData.writeTransferHandle, appData.writeBuffer, 4,
//                    USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
//        }
//    }
}