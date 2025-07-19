/*
 * MAIN Generated Driver File
 *
 * @file main.c
 *
 * (Se mantienen los encabezados y licencia)
*/

#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/uart/uart1.h"        // Para funciones UART (HC-05)
#include "mcc_generated_files/pwm/ccp1.h"          // Para CCP1_LoadDutyValue() (Motor 1)
#include "mcc_generated_files/pwm/ccp2.h"          // Para CCP2_LoadDutyValue() (Motor 2)
#include "mcc_generated_files/system/pins.h"       // Para funciones GPIO como IO_RD0_SetHigh()
#include "mcc_generated_files/system/interrupt.h"  // Para INTERRUPT_GlobalInterruptHighEnable()
#include "mcc_generated_files/i2c_host/i2c1.h"     // Para el módulo I2C del PIC (base para LCD y MPU6050)
#include "I2C_LCD.h"                               // Driver para tu LCD I2C
#include "I2C_MPU6050.h"                           // Driver para tu MPU6050
#include "HC_SR04.h"                               // Driver para tu HC-SR04
#include "mcc_generated_files/timer/tmr0.h"        // Para TMR0, usado con HC-SR04
#include "MotorControl.h"                          // ¡Nuevo! Driver para control de motores
#include <string.h>                                // Necesario para strlen().
#include <xc.h>                                    // Para __delay_ms.
#include <stdio.h>                                 // Para sprintf().
#include <math.h>                                  // Para funciones matemáticas como atan2f.
#include <stdlib.h>                                // Para rand() y srand().


// ***************************************************************
// Variables globales para la comunicación UART
// ***************************************************************
volatile uint8_t receivedByte;
volatile bool newDataReceived = false;

// ***************************************************************
// Variables globales para MPU6050
// ***************************************************************
MPU6050_Data_t mpu_data; // Estructura para almacenar los datos del MPU6050

// ***************************************************************
// Información Fija del Paciente para LCD
// ***************************************************************
const char *fixedPatientName = "PACIENTE: John Doe";
const char *fixedDisease     = "ENFERMEDAD: Paraplejia";
const char *fixedPhone       = "TELEFONO: 987-654-321";

// ***************************************************************
// Variables y Definiciones para HC-SR04
// ***************************************************************
#define THRESHOLD_DISTANCE_CM 10.0f // Distancia de umbral para detener motores

volatile float distance_cm; // Distancia calculada en cm

// Bandera para indicar si la silla de ruedas está en modo de retroceso
bool is_reversing_flag = false;


// ***************************************************************
// Callback de recepción UART1
// ***************************************************************
void UART1_RxDataHandler(void) {
    if (UART1_IsRxReady()) { //
        receivedByte = UART1_Read(); //
        newDataReceived = true;
    }
}

// ***************************************************************
// Función auxiliar para enviar una cadena de texto por UART1
// ***************************************************************
void UART1_sendString(const char *str) {
    while (*str != '\0') {
        while (!UART1_IsTxReady()) { //
            // Esperar
        }
        UART1_Write(*str); //
        str++;
    }
}


/*
    Main application
*/

int main(void)
{
    SYSTEM_Initialize();

    INTERRUPT_GlobalInterruptHighEnable(); //
    UART1_RxCompleteCallbackRegister(UART1_RxDataHandler); //

    // Inicialización del driver de motores
    Motors_Initialize(); // Nueva función de inicialización de MotorControl.c
    
    // Establecer velocidad de los motores a 0% al inicio
    // Estos ya no son necesarios aquí si Motors_Initialize llama a Motors_StopAll()
    // CCP1_LoadDutyValue(0); 
    // CCP2_LoadDutyValue(0); 

    // Inicialización de la LCD I2C
    __delay_ms(100);
    i2c_lcd_init();     //
    // ***************************************************************
    // LÓGICA DE VISUALIZACIÓN FIJA DE LA LCD I2C AL INICIO
    // ***************************************************************
    i2c_lcd_clear();
    i2c_lcd_goto(0,0);
    i2c_lcd_puts(fixedPatientName); // Muestra nombre del paciente en línea 1
    i2c_lcd_goto(0,1);
    i2c_lcd_puts(fixedDisease);     // Muestra enfermedad en línea 2
    // ***************************************************************


    // Inicialización y calibración del MPU6050
    __delay_ms(200);
    if (MPU6050_Init()) { //
        UART1_sendString("MPU6050: Inicializado correctamente.\r\n");
        MPU6050_Calibrate(); //
        UART1_sendString("MPU6050: Calibracion finalizada.\r\n");
        __delay_ms(1000);
    } else {
        UART1_sendString("MPU6050: ERROR al inicializar.\r\n");
        while(1); // Si el MPU6050 no inicializa, detiene el programa
    }
    // ***************************************************************

    // Inicializar HC-SR04
    HC_SR04_Init(); // Inicializa los pines del HC-SR04

    // Inicializar TMR0 (Necesario para el driver HC-SR04)
    TMR0_Initialize(); //
    TMR0_Start();      // Inicia el Timer0 para que cuente


    // Envío de "Hola Mundo" por UART (para depuración serial)
    __delay_ms(100);
    UART1_sendString("Hola Mundo desde PIC18F57Q43!\r\n"); //
    for(uint16_t i = 0; i < 50; i++) {
        __delay_ms(100);
    }

    while(1) // Bucle principal infinito de la aplicación
    {
        // ***************************************************************
        // Lógica para recibir y procesar comandos del celular via HC-05 (UART)
        // ***************************************************************
        if (newDataReceived) {
            newDataReceived = false;

            switch (receivedByte) {
                case 'F':
                    Robot_MoveForward(MOTOR_OPTIMAL_DUTY_VALUE); //
                    UART1_sendString("Cmd UART: Adelante\r\n");
                    is_reversing_flag = false;
                    break;
                case 'B':
                    Robot_MoveBackward(MOTOR_OPTIMAL_DUTY_VALUE); //
                    UART1_sendString("Cmd UART: Atras\r\n");
                    is_reversing_flag = true;
                    break;
                case 'S':
                    Robot_Stop(); //
                    UART1_sendString("Cmd UART: Detener\r\n");
                    is_reversing_flag = false;
                    break;
                case 'L': // Giro a la izquierda
                    Robot_TurnLeft(MOTOR_OPTIMAL_DUTY_VALUE); //
                    UART1_sendString("Cmd UART: Izquierda\r\n");
                    is_reversing_flag = false;
                    break;
                case 'R': // Giro a la derecha
                    Robot_TurnRight(MOTOR_OPTIMAL_DUTY_VALUE); //
                    UART1_sendString("Cmd UART: Derecha\r\n");
                    is_reversing_flag = false;
                    break;
                case 'P': // Muestra info de paciente (si se presiona P)
                    i2c_lcd_clear();
                    i2c_lcd_goto(0,0); i2c_lcd_puts(fixedPatientName);
                    i2c_lcd_goto(0,1); i2c_lcd_puts(fixedDisease);
                    __delay_ms(3000);
                    i2c_lcd_clear();
                    i2c_lcd_goto(0,0); i2c_lcd_puts(fixedPatientName);
                    i2c_lcd_goto(0,1); i2c_lcd_puts(fixedDisease);
                    break;
                default:
                    UART1_sendString("Cmd UART: Desconocido ("); UART1_Write(receivedByte); UART1_sendString(")\r\n");
                    is_reversing_flag = false;
                    break;
            }
        }

        // ***************************************************************
        // Lógica del MPU6050 (Lectura y Control por Ejes/Movimiento)
        // Solo controla los motores si no hay un comando UART reciente.
        // ***************************************************************
        static uint16_t mpu_update_counter = 0;
        if(mpu_update_counter >= 100) {
            mpu_update_counter = 0;

            MPU6050_ReadSensorData(&mpu_data); //
            MPU6050_Movement_t current_movement = MPU6050_GetMovement(&mpu_data); //
            
            // Actualizar is_reversing_flag basado en el MPU6050 si no hay comando UART
            if (!newDataReceived) {
                if (current_movement == MOVEMENT_BACKWARD) {
                    is_reversing_flag = true;
                } else if (current_movement == MOVEMENT_NONE ||
                           current_movement == MOVEMENT_FORWARD ||
                           current_movement == MOVEMENT_LEFT ||
                           current_movement == MOVEMENT_RIGHT ||
                           current_movement == MOVEMENT_SHAKE ||
                           current_movement == MOVEMENT_TAP) {
                    is_reversing_flag = false;
                }
            }

            // Control de motores basado en el MPU6050 (solo si no hay un comando UART este ciclo)
            if(!newDataReceived) {
                switch (current_movement) {
                    case MOVEMENT_FORWARD:
                        Robot_MoveForward(MOTOR_OPTIMAL_DUTY_VALUE); //
                        UART1_sendString("MPU: Adelante\r\n");
                        break;
                    case MOVEMENT_BACKWARD:
                        Robot_MoveBackward(MOTOR_OPTIMAL_DUTY_VALUE); //
                        UART1_sendString("MPU: Atras\r\n");
                        break;
                    case MOVEMENT_LEFT:
                        Robot_TurnLeft(MOTOR_OPTIMAL_DUTY_VALUE); //
                        UART1_sendString("MPU: Izquierda\r\n");
                        break;
                    case MOVEMENT_RIGHT:
                        Robot_TurnRight(MOTOR_OPTIMAL_DUTY_VALUE); //
                        UART1_sendString("MPU: Derecha\r\n");
                        break;
                    case MOVEMENT_SHAKE:
                        Robot_Stop(); //
                        UART1_sendString("MPU: SHAKE - STOP\r\n");
                        break;
                    case MOVEMENT_TAP:
                        UART1_sendString("MPU: TAP detectado\r\n");
                        break;
                    default: // MOVEMENT_NONE
                        Robot_Stop(); //
                        UART1_sendString("MPU: Sin Movimiento\r\n");
                        break;
                }
            }
        }
        mpu_update_counter++;


        // ***************************************************************
        // Lógica del Sensor HC-SR04 (Detección de Obstáculos al Retroceder)
        // Se activa solo si is_reversing_flag es true.
        // ***************************************************************
        static uint16_t hcsr04_trigger_counter = 0;
        if (is_reversing_flag) {
            if (hcsr04_trigger_counter >= 50) {
                hcsr04_trigger_counter = 0;
                
                uint16_t current_distance = HC_SR04_Read_Distance_CM(); //

                if (HC_SR04_Get_Last_Status() == HC_SR04_OK) { //
                    distance_cm = (float)current_distance;

                    if (distance_cm > 0 && distance_cm <= THRESHOLD_DISTANCE_CM) {
                        Robot_Stop(); // Detener motores
                        is_reversing_flag = false;

                        char alert_str[64];
                        sprintf(alert_str, "ALERTA! Obstaculo a %.1fcm. MOTORES DETENIDOS.\r\n", (double)distance_cm);
                        UART1_sendString(alert_str);
                        
                        i2c_lcd_clear();
                        i2c_lcd_goto(0,0); i2c_lcd_puts("!! ALERTA !!   ");
                        sprintf(alert_str, "Obj:%.1fcm-DETENIDO",(double)distance_cm);
                        i2c_lcd_goto(0,1); i2c_lcd_puts(alert_str);
                        __delay_ms(2000);
                        
                        i2c_lcd_clear();
                        i2c_lcd_goto(0,0); i2c_lcd_puts(fixedPatientName);
                        i2c_lcd_goto(0,1); i2c_lcd_puts(fixedDisease);
                    }
                }
            }
            hcsr04_trigger_counter++;
        } else {
            hcsr04_trigger_counter = 0;
        }
        // ***************************************************************
    }
}