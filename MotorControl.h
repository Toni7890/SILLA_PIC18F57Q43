/*
 * =============================================================================
 * MotorControl.h - CABECERA PARA EL CONTROL DE 2 MOTORES DC CON L298N
 * =============================================================================
 * Adaptado para PIC18F57Q43 y MCC-generado pins.h/ccp.h
 *
 * NOTA: Este driver asume que los pines de dirección y PWM están
 * configurados como salida en MCC y que los módulos CCP1 y CCP2 están
 * habilitados en modo PWM.
 *
 * CONEXIONES (SEGÚN TU PROYECTO):
 * Motor A (Izquierdo):
 * CCP1 (RC6) -> ENA (L298N)  - Velocidad Motor A
 * RD0 (GPIO) -> IN1 (L298N)  - Dirección Motor A
 * RD1 (GPIO) -> IN2 (L298N)  - Dirección Motor A
 *
 * Motor B (Derecho):
 * CCP2 (RC7) -> ENB (L298N)  - Velocidad Motor B
 * RD2 (GPIO) -> IN3 (L298N)  - Dirección Motor B
 * RD3 (GPIO) -> IN4 (L298N)  - Dirección Motor B
 *
 * =============================================================================
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "mcc_generated_files/system/pins.h" // Para IO_RDx_SetHigh/Low()
#include "mcc_generated_files/pwm/ccp1.h"    // Para CCP1_LoadDutyValue()
#include "mcc_generated_files/pwm/ccp2.h"    // Para CCP2_LoadDutyValue()
#include <xc.h>     // Para __delay_ms
#include <stdint.h> // Para uint8_t, uint16_t
#include <stdbool.h>

// =============================================================================
// DEFINICIONES DE PINES DE CONTROL (ADAPTADAS A TUS PINES DE main.c)
// =============================================================================
// Control Motor A (RD0 y RD1)
#define MOTOR_A_IN1_SetHigh() IO_RD0_SetHigh()
#define MOTOR_A_IN1_SetLow()  IO_RD0_SetLow()
#define MOTOR_A_IN2_SetHigh() IO_RD1_SetHigh()
#define MOTOR_A_IN2_SetLow()  IO_RD1_SetLow()

// Control Motor B (RD2 y RD3)
#define MOTOR_B_IN3_SetHigh() IO_RD2_SetHigh()
#define MOTOR_B_IN3_SetLow()  IO_RD2_SetLow()
#define MOTOR_B_IN4_SetHigh() IO_RD3_SetHigh()
#define MOTOR_B_IN4_SetLow()  IO_RD3_SetLow()

// =============================================================================
// DEFINICIONES DE ESTADOS Y VELOCIDADES
// =============================================================================
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_BRAKE
} motor_direction_t;

typedef enum {
    MOTOR_A = 0,    // Motor Izquierdo (CCP1)
    MOTOR_B = 1     // Motor Derecho (CCP2)
} motor_id_t;

// Configuraciones específicas para motores con reductor 180:1
#define MOTOR_MIN_SPEED_REDUCTION   15      // Mínimo 15% para vencer la reducción
#define MOTOR_MAX_PWM_VALUE         999UL   // Valor máximo de PWM para 100% (si 10-bit es 1023, pero el ejemplo usa 999)
// Valor del ciclo de trabajo para 4.5V desde 12V (37.5% de 999 = 374.625 ~ 375)
#define MOTOR_OPTIMAL_DUTY_VALUE    375
#define MOTOR_STARTUP_DUTY_VALUE    (uint16_t)((70UL * MOTOR_MAX_PWM_VALUE) / 100UL) // 70% de 999
#define MOTOR_RUNNING_DUTY_VALUE    MOTOR_OPTIMAL_DUTY_VALUE

#define STARTUP_DURATION_MS         300     // Duración del arranque rápido (300ms)
#define SYNC_DELAY_BETWEEN_MOTORS   5       // Delay mínimo entre motores (5ms)

// =============================================================================
// PROTOTIPOS DE FUNCIONES
// =============================================================================

// Funciones básicas
void Motors_Initialize(void);
void Motor_SetSpeed(motor_id_t motor_id, uint16_t pwm_duty_value); // Ahora acepta el valor directo del PWM
void Motor_SetDirection(motor_id_t motor_id, motor_direction_t direction);
void Motor_Control(motor_id_t motor_id, motor_direction_t direction, uint16_t pwm_duty_value);
void Motor_Stop(motor_id_t motor_id);
void Motor_Brake(motor_id_t motor_id);
void Motors_StopAll(void);

// Funciones de sincronización y arranque rápido
void Motors_SynchronizedStart(motor_direction_t dir_a, motor_direction_t dir_b, uint16_t final_pwm_duty);
void Motors_FastStart_ThenConstant(motor_direction_t dir_a, motor_direction_t dir_b);
void Motors_SynchronizedStop(void);
void Motors_SynchronizedDirection(motor_direction_t dir_a, motor_direction_t dir_b, uint16_t pwm_duty_value);

// Funciones de robot (control coordinado)
void Robot_MoveForward(uint16_t speed_pwm_duty);
void Robot_MoveBackward(uint16_t speed_pwm_duty);
void Robot_TurnLeft(uint16_t speed_pwm_duty);
void Robot_TurnRight(uint16_t speed_pwm_duty);
void Robot_RotateLeft(uint16_t speed_pwm_duty);
void Robot_RotateRight(uint16_t speed_pwm_duty);
void Robot_Stop(void);


#endif /* MOTOR_CONTROL_H */