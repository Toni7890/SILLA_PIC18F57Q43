/*
 * =============================================================================
 * MotorControl.c - IMPLEMENTACIÓN PARA EL CONTROL DE 2 MOTORES DC CON L298N
 * =============================================================================
 * Adaptado para PIC18F57Q43 y MCC-generado pins.h/ccp.h
 * =============================================================================
 */

#include "MotorControl.h"
#include "mcc_generated_files/system/clock.h" // Para _XTAL_FREQ si no está en xc.h

// *****************************************************************************
// PROTOTIPOS DE FUNCIONES PRIVADAS
// *****************************************************************************
// CORREGIDO: Declaración de la función ConvertPercentToPWM antes de su uso.
static uint16_t ConvertPercentToPWM(uint8_t percentage);


// Variables globales internas para el estado de los motores
volatile uint16_t current_motor_speeds_pwm[2] = {0, 0}; // Almacena el valor PWM actual
volatile motor_direction_t current_motor_directions[2] = {MOTOR_STOP, MOTOR_STOP};

// =============================================================================
// FUNCIONES BÁSICAS DE CONTROL DE MOTORES
// =============================================================================

/**
 * @brief Inicializa el control de ambos motores.
 * No configura TRIS/ANSEL directamente; asume MCC lo hace.
 */
void Motors_Initialize(void)
{
    // MCC Pin Manager ya configura los pines como Output/Digital
    // TRISAbits.TRISA0 = 0; // Se elimina, lo hace MCC
    // ANSELAbits.ANSELA0 = 0; // Se elimina, lo hace MCC

    // Estado inicial: ambos motores detenidos
    Motors_StopAll();
}

/**
 * @brief Establece la velocidad de un motor específico.
 * La velocidad se da como un valor PWM directo (0-MOTOR_MAX_PWM_VALUE).
 */
void Motor_SetSpeed(motor_id_t motor_id, uint16_t pwm_duty_value)
{
    if (pwm_duty_value > MOTOR_MAX_PWM_VALUE) pwm_duty_value = MOTOR_MAX_PWM_VALUE;

    // Para motores con reductor, asegurar una velocidad mínima si no es 0
    // Adaptado para usar valores PWM directamente
    if (pwm_duty_value > 0 && pwm_duty_value < ConvertPercentToPWM(MOTOR_MIN_SPEED_REDUCTION)) {
        pwm_duty_value = ConvertPercentToPWM(MOTOR_MIN_SPEED_REDUCTION);
    }

    if (motor_id == MOTOR_A) {
        CCP1_LoadDutyValue(pwm_duty_value); //
    } else if (motor_id == MOTOR_B) {
        CCP2_LoadDutyValue(pwm_duty_value); //
    }

    current_motor_speeds_pwm[motor_id] = pwm_duty_value;
}

/**
 * @brief Establece la dirección de un motor específico.
 */
void Motor_SetDirection(motor_id_t motor_id, motor_direction_t direction)
{
    if (motor_id == MOTOR_A) {
        switch (direction) {
            case MOTOR_FORWARD:
                MOTOR_A_IN1_SetHigh();
                MOTOR_A_IN2_SetLow();
                break;
            case MOTOR_BACKWARD:
                MOTOR_A_IN1_SetLow();
                MOTOR_A_IN2_SetHigh();
                break;
            case MOTOR_BRAKE: // Freno con cortocircuito
                MOTOR_A_IN1_SetHigh();
                MOTOR_A_IN2_SetHigh();
                break;
            case MOTOR_STOP: // Desconexión total (floating)
            default:
                MOTOR_A_IN1_SetLow();
                MOTOR_A_IN2_SetLow();
                break;
        }
    } else if (motor_id == MOTOR_B) {
        switch (direction) {
            case MOTOR_FORWARD:
                MOTOR_B_IN3_SetHigh();
                MOTOR_B_IN4_SetLow();
                break;
            case MOTOR_BACKWARD:
                MOTOR_B_IN3_SetLow();
                MOTOR_B_IN4_SetHigh();
                break;
            case MOTOR_BRAKE: // Freno con cortocircuito
                MOTOR_B_IN3_SetHigh();
                MOTOR_B_IN4_SetHigh();
                break;
            case MOTOR_STOP: // Desconexión total
            default:
                MOTOR_B_IN3_SetLow();
                MOTOR_B_IN4_SetLow();
                break;
        }
    }
    current_motor_directions[motor_id] = direction;
}

/**
 * @brief Control completo de un motor (dirección + velocidad).
 */
void Motor_Control(motor_id_t motor_id, motor_direction_t direction, uint16_t pwm_duty_value)
{
    // Si hay cambio de dirección y motor está en movimiento, parar primero
    if (current_motor_directions[motor_id] != direction && current_motor_speeds_pwm[motor_id] > 0) {
        Motor_Stop(motor_id);
        __delay_ms(200);  // Pausa más larga para reductores
    }

    Motor_SetDirection(motor_id, direction);

    if (direction == MOTOR_STOP || direction == MOTOR_BRAKE) {
        Motor_SetSpeed(motor_id, 0);
    } else {
        Motor_SetSpeed(motor_id, pwm_duty_value);
    }
}

/**
 * @brief Detiene un motor específico.
 */
void Motor_Stop(motor_id_t motor_id)
{
    Motor_SetSpeed(motor_id, 0);
    Motor_SetDirection(motor_id, MOTOR_STOP);
}

/**
 * @brief Frena un motor específico.
 */
void Motor_Brake(motor_id_t motor_id)
{
    Motor_SetDirection(motor_id, MOTOR_BRAKE);
    Motor_SetSpeed(motor_id, MOTOR_MAX_PWM_VALUE); // Aplicar 100% PWM para frenado dinámico
    __delay_ms(150);  // Tiempo de frenado
    Motor_Stop(motor_id);
}

/**
 * @brief Detiene ambos motores.
 */
void Motors_StopAll(void)
{
    Motor_Stop(MOTOR_A);
    Motor_Stop(MOTOR_B);
}

// =============================================================================
// FUNCIONES DE SINCRONIZACIÓN Y ARRANQUE RÁPIDO
// =============================================================================

/**
 * @brief Arranque rápido sincronizado: alta velocidad inicial + velocidad constante.
 * @param dir_a Dirección motor A
 * @param dir_b Dirección motor B
 */
void Motors_FastStart_ThenConstant(motor_direction_t dir_a, motor_direction_t dir_b)
{
    // FASE 1: Configurar direcciones SIMULTÁNEAMENTE
    Motor_SetDirection(MOTOR_A, dir_a);
    Motor_SetDirection(MOTOR_B, dir_b);
    __delay_ms(SYNC_DELAY_BETWEEN_MOTORS); // 5ms para estabilizar

    // FASE 2: Arranque rápido SIMULTÁNEO para vencer inercia
    Motor_SetSpeed(MOTOR_A, MOTOR_STARTUP_DUTY_VALUE); // 70% inicial
    Motor_SetSpeed(MOTOR_B, MOTOR_STARTUP_DUTY_VALUE); // 70% inicial

    // FASE 3: Mantener velocidad alta por período corto
    __delay_ms(STARTUP_DURATION_MS); // 300ms de arranque rápido

    // FASE 4: Reducir a velocidad constante SIMULTÁNEAMENTE
    Motor_SetSpeed(MOTOR_A, MOTOR_RUNNING_DUTY_VALUE); // Valor PWM constante
    Motor_SetSpeed(MOTOR_B, MOTOR_RUNNING_DUTY_VALUE); // Valor PWM constante
}

/**
 * @brief Arranque sincronizado con velocidad personalizable.
 * @param dir_a Dirección motor A
 * @param dir_b Dirección motor B
 * @param final_pwm_duty Valor PWM final deseado.
 */
void Motors_SynchronizedStart(motor_direction_t dir_a, motor_direction_t dir_b, uint16_t final_pwm_duty)
{
    // Configurar direcciones simultáneamente
    Motor_SetDirection(MOTOR_A, dir_a);
    Motor_SetDirection(MOTOR_B, dir_b);
    __delay_ms(SYNC_DELAY_BETWEEN_MOTORS);

    // Arranque rápido para vencer inercia
    Motor_SetSpeed(MOTOR_A, MOTOR_STARTUP_DUTY_VALUE);
    Motor_SetSpeed(MOTOR_B, MOTOR_STARTUP_DUTY_VALUE);
    __delay_ms(STARTUP_DURATION_MS);

    // Transición suave a velocidad final
    uint16_t current_pwm = MOTOR_STARTUP_DUTY_VALUE;
    while (current_pwm != final_pwm_duty) {
        if (current_pwm > final_pwm_duty) {
            current_pwm -= (current_pwm - final_pwm_duty < 5) ? 1 : (current_pwm / 20); // Ajuste más dinámico
            if (current_pwm < final_pwm_duty) current_pwm = final_pwm_duty;
        } else {
            current_pwm += (final_pwm_duty - current_pwm < 5) ? 1 : (current_pwm / 20); // Ajuste más dinámico
            if (current_pwm > final_pwm_duty) current_pwm = final_pwm_duty;
        }

        Motor_SetSpeed(MOTOR_A, current_pwm);
        Motor_SetSpeed(MOTOR_B, current_pwm);
        __delay_ms(20); // 20ms entre pasos para transición suave
    }
}

/**
 * @brief Parada sincronizada de ambos motores.
 */
void Motors_SynchronizedStop(void)
{
    // Parada simultánea para evitar desfase
    Motor_SetSpeed(MOTOR_A, 0);
    Motor_SetSpeed(MOTOR_B, 0);

    // Esperar un ciclo para sincronizar
    __delay_ms(SYNC_DELAY_BETWEEN_MOTORS);

    // Configurar direcciones en STOP
    Motor_SetDirection(MOTOR_A, MOTOR_STOP);
    Motor_SetDirection(MOTOR_B, MOTOR_STOP);
}

/**
 * @brief Control direccional sincronizado.
 * @param dir_a Dirección motor A
 * @param dir_b Dirección motor B
 * @param pwm_duty_value Valor PWM para ambos motores.
 */
void Motors_SynchronizedDirection(motor_direction_t dir_a, motor_direction_t dir_b, uint16_t pwm_duty_value)
{
    // Si hay cambio de dirección, parar ambos primero
    bool direction_change = (current_motor_directions[MOTOR_A] != dir_a && current_motor_speeds_pwm[MOTOR_A] > 0) ||
                            (current_motor_directions[MOTOR_B] != dir_b && current_motor_speeds_pwm[MOTOR_B] > 0);

    if (direction_change) {
        Motors_SynchronizedStop();
        __delay_ms(100); // Pausa de seguridad
    }

    // Aplicar nuevas direcciones y velocidad
    Motors_SynchronizedStart(dir_a, dir_b, pwm_duty_value);
}

// =============================================================================
// FUNCIONES DE ROBOT (CONTROL COORDINADO)
// =============================================================================

void Robot_MoveForward(uint16_t speed_pwm_duty)
{
    Motors_SynchronizedDirection(MOTOR_FORWARD, MOTOR_FORWARD, speed_pwm_duty);
}

void Robot_MoveBackward(uint16_t speed_pwm_duty)
{
    Motors_SynchronizedDirection(MOTOR_BACKWARD, MOTOR_BACKWARD, speed_pwm_duty);
}

void Robot_TurnLeft(uint16_t speed_pwm_duty)
{
    // Motor izquierdo más lento para giro
    // Opcional: un motor adelante y otro atrás para rotación sobre el eje.
    // Usaremos rotación sobre el eje para giros más bruscos.
    Motors_SynchronizedDirection(MOTOR_BACKWARD, MOTOR_FORWARD, speed_pwm_duty);
}

void Robot_TurnRight(uint16_t speed_pwm_duty)
{
    // Motor derecho más lento para giro
    // Usaremos rotación sobre el eje para giros más bruscos.
    Motors_SynchronizedDirection(MOTOR_FORWARD, MOTOR_BACKWARD, speed_pwm_duty);
}

void Robot_RotateLeft(uint16_t speed_pwm_duty)
{
    // Gira sobre su propio eje a la izquierda (motores opuestos)
    Motors_SynchronizedDirection(MOTOR_BACKWARD, MOTOR_FORWARD, speed_pwm_duty);
}

void Robot_RotateRight(uint16_t speed_pwm_duty)
{
    // Gira sobre su propio eje a la derecha (motores opuestos)
    Motors_SynchronizedDirection(MOTOR_FORWARD, MOTOR_BACKWARD, speed_pwm_duty);
}

void Robot_Stop(void)
{
    Motors_SynchronizedStop();
}

/**
 * @brief Convierte porcentaje a valor PWM (0-1023 para 10-bit).
 * Utiliza 1023 como valor máximo para 100% para aprovechar la resolución completa.
 */
static uint16_t ConvertPercentToPWM(uint8_t percentage)
{
    if (percentage > 100) percentage = 100;
    return (uint16_t)((percentage * 1023UL) / 100UL); // Usa 1023UL para máxima resolución de 10-bit PWM
}