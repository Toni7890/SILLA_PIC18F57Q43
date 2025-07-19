/*
 * ===============================================================================
 * HC_SR04.H - LIBRERÍA PARA SENSOR ULTRASÓNICO HC-SR04
 * ===============================================================================
 * Autor: Librería para PIC18F57Q43
 * Versión: 1.1 - Adaptado para MCC y configuración de proyecto
 * Fecha: 2025
 *
 * DESCRIPCIÓN:
 * Librería completa para manejo del sensor ultrasónico HC-SR04
 * Incluye medición de distancia y detección de objetos con umbral configurable
 *
 * CONEXIONES HARDWARE (ADAPTADAS A TU PROYECTO):
 * TRIG: RD4 (Salida - Trigger del sensor)
 * ECHO: RA5 (Entrada - Echo del sensor, conectada a INT0)
 * VCC: 5V
 * GND: GND
 *
 * ESPECIFICACIONES (del sensor):
 * - Rango de medición: 2cm - 400cm
 * - Precisión: ±3mm
 * - Ángulo de detección: 15°
 * - Frecuencia ultrasónica: 40kHz
 * ===============================================================================
 */

#ifndef HC_SR04_H
#define HC_SR04_H

// =============================================================================
// INCLUDES NECESARIOS
// =============================================================================
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "mcc_generated_files/system/pins.h"   // Para las macros de pines (IO_RD4_SetHigh, IO_RA5_GetValue)
#include "mcc_generated_files/system/clock.h"  // Para _XTAL_FREQ, necesario para __delay_us/__delay_ms
#include "mcc_generated_files/timer/tmr0.h"    // Para TMR0_ReadTimer, etc.

// Definir la frecuencia del oscilador si no está definida (MCC ya lo hace en clock.h)
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 64000000UL // Usar la frecuencia de tu sistema
#endif

// =============================================================================
// CONFIGURACIÓN DE HARDWARE - MODIFICADO PARA TU PROYECTO
// =============================================================================
// Macros de pin adaptadas a tus IO_RD4 y IO_RA5 de pins.h
#define TRIG_PIN_SetHigh()          IO_RD4_SetHigh()
#define TRIG_PIN_SetLow()           IO_RD4_SetLow()
#define ECHO_PIN_GetValue()         IO_RA5_GetValue() // RA5 es el pin de Echo

// =============================================================================
// CONFIGURACIÓN DEL SENSOR Y UMBRALES
// =============================================================================
#define HC_SR04_TIMEOUT_US          25000   // Tiempo máximo de espera para el eco (25ms para 400cm)
#define HC_SR04_MIN_DISTANCE_CM     2       // Distancia mínima detectable
#define HC_SR04_MAX_DISTANCE_CM     400     // Distancia máxima detectable
#define HC_SR04_DEFAULT_THRESHOLD   15      // Umbral por defecto para detección de objeto (cm)

// Coeficientes para cálculo de distancia (0.0343 us/cm es la velocidad del sonido a 20C)
// Pulso de ida y vuelta, por lo que es 58.2 us/cm (o 29.1 us/cm para solo ida)
#define HC_SR04_SPEED_OF_SOUND_US_PER_CM_ROUNDTRIP  58.0f // (aprox. 58.0 us/cm para 20C)

// Factor de conversión de ticks a microsegundos (basado en TMR0: 1:8 prescaler de 16MHz Fosc/4)
// TMR0 tick = 1 / (16MHz / 8) = 1 / 2MHz = 0.5 us/tick
#define TMR0_TICKS_TO_US_FACTOR     0.5f

// =============================================================================
// TIPOS DE DATOS Y ESTRUCTURAS
// =============================================================================
typedef struct {
    uint16_t    detection_threshold_cm; // Umbral de detección en cm
    uint32_t    timeout_us;             // Timeout para la medición en us
    uint16_t    stabilization_ms;       // Tiempo de estabilización entre mediciones en ms
    bool        auto_ranging;           // Habilitar rango automático
    bool        filter_enabled;         // Habilitar filtro de promedio móvil
} hc_sr04_config_t;

typedef enum {
    HC_SR04_OK,             // Medición exitosa
    HC_SR04_TIMEOUT,        // No se detectó eco
    HC_SR04_OUT_OF_RANGE,   // Distancia fuera del rango del sensor (ej. < 2cm o > 400cm)
    HC_SR04_ERROR           // Error general
} hc_sr04_status_t;

// =============================================================================
// FUNCIONES PÚBLICAS
// =============================================================================

/**
 * @brief Inicializa el sensor HC-SR04 y sus pines.
 */
void HC_SR04_Init(void);

/**
 * @brief Configura el sensor HC-SR04 con nuevos parámetros.
 */
void HC_SR04_SetConfig(hc_sr04_config_t config);

/**
 * @brief Lee la distancia actual medida por el sensor HC-SR04 en centímetros.
 * @return La distancia en cm. Retorna 0 si hay timeout o error.
 */
uint16_t HC_SR04_Read_Distance_CM(void);

/**
 * @brief Verifica si un objeto está detectado dentro de un umbral específico.
 * @param threshold_cm Umbral de distancia en centímetros.
 * @return true si se detecta un objeto dentro del umbral, false en caso contrario.
 */
bool HC_SR04_Is_Object_Detected(uint16_t threshold_cm);

/**
 * @brief Obtiene el último estado de la medición del sensor.
 * @return El último estado de la medición (HC_SR04_OK, HC_SR04_TIMEOUT, etc.).
 */
hc_sr04_status_t HC_SR04_Get_Last_Status(void);

// =============================================================================
// MACROS ÚTILES PARA EL USUARIO
// =============================================================================

// Macros para detección rápida
#define OBJECT_DETECTED_DEFAULT()   HC_SR04_Is_Object_Detected(HC_SR04_DEFAULT_THRESHOLD)
#define OBJECT_CLOSE()              HC_SR04_Is_Object_Detected(10)
#define OBJECT_VERY_CLOSE()         HC_SR04_Is_Object_Detected(5)
#define GET_DISTANCE_CM()           HC_SR04_Read_Distance_CM()

#endif /* HC_SR04_H */