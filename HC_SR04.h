/*
 * ===============================================================================
 * HC_SR04.H - LIBRER�A PARA SENSOR ULTRAS�NICO HC-SR04
 * ===============================================================================
 * Autor: Librer�a para PIC18F57Q43
 * Versi�n: 1.1 - Adaptado para MCC y configuraci�n de proyecto
 * Fecha: 2025
 *
 * DESCRIPCI�N:
 * Librer�a completa para manejo del sensor ultras�nico HC-SR04
 * Incluye medici�n de distancia y detecci�n de objetos con umbral configurable
 *
 * CONEXIONES HARDWARE (ADAPTADAS A TU PROYECTO):
 * TRIG: RD4 (Salida - Trigger del sensor)
 * ECHO: RA5 (Entrada - Echo del sensor, conectada a INT0)
 * VCC: 5V
 * GND: GND
 *
 * ESPECIFICACIONES (del sensor):
 * - Rango de medici�n: 2cm - 400cm
 * - Precisi�n: �3mm
 * - �ngulo de detecci�n: 15�
 * - Frecuencia ultras�nica: 40kHz
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

// Definir la frecuencia del oscilador si no est� definida (MCC ya lo hace en clock.h)
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 64000000UL // Usar la frecuencia de tu sistema
#endif

// =============================================================================
// CONFIGURACI�N DE HARDWARE - MODIFICADO PARA TU PROYECTO
// =============================================================================
// Macros de pin adaptadas a tus IO_RD4 y IO_RA5 de pins.h
#define TRIG_PIN_SetHigh()          IO_RD4_SetHigh()
#define TRIG_PIN_SetLow()           IO_RD4_SetLow()
#define ECHO_PIN_GetValue()         IO_RA5_GetValue() // RA5 es el pin de Echo

// =============================================================================
// CONFIGURACI�N DEL SENSOR Y UMBRALES
// =============================================================================
#define HC_SR04_TIMEOUT_US          25000   // Tiempo m�ximo de espera para el eco (25ms para 400cm)
#define HC_SR04_MIN_DISTANCE_CM     2       // Distancia m�nima detectable
#define HC_SR04_MAX_DISTANCE_CM     400     // Distancia m�xima detectable
#define HC_SR04_DEFAULT_THRESHOLD   15      // Umbral por defecto para detecci�n de objeto (cm)

// Coeficientes para c�lculo de distancia (0.0343 us/cm es la velocidad del sonido a 20C)
// Pulso de ida y vuelta, por lo que es 58.2 us/cm (o 29.1 us/cm para solo ida)
#define HC_SR04_SPEED_OF_SOUND_US_PER_CM_ROUNDTRIP  58.0f // (aprox. 58.0 us/cm para 20C)

// Factor de conversi�n de ticks a microsegundos (basado en TMR0: 1:8 prescaler de 16MHz Fosc/4)
// TMR0 tick = 1 / (16MHz / 8) = 1 / 2MHz = 0.5 us/tick
#define TMR0_TICKS_TO_US_FACTOR     0.5f

// =============================================================================
// TIPOS DE DATOS Y ESTRUCTURAS
// =============================================================================
typedef struct {
    uint16_t    detection_threshold_cm; // Umbral de detecci�n en cm
    uint32_t    timeout_us;             // Timeout para la medici�n en us
    uint16_t    stabilization_ms;       // Tiempo de estabilizaci�n entre mediciones en ms
    bool        auto_ranging;           // Habilitar rango autom�tico
    bool        filter_enabled;         // Habilitar filtro de promedio m�vil
} hc_sr04_config_t;

typedef enum {
    HC_SR04_OK,             // Medici�n exitosa
    HC_SR04_TIMEOUT,        // No se detect� eco
    HC_SR04_OUT_OF_RANGE,   // Distancia fuera del rango del sensor (ej. < 2cm o > 400cm)
    HC_SR04_ERROR           // Error general
} hc_sr04_status_t;

// =============================================================================
// FUNCIONES P�BLICAS
// =============================================================================

/**
 * @brief Inicializa el sensor HC-SR04 y sus pines.
 */
void HC_SR04_Init(void);

/**
 * @brief Configura el sensor HC-SR04 con nuevos par�metros.
 */
void HC_SR04_SetConfig(hc_sr04_config_t config);

/**
 * @brief Lee la distancia actual medida por el sensor HC-SR04 en cent�metros.
 * @return La distancia en cm. Retorna 0 si hay timeout o error.
 */
uint16_t HC_SR04_Read_Distance_CM(void);

/**
 * @brief Verifica si un objeto est� detectado dentro de un umbral espec�fico.
 * @param threshold_cm Umbral de distancia en cent�metros.
 * @return true si se detecta un objeto dentro del umbral, false en caso contrario.
 */
bool HC_SR04_Is_Object_Detected(uint16_t threshold_cm);

/**
 * @brief Obtiene el �ltimo estado de la medici�n del sensor.
 * @return El �ltimo estado de la medici�n (HC_SR04_OK, HC_SR04_TIMEOUT, etc.).
 */
hc_sr04_status_t HC_SR04_Get_Last_Status(void);

// =============================================================================
// MACROS �TILES PARA EL USUARIO
// =============================================================================

// Macros para detecci�n r�pida
#define OBJECT_DETECTED_DEFAULT()   HC_SR04_Is_Object_Detected(HC_SR04_DEFAULT_THRESHOLD)
#define OBJECT_CLOSE()              HC_SR04_Is_Object_Detected(10)
#define OBJECT_VERY_CLOSE()         HC_SR04_Is_Object_Detected(5)
#define GET_DISTANCE_CM()           HC_SR04_Read_Distance_CM()

#endif /* HC_SR04_H */