/*
 * ===============================================================================
 * HC_SR04.C - IMPLEMENTACI�N DEL SENSOR ULTRAS�NICO HC-SR04
 * ===============================================================================
 * Versi�n: 1.1 - Adaptado para MCC, TMR0, y __delay_us/__delay_ms
 * ===============================================================================
 */

#include "HC_SR04.h"
// #include "DELAYS.h"  // Esta l�nea se elimina, usaremos __delay_ms/__delay_us de xc.h

// =============================================================================
// VARIABLES PRIVADAS EST�TICAS
// =============================================================================

// Configuraci�n actual del sensor (valores por defecto)
static hc_sr04_config_t sensor_config = {
    .detection_threshold_cm = HC_SR04_DEFAULT_THRESHOLD,
    .timeout_us             = HC_SR04_TIMEOUT_US,
    .stabilization_ms       = 10, // Unos 10ms de estabilizaci�n por defecto
    .auto_ranging           = false,
    .filter_enabled         = false
};

// Estado del sensor
static hc_sr04_status_t last_status = HC_SR04_OK;
static uint32_t         last_echo_time_us = 0; // Tiempo del pulso en microsegundos
static uint16_t         last_distance_cm = 0;
static bool             sensor_initialized = false;

// Buffer para filtrado de mediciones (si filter_enabled)
#define FILTER_BUFFER_SIZE 5
static uint16_t distance_buffer[FILTER_BUFFER_SIZE];
static uint8_t buffer_index = 0;
static bool buffer_full = false;

// =============================================================================
// FUNCIONES PRIVADAS (DECLARACIONES)
// =============================================================================
static void     HC_SR04_Add_To_Filter(uint16_t distance);
static uint16_t HC_SR04_Get_Filtered_Value(void);

// =============================================================================
// FUNCIONES DE BAJO NIVEL (ADAPTADAS PARA TMR0 Y __delay_us/__delay_ms)
// =============================================================================

/**
 * @brief Env�a un pulso de disparo al pin TRIG del HC-SR04.
 */
static void HC_SR04_Trigger_Pulse(void) {
    TRIG_PIN_SetLow();
    __delay_us(2);
    TRIG_PIN_SetHigh();
    __delay_us(10);
    TRIG_PIN_SetLow();
}

/**
 * @brief Mide la duraci�n del pulso ECHO en microsegundos usando TMR0.
 * Esta funci�n es bloqueante.
 * @return Duraci�n del pulso ECHO en microsegundos, o 0 si hay timeout.
 */
static uint32_t HC_SR04_Measure_Echo_Time(void) {
    uint16_t start_time, end_time;
    uint32_t pulse_duration_ticks;
    uint32_t timeout_counter_us = 0; // Cambiado a uint32_t para evitar overflow

    // Esperar a que el pin ECHO est� LOW (limpiar cualquier eco anterior)
    // Usamos el timeout para evitar bloqueo si el pin se queda HIGH indefinidamente
    while(ECHO_PIN_GetValue() == 1) {
        if (timeout_counter_us++ > (HC_SR04_TIMEOUT_US * 2UL)) return 0; // CR�TICO: Multiplicar por 2UL para forzar uint32_t
        __delay_us(1);
    }
    timeout_counter_us = 0; // Reiniciar contador para el siguiente bucle

    // Esperar a que el pin ECHO suba (inicio del pulso)
    while(ECHO_PIN_GetValue() == 0) {
        if (timeout_counter_us++ > HC_SR04_TIMEOUT_US) return 0; // Timeout
        __delay_us(1);
    }
    // TMR0_Start() se asume que se inicia una vez en main.c
    start_time = TMR0_CounterGet(); // �CORREGIDO! Usar TMR0_CounterGet()
    timeout_counter_us = 0; // Reiniciar contador

    // Esperar a que el pin ECHO baje (fin del pulso)
    while(ECHO_PIN_GetValue() == 1) {
        if (timeout_counter_us++ > HC_SR04_TIMEOUT_US) { // Timeout
            // TMR0_Stop() no es estrictamente necesario aqu� si TMR0 es libre
            return 0;
        }
        __delay_us(1);
    }
    // TMR0_Stop() no es estrictamente necesario aqu�
    end_time = TMR0_CounterGet(); // �CORREGIDO! Usar TMR0_CounterGet()

    // Calcular duraci�n del pulso y manejar desbordamiento de 16 bits
    if (end_time < start_time) { // Si el temporizador de 16 bits desbord�
        pulse_duration_ticks = (uint32_t)(0xFFFF - start_time) + end_time + 1; // +1 por el ajuste de desbordamiento
    } else {
        pulse_duration_ticks = end_time - start_time;
    }

    // Convertir ticks a microsegundos
    return (uint32_t)((float)pulse_duration_ticks * TMR0_TICKS_TO_US_FACTOR);
}


// =============================================================================
// FUNCIONES P�BLICAS
// =============================================================================

/**
 * @brief Inicializa el sensor HC-SR04 y sus pines.
 */
void HC_SR04_Init(void) {
    // Configurar el pin Trigger (RD4) como salida
    IO_RD4_SetDigitalOutput(); //
    TRIG_PIN_SetLow(); // Asegurarse de que el Trigger est� en LOW

    // Configurar el pin Echo (RA5) como entrada
    IO_RA5_SetDigitalInput(); //

    // TMR0_Initialize() y TMR0_Start() deben ser llamados en main.c una vez
    sensor_initialized = true;
}

/**
 * @brief Configura el sensor HC-SR04 con nuevos par�metros.
 */
void HC_SR04_SetConfig(hc_sr04_config_t config) {
    sensor_config = config;
}

/**
 * @brief Lee la distancia actual medida por el sensor HC-SR04 en cent�metros.
 * @return La distancia en cm. Retorna 0 si hay timeout o error.
 */
uint16_t HC_SR04_Read_Distance_CM(void) {
    if (!sensor_initialized) return 0; // Asegurarse de que el sensor est� inicializado

    // ***************************************************************
    // CORREGIDO: Usar bucle para retardo variable en lugar de __delay_ms(variable)
    for(uint16_t i = 0; i < sensor_config.stabilization_ms; i++) {
        __delay_ms(1);
    }
    // ***************************************************************

    HC_SR04_Trigger_Pulse(); // Env�a el pulso de disparo
    last_echo_time_us = HC_SR04_Measure_Echo_Time(); // Mide la duraci�n del eco

    if (last_echo_time_us == 0) {
        last_status = HC_SR04_TIMEOUT;
        last_distance_cm = 0;
        return 0;
    }

    // Calcular distancia en cm
    // Distancia (cm) = Duraci�n_us / (Velocidad del Sonido en us/cm de ida y vuelta)
    float distance_float = (float)last_echo_time_us / HC_SR04_SPEED_OF_SOUND_US_PER_CM_ROUNDTRIP;
    last_distance_cm = (uint16_t)distance_float;

    // Verificar si la distancia est� dentro del rango v�lido
    if (last_distance_cm < HC_SR04_MIN_DISTANCE_CM || last_distance_cm > HC_SR04_MAX_DISTANCE_CM) {
        last_status = HC_SR04_OUT_OF_RANGE;
        return 0; // Retorna 0 si est� fuera de rango o es inv�lido
    }

    last_status = HC_SR04_OK;

    // Si el filtrado est� habilitado, aplicar el filtro
    if (sensor_config.filter_enabled) {
        HC_SR04_Add_To_Filter(last_distance_cm);
        return HC_SR04_Get_Filtered_Value();
    } else {
        return last_distance_cm;
    }
}

/**
 * @brief Verifica si un objeto est� detectado dentro de un umbral espec�fico.
 * @param threshold_cm Umbral de distancia en cent�metros.
 * @return true si se detecta un objeto dentro del umbral, false en caso contrario.
 */
bool HC_SR04_Is_Object_Detected(uint16_t threshold_cm) {
    uint16_t distance = HC_SR04_Read_Distance_CM();
    return (distance > 0 && distance <= threshold_cm); // Distancia > 0 para descartar timeouts
}

/**
 * @brief Obtiene el �ltimo estado de la medici�n del sensor.
 * @return El �ltimo estado de la medici�n (HC_SR04_OK, HC_SR04_TIMEOUT, etc.).
 */
hc_sr04_status_t HC_SR04_Get_Last_Status(void) {
    return last_status;
}


// =============================================================================
// FUNCIONES PRIVADAS DE FILTRO
// =============================================================================

/**
 * @brief Agrega una medici�n al buffer de filtro
 */
static void HC_SR04_Add_To_Filter(uint16_t distance) {
    distance_buffer[buffer_index] = distance;
    buffer_index = (buffer_index + 1) % FILTER_BUFFER_SIZE;
    
    if(!buffer_full && buffer_index == 0) { // Si el buffer no estaba lleno y dimos la vuelta
        buffer_full = true;
    }
}

/**
 * @brief Obtiene el valor filtrado (promedio) del buffer
 */
static uint16_t HC_SR04_Get_Filtered_Value(void) {
    uint32_t sum = 0;
    uint8_t count = buffer_full ? FILTER_BUFFER_SIZE : buffer_index; // Contar solo las muestras v�lidas

    if (count == 0) return 0; // No hay datos para promediar

    for(uint8_t i = 0; i < count; i++) {
        sum += distance_buffer[i];
    }
    return (uint16_t)(sum / count);
}