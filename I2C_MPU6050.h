/*******************************************************************************
 * I2C_MPU6050.h - Librería MPU6050 para Detección de Movimientos Direccionales
 *
 * PROYECTO: Sistema de Control con Movimientos para Personas con Discapacidad Motora
 * MICROCONTROLADOR: PIC18F57Q43
 * COMUNICACIÓN: I2C Host de MCC (RB1=SCL, RB2=SDA)
 *
 * FUNCIONALIDAD:
 * - Detección de movimientos direccionales: ADELANTE, ATRÁS, IZQUIERDA, DERECHA
 * - Fusión de datos del acelerómetro y giroscopio
 * - Filtrado digital para reducir ruido
 * - Calibración automática
 * - Detección de gestos específicos para control adaptativo
 *
 * AUTOR: Ingeniero Embebido
 * FECHA: 2025
 *******************************************************************************/

#ifndef I2C_MPU6050_H
#define I2C_MPU6050_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
// #include "config.h" // Si tienes un archivo de configuración propio, asegúrate de que exista y lo que define sea compatible
// ***************************************************************
// Cambiado: Incluir el driver I2C de MCC en lugar del Software_I2C
#include "mcc_generated_files/i2c_host/i2c1.h"
// ***************************************************************

/*******************************************************************************
 * DEFINICIONES Y CONSTANTES
 *******************************************************************************/

// Dirección I2C del MPU6050
#define MPU6050_I2C_ADDR            0x68    // Dirección por defecto (AD0 a GND)

// Registros principales del MPU6050
#define MPU6050_WHO_AM_I            0x75    // Identificador del dispositivo
#define MPU6050_PWR_MGMT_1          0x6B    // Administración de energía
#define MPU6050_SMPRT_DIV           0x19    // Divisor de la frecuencia de muestreo
#define MPU6050_CONFIG              0x1A    // Configuración
#define MPU6050_GYRO_CONFIG         0x1B    // Configuración del giroscopio
#define MPU6050_ACCEL_CONFIG        0x1C    // Configuración del acelerómetro

// Registros de datos del sensor
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_TEMP_OUT_L          0x42
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48

// Escalas del sensor (para Full Scale Range: FSR)
#define ACCEL_RANGE_2G              0x00    // +/- 2g
#define ACCEL_RANGE_4G              0x08    // +/- 4g
#define ACCEL_RANGE_8G              0x10    // +/- 8g
#define ACCEL_RANGE_16G             0x18    // +/- 16g

#define GYRO_RANGE_250_DPS          0x00    // +/- 250 grados/seg
#define GYRO_RANGE_500_DPS          0x08    // +/- 500 grados/seg
#define GYRO_RANGE_1000_DPS         0x10    // +/- 1000 grados/seg
#define GYRO_RANGE_2000_DPS         0x18    // +/- 2000 grados/seg

// Conversión de valores raw a Gs o DPS (depende del rango seleccionado)
// Para 2g: 16384.0f LSB/g
// Para 250 dps: 131.0f LSB/(dps)

// Umbrales de movimiento y calibración (ajustables)
#define MPU6050_ACCEL_THRESHOLD_G   0.2f    // Umbral de aceleración en Gs para detección de movimiento
#define MPU6050_GYRO_THRESHOLD_DPS  10.0f   // Umbral de giroscopio en DPS para detección de giro

#define MPU6050_CALIBRATION_SAMPLES 100     // Número de muestras para calibración

// Definiciones para detección de gestos avanzados
#define MPU6050_TILT_THRESHOLD_DEG  15.0f   // Umbral de inclinación en grados
#define MPU6050_SHAKE_THRESHOLD_G   1.5f    // Umbral de aceleración para detección de "shake"
#define MPU6050_TAP_THRESHOLD_G     3.0f    // Umbral de aceleración para detección de "tap"

/*******************************************************************************
 * TIPOS DE DATOS
 *******************************************************************************/

typedef struct {
    float x;
    float y;
    float z;
} MPU6050_Accel_t;

typedef struct {
    float x;
    float y;
    float z;
} MPU6050_Gyro_t;

typedef struct {
    MPU6050_Accel_t accel;
    MPU6050_Gyro_t gyro;
    float temp; // Temperatura en grados Celsius
} MPU6050_Data_t;

typedef enum {
    MOVEMENT_NONE,
    MOVEMENT_FORWARD,
    MOVEMENT_BACKWARD,
    MOVEMENT_LEFT,
    MOVEMENT_RIGHT,
    MOVEMENT_TILT_FORWARD,
    MOVEMENT_TILT_BACKWARD,
    MOVEMENT_TILT_LEFT,
    MOVEMENT_TILT_RIGHT,
    MOVEMENT_SHAKE,
    MOVEMENT_TAP
} MPU6050_Movement_t;

typedef struct {
    MPU6050_Accel_t current_accel;
    MPU6050_Gyro_t current_gyro;
    float current_temp;
    MPU6050_Accel_t filtered_accel; // Para datos filtrados
    MPU6050_Gyro_t filtered_gyro;   // Para datos filtrados
} MPU6050_Movement_Detector_t;

/*******************************************************************************
 * CONFIGURACIÓN DE FUNCIONALIDADES (Descomentar para habilitar)
 *******************************************************************************/

// Descomenta para habilitar la salida de depuración por UART
// #define MPU6050_DEBUG_ENABLED

// Descomenta para habilitar calibración automática al inicio
#define MPU6050_AUTO_CALIBRATE

// Descomenta para habilitar filtro de Kalman (requiere más recursos)
// #define MPU6050_KALMAN_FILTER

// Descomenta para habilitar detección de gestos avanzados
#define MPU6050_ADVANCED_GESTURES

/*******************************************************************************
 * INFORMACIÓN DEL MÓDULO
 *******************************************************************************/

#define MPU6050_DMP_VERSION_MAJOR   1
#define MPU6050_DMP_VERSION_MINOR   0
#define MPU6050_DMP_VERSION_PATCH   0
#define MPU6050_DMP_VERSION_STRING  "1.0.0-accessibility"

/*******************************************************************************
 * FUNCIONES PÚBLICAS
 *******************************************************************************/

/**
 * @brief Inicializa el sensor MPU6050.
 * @return true si la inicialización fue exitosa, false en caso contrario.
 */
bool MPU6050_Init(void);

/**
 * @brief Calibra el sensor MPU6050 (calcula offsets en reposo).
 */
void MPU6050_Calibrate(void);

/**
 * @brief Lee los datos crudos del acelerómetro, giroscopio y temperatura.
 * @param data Puntero a la estructura donde se guardarán los datos.
 */
void MPU6050_ReadSensorData(MPU6050_Data_t *data);

/**
 * @brief Detecta el movimiento actual basado en los datos del sensor.
 * @param data Puntero a la estructura con los datos del sensor.
 * @return Tipo de movimiento detectado.
 */
MPU6050_Movement_t MPU6050_GetMovement(MPU6050_Data_t *data);

#endif /* I2C_MPU6050_H */

/*******************************************************************************
 * NOTAS DE IMPLEMENTACIÓN:
 *
 * 1. Esta librería está específicamente diseñada para detectar movimientos
 * direccionales en aplicaciones de accesibilidad.
 *
 * 2. Utiliza el I2C Host de MCC (I2C1) para comunicarse con el MPU6050.
 *
 * 3. El "DMP" simulado combina datos del acelerómetro y giroscopio para
 * detectar movimientos específicos.
 *
 * 4. Los umbrales pueden ajustarse según las necesidades específicas del
 * usuario final.
 *
 * 5. Incluye filtros digitales para reducir ruido y mejorar la precisión.
 *
 * 6. Compatible con el código generado por MCC para PIC18F57Q43.
 *******************************************************************************/