/*******************************************************************************
 * I2C_MPU6050.c - Implementación de la Librería MPU6050 para Detección de Movimientos
 *
 * PROYECTO: Sistema de Control con Movimientos para Personas con Discapacidad Motora
 * MICROCONTROLADOR: PIC18F57Q43
 * COMUNICACIÓN: I2C Host de MCC (RB1=SCL, RB2=SDA)
 *
 * AUTOR: Ingeniero Embebido
 * FECHA: 2025
 *******************************************************************************/\

#include "I2C_MPU6050.h"
#include <stdio.h> // Para sprintf, si MPU6050_DEBUG_ENABLED está definido
// ***************************************************************
// Incluir el driver I2C de MCC
#include "mcc_generated_files/i2c_host/i2c1.h"
// ***************************************************************
// ***************************************************************
// Añadido: Inclusiones directas para macros de retardo
#include <xc.h> // Contiene los macros __delay_ms y __delay_us
#include "mcc_generated_files/system/clock.h" // Contiene la definición de _XTAL_FREQ
// ***************************************************************

// Incluir el driver UART para depuración si está habilitado
#ifdef MPU6050_DEBUG_ENABLED
#include "mcc_generated_files/uart/uart1.h"
#endif

// Para retardos (Estas líneas ya no son estrictamente necesarias aquí si xc.h y clock.h se incluyen directamente)
// #ifndef _XTAL_FREQ
// #define _XTAL_FREQ 64000000UL // Asegurarse de que esté definido para __delay_ms/__delay_us
// #endif

/*******************************************************************************
 * VARIABLES GLOBALES
 *******************************************************************************/

// Offsets de calibración
static MPU6050_Accel_t accel_offset = {0.0f, 0.0f, 0.0f};
static MPU6050_Gyro_t gyro_offset = {0.0f, 0.0f, 0.0f};

// Detector de movimientos
static MPU6050_Movement_Detector_t movement_detector;

// Flag de inicialización
static bool mpu6050_initialized = false;

// Último timestamp para cálculo de tiempo (si usas filtrado complementario o Kalman)
static uint16_t last_timestamp = 0; // Se puede usar TMR2 o TMR4 u otro Timer del MCC

// Escala de sensibilidad (LSB/unidad) - Ajusta según el rango seleccionado en MPU6050_Init
static float accel_lsb_per_g = 16384.0f; // Por defecto para +/- 2g
static float gyro_lsb_per_dps = 131.0f;  // Por defecto para +/- 250 dps

/*******************************************************************************
 * FUNCIONES PRIVADAS - PROTOTIPO
 *******************************************************************************/\
static bool MPU6050_WriteRegister(uint8_t reg, uint8_t data);
static uint8_t MPU6050_ReadRegister(uint8_t reg);
static void MPU6050_ReadSensorRaw(int16_t *accelX, int16_t *accelY, int16_t *accelZ,
                                  int16_t *temp,
                                  int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ);
#ifdef MPU6050_KALMAN_FILTER
// static void Kalman_Filter_Update(float gyro_angle, float accel_angle, float dt);
#endif
#ifdef MPU6050_ADVANCED_GESTURES
static MPU6050_Movement_t DetectAdvancedGestures(MPU6050_Data_t *data);
static uint8_t CalculateConfidence(MPU6050_Movement_t movement, MPU6050_Data_t *data);
#endif

/*******************************************************************************
 * IMPLEMENTACIÓN DE FUNCIONES PRIVADAS
 *******************************************************************************/\

/**
 * @brief Escribe un byte en un registro del MPU6050 usando I2C de MCC.
 * @param reg Dirección del registro.
 * @param data Byte de datos a escribir.
 * @return true si la escritura fue exitosa, false en caso contrario (ej. NACK).
 */
static bool MPU6050_WriteRegister(uint8_t reg, uint8_t data)
{
    uint8_t tx_buffer[2];
    tx_buffer[0] = reg;
    tx_buffer[1] = data;

    // ***************************************************************
    // Adaptación a I2C1_Write de MCC con verificación de ocupado
    while(I2C1_IsBusy()) {
        __delay_us(10); // Esperar si el bus I2C está ocupado
    }
    bool result = I2C1_Write(MPU6050_I2C_ADDR, tx_buffer, 2); // Escribe 2 bytes (registro + dato)
    if(result) { // Si la solicitud de escritura se inició correctamente
        while(I2C1_IsBusy()) {
            __delay_us(10); // Esperar hasta que la transmisión I2C termine
        }
        // Opcional: Verificar I2C1_ErrorGet() si se desea un manejo de errores más granular aquí
    }
    __delay_us(50); // Pequeño retardo después de la transacción I2C
    return result;
    // ***************************************************************
}

/**
 * @brief Lee un byte de un registro del MPU6050 usando I2C de MCC.
 * @param reg Dirección del registro a leer.
 * @return Byte de datos leído.
 */
static uint8_t MPU6050_ReadRegister(uint8_t reg)
{
    uint8_t data_read = 0;
    uint8_t reg_addr = reg;

    // ***************************************************************
    // Adaptación a I2C1_WriteRead de MCC con verificación de ocupado
    while(I2C1_IsBusy()) {
        __delay_us(10); // Esperar si el bus I2C está ocupado
    }
    // Escribe la dirección del registro, luego lee 1 byte
    bool result = I2C1_WriteRead(MPU6050_I2C_ADDR, &reg_addr, 1, &data_read, 1); //
    if(result) { // Si la solicitud de lectura se inició correctamente
        while(I2C1_IsBusy()) {
            __delay_us(10); // Esperar hasta que la transmisión I2C termine
        }
        // Opcional: Verificar I2C1_ErrorGet()
    }
    __delay_us(50); // Pequeño retardo después de la transacción I2C
    return data_read;
    // ***************************************************************
}

/**
 * @brief Lee los datos crudos del sensor MPU6050.
 * @param accelX, accelY, accelZ Punteros a variables para aceleración cruda.
 * @param temp Puntero a variable para temperatura cruda.
 * @param gyroX, gyroY, gyroZ Punteros a variables para giroscopio crudo.
 */
static void MPU6050_ReadSensorRaw(int16_t *accelX, int16_t *accelY, int16_t *accelZ,
                                  int16_t *temp,
                                  int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ)
{
    uint8_t rawData[14]; // Buffer para los 14 bytes de datos de acelerómetro, temperatura y giroscopio
    uint8_t reg_addr = MPU6050_ACCEL_XOUT_H; // Dirección del primer registro de datos

    // ***************************************************************
    // Adaptación a I2C1_WriteRead para leer múltiples bytes secuenciales
    while(I2C1_IsBusy()) {
        __delay_us(10);
    }
    // Escribe la dirección del primer registro (MPU6050_ACCEL_XOUT_H)
    // luego lee 14 bytes secuencialmente a partir de esa dirección
    bool result = I2C1_WriteRead(MPU6050_I2C_ADDR, &reg_addr, 1, rawData, 14); //
    if(result) {
        while(I2C1_IsBusy()) {
            __delay_us(10);
        }
        // Opcional: Verificar I2C1_ErrorGet()
    }
    __delay_us(100); // Retardo después de la lectura de ráfaga
    // ***************************************************************

    // Reconstruir valores de 16 bits
    *accelX = (rawData[0] << 8) | rawData[1];
    *accelY = (rawData[2] << 8) | rawData[3];
    *accelZ = (rawData[4] << 8) | rawData[5];
    *temp   = (rawData[6] << 8) | rawData[7];
    *gyroX  = (rawData[8] << 8) | rawData[9];
    *gyroY  = (rawData[10] << 8) | rawData[11];
    *gyroZ  = (rawData[12] << 8) | rawData[13];
}

#ifdef MPU6050_KALMAN_FILTER
// Implementación del filtro de Kalman si está habilitado
// void Kalman_Filter_Update(float gyro_angle, float accel_angle, float dt) { ... }
#endif

#ifdef MPU6050_ADVANCED_GESTURES
/**
 * @brief Detecta gestos avanzados (shake, tap)
 */
static MPU6050_Movement_t DetectAdvancedGestures(MPU6050_Data_t *data) {
    float abs_ax = fabsf(data->accel.x);
    float abs_ay = fabsf(data->accel.y);
    float abs_az = fabsf(data->accel.z);

    // Detección de sacudida (cambio rápido de aceleración en cualquier eje)
#ifdef MPU6050_SHAKE_THRESHOLD_G
    if ((abs_ax > MPU6050_SHAKE_THRESHOLD_G) ||
        (abs_ay > MPU6050_SHAKE_THRESHOLD_G) ||
        (abs_az > MPU6050_SHAKE_THRESHOLD_G)) {
        return MOVEMENT_SHAKE;
    }
#endif

    // Detección de golpe (pico de aceleración corto)
#ifdef MPU6050_TAP_THRESHOLD_G
    if ((abs_ax + abs_ay + abs_az) > MPU6050_TAP_THRESHOLD_G) {
        return MOVEMENT_TAP;
    }
#endif

    return MOVEMENT_NONE;
}

/**
 * @brief Calcula nivel de confianza del movimiento detectado
 */
static uint8_t CalculateConfidence(MPU6050_Movement_t movement, MPU6050_Data_t *data) {

    uint8_t confidence = 50; // Confianza base

    switch (movement) {
        case MOVEMENT_FORWARD:
        case MOVEMENT_BACKWARD:
            // Mayor confianza si la aceleración Y es dominante
            if (fabsf(data->accel.y) > fabsf(data->accel.x) &&
                fabsf(data->accel.y) > fabsf(data->accel.z)) {
                confidence += 30;
            }
            break;

        case MOVEMENT_LEFT:
        case MOVEMENT_RIGHT:
            // Mayor confianza si el giro Z es dominante
            if (fabsf(data->gyro.z) > fabsf(data->gyro.x) &&
                fabsf(data->gyro.z) > fabsf(data->gyro.y)) {
                confidence += 30;
            }
            break;

        case MOVEMENT_TILT_FORWARD:
        case MOVEMENT_TILT_BACKWARD:
        case MOVEMENT_TILT_LEFT:
        case MOVEMENT_TILT_RIGHT:
            // Mayor confianza si el ángulo de inclinación es pronunciado
            // Esta lógica dependería de cómo calcules los ángulos de Roll/Pitch
            confidence += (uint8_t)(fabsf(data->accel.x) + fabsf(data->accel.y)); // Simplificado
            if (confidence > 100) confidence = 100;
            break;

        case MOVEMENT_SHAKE:
        case MOVEMENT_TAP:
            confidence = 100; // Alta confianza para estos gestos
            break;

        default:
            break;
    }

    return confidence;
}
#endif

/*******************************************************************************
 * IMPLEMENTACIÓN DE FUNCIONES PÚBLICAS
 *******************************************************************************/\

/**
 * @brief Inicializa el sensor MPU6050.
 * @return true si la inicialización fue exitosa, false en caso contrario.
 */
bool MPU6050_Init(void)
{
    // Verificar si el MPU6050 está presente (WHO_AM_I register)
    if (MPU6050_ReadRegister(MPU6050_WHO_AM_I) != MPU6050_I2C_ADDR) { // MPU6050_I2C_ADDR es 0x68
#ifdef MPU6050_DEBUG_ENABLED
        // Asumiendo que UART1_sendString existe en main.c o en un archivo común
        // Nota: Asegúrate de que UART1_sendString esté accesible o incluido aquí si MPU6050_DEBUG_ENABLED está activo.
        // Si no está incluido, puedes agregar #include "mcc_generated_files/uart/uart1.h" al inicio.
        // Y también asegúrate de que UART1_sendString no use sprintf si este archivo no incluye <stdio.h>
        UART1_sendString("MPU6050: WHO_AM_I incorrecto. No detectado.\r\n");
#endif
        return false;
    }

    // Resetear el MPU6050
    MPU6050_WriteRegister(MPU6050_PWR_MGMT_1, 0x80); // Resetear dispositivo
    __delay_ms(100);

    // Activar el sensor (sacarlo del modo SLEEP)
    MPU6050_WriteRegister(MPU6050_PWR_MGMT_1, 0x00); // Sacar del sleep, usar X-axis Gyro clock
    __delay_ms(100);

    // Configurar el Sample Rate (1kHz / (1+SMPRT_DIV))
    MPU6050_WriteRegister(MPU6050_SMPRT_DIV, 0x07); // Sample Rate = 1kHz / (1+7) = 125Hz

    // Configurar el Digital Low Pass Filter (DLPF)
    MPU6050_WriteRegister(MPU6050_CONFIG, 0x03); // DLPF_CFG = 3 (Bandwidth: Accel 44Hz, Gyro 42Hz, Delay 4.9ms)

    // Configurar rango del giroscopio (ej. +/- 250 grados/seg)
    MPU6050_WriteRegister(MPU6050_GYRO_CONFIG, GYRO_RANGE_250_DPS);
    gyro_lsb_per_dps = 131.0f; // Actualizar la escala para la conversión

    // Configurar rango del acelerómetro (ej. +/- 2g)
    MPU6050_WriteRegister(MPU6050_ACCEL_CONFIG, ACCEL_RANGE_2G);
    accel_lsb_per_g = 16384.0f; // Actualizar la escala para la conversión

    mpu6050_initialized = true;
    return true;
}

/**
 * @brief Calibra el sensor MPU6050 (calcula offsets en reposo).
 */
void MPU6050_Calibrate(void)
{
#ifdef MPU6050_AUTO_CALIBRATE
    if (!mpu6050_initialized) return;

    int32_t temp_accelX = 0, temp_accelY = 0, temp_accelZ = 0;
    int32_t temp_gyroX = 0, temp_gyroY = 0, temp_gyroZ = 0;
    int16_t ax, ay, az, gx, gy, gz, temp;

#ifdef MPU6050_DEBUG_ENABLED
    // Asumiendo que UART1_sendString existe
    UART1_sendString("MPU6050: Iniciando calibracion...\r\n");
    __delay_ms(100);
#endif

    // Tomar múltiples muestras en reposo
    for (uint16_t i = 0; i < MPU6050_CALIBRATION_SAMPLES; i++) {
        MPU6050_ReadSensorRaw(&ax, &ay, &az, &temp, &gx, &gy, &gz);
        temp_accelX += ax;
        temp_accelY += ay;
        temp_accelZ += az;
        temp_gyroX += gx;
        temp_gyroY += gy;
        temp_gyroZ += gz;
        __delay_ms(10); // Pequeño retardo entre muestras
    }

    // Calcular promedios
    accel_offset.x = (float)temp_accelX / MPU6050_CALIBRATION_SAMPLES;
    accel_offset.y = (float)temp_accelY / MPU6050_CALIBRATION_SAMPLES;
    accel_offset.z = (float)temp_accelZ / MPU6050_CALIBRATION_SAMPLES;
    gyro_offset.x = (float)temp_gyroX / MPU6050_CALIBRATION_SAMPLES;
    gyro_offset.y = (float)temp_gyroY / MPU6050_CALIBRATION_SAMPLES;
    gyro_offset.z = (float)temp_gyroZ / MPU6050_CALIBRATION_SAMPLES;

    // Ajustar offset Z del acelerómetro para la gravedad (si está orientado con Z hacia arriba)
    // Suponiendo que Z apunta hacia arriba en reposo, su valor debe ser +1g.
    // Si apunta hacia abajo, sería -1g. Ajusta según la orientación física del sensor.
    accel_offset.z -= accel_lsb_per_g; // Restar 1g (en LSBs)

#ifdef MPU6050_DEBUG_ENABLED
    char buffer[64];
    sprintf(buffer, "MPU6050: Calibracion finalizada.\r\n");
    UART1_sendString(buffer);
    sprintf(buffer, "Offsets Accel: X:%.2f Y:%.2f Z:%.2f\r\n", (double)accel_offset.x, (double)accel_offset.y, (double)accel_offset.z);
    UART1_sendString(buffer);
    sprintf(buffer, "Offsets Gyro: X:%.2f Y:%.2f Z:%.2f\r\r\n", (double)gyro_offset.x, (double)gyro_offset.y, (double)gyro_offset.z);
    UART1_sendString(buffer);
#endif

#endif // MPU6050_AUTO_CALIBRATE
}

/**
 * @brief Lee los datos crudos del acelerómetro, giroscopio y temperatura, aplica offsets y escala.
 * @param data Puntero a la estructura donde se guardarán los datos.
 */
void MPU6050_ReadSensorData(MPU6050_Data_t *data)
{
    if (!mpu6050_initialized) return;

    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, temp_raw;

    MPU6050_ReadSensorRaw(&ax_raw, &ay_raw, &az_raw, &temp_raw, &gx_raw, &gy_raw, &gz_raw);

    // Aplicar offsets de calibración y escalar a Gs o DPS
    // Los casts a (double) en sprintf son importantes para evitar advertencias de formato
    data->accel.x = ((float)ax_raw - accel_offset.x) / accel_lsb_per_g;
    data->accel.y = ((float)ay_raw - accel_offset.y) / accel_lsb_per_g;
    data->accel.z = ((float)az_raw - accel_offset.z) / accel_lsb_per_g;

    data->gyro.x = ((float)gx_raw - gyro_offset.x) / gyro_lsb_per_dps;
    data->gyro.y = ((float)gy_raw - gyro_offset.y) / gyro_lsb_per_dps;
    data->gyro.z = ((float)gz_raw - gyro_offset.z) / gyro_lsb_per_dps;

    // Convertir temperatura a grados Celsius
    data->temp = (temp_raw / 340.0f) + 36.53f;

    // Si se usa filtro de Kalman o complementario, aplicarlo aquí
#ifdef MPU6050_KALMAN_FILTER
    // Aquí se necesitaría el cálculo de dt (delta tiempo)
    // Kalman_Filter_Update(data->gyro.x, data->accel.x, dt); // Ejemplo
#endif
    // En este driver, los datos filtrados se almacenan directamente
    movement_detector.current_accel = data->accel;
    movement_detector.current_gyro = data->gyro;
    movement_detector.current_temp = data->temp;
}

/**
 * @brief Detecta el movimiento actual basado en los datos del sensor.
 * @param data Puntero a la estructura con los datos del sensor.
 * @return Tipo de movimiento detectado.
 */
MPU6050_Movement_t MPU6050_GetMovement(MPU6050_Data_t *data)
{
    // Los datos ya vienen con offsets y escala
    // Usar fabsf para valores absolutos de floats
    float roll_accel = (atan2f(data->accel.y, data->accel.z) * 180.0f / (float)M_PI);
    float pitch_accel = (atan2f(-data->accel.x, sqrtf(data->accel.y*data->accel.y + data->accel.z*data->accel.z)) * 180.0f / (float)M_PI);

    // Considerar los offsets de calibración si MPU6050_AUTO_CALIBRATE está definido y son significativos
    // roll_accel -= offsetRoll;
    // pitch_accel -= offsetPitch;

    // Detección de inclinación para control direccional
    // Ajusta estos umbrales según el montaje del MPU6050 y la sensibilidad deseada
    // MPU6050_TILT_THRESHOLD_DEG es 15.0f por defecto en I2C_MPU6050.h
    if (roll_accel > MPU6050_TILT_THRESHOLD_DEG) {
        return MOVEMENT_RIGHT;
    } else if (roll_accel < -MPU6050_TILT_THRESHOLD_DEG) {
        return MOVEMENT_LEFT;
    } else if (pitch_accel > MPU6050_TILT_THRESHOLD_DEG) {
        return MOVEMENT_FORWARD;
    } else if (pitch_accel < -MPU6050_TILT_THRESHOLD_DEG) {
        return MOVEMENT_BACKWARD;
    }

#ifdef MPU6050_ADVANCED_GESTURES
    MPU6050_Movement_t advanced_movement = DetectAdvancedGestures(data);
    if (advanced_movement != MOVEMENT_NONE) {
        return advanced_movement;
    }
#endif

    return MOVEMENT_NONE; // Si no hay movimiento significativo
}