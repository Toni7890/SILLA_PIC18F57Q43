#include "I2C_LCD.h"

/* ------------------------------------------------------------------ */
/* Funciones auxiliares mejoradas                                    */
/* ------------------------------------------------------------------ */
static uint8_t word = 0;

/* Envío físico de un byte al PCF8574 con verificación bloqueante ---- */
static void PCF_Wr(uint8_t data)
{
    /* Mantener LED encendido (bit P3) */
    data |= (1 << _LCD_LED);

    uint8_t tmp = data;
    
    /* CRÍTICO: Esperar hasta que I2C esté disponible */
    while(I2C1_IsBusy()) {
        __delay_us(10);  // Pequeña espera para evitar bloqueo total
    }
    
    /* Enviar el dato usando la función I2C1_Write de MCC */
    bool result = I2C1_Write(0x27, &tmp, 1);
    
    if(result) {
        /* CRÍTICO: Esperar hasta que la transmisión termine completamente */
        while(I2C1_IsBusy()) {
            __delay_us(10);
        }
        
        /* Delay adicional para asegurar que el PCF8574 procese el comando */
        __delay_us(50);
    }
}

/* Pulso EN con los 4 bits ya colocados ------------------------------- */
static void _write_nibble(uint8_t nibble)
{
    /* Configurar los bits de datos D4-D7 */
    if (nibble & 0x01) word |=  (1 << _LCD_D4); else word &= ~(1 << _LCD_D4);
    if (nibble & 0x02) word |=  (1 << _LCD_D5); else word &= ~(1 << _LCD_D5);
    if (nibble & 0x04) word |=  (1 << _LCD_D6); else word &= ~(1 << _LCD_D6);
    if (nibble & 0x08) word |=  (1 << _LCD_D7); else word &= ~(1 << _LCD_D7);

    /* Secuencia de pulso EN: Low -> High -> Low */
    word &= ~(1 << _LCD_EN); PCF_Wr(word); __delay_us(1);
    word |=  (1 << _LCD_EN); PCF_Wr(word); __delay_us(1);
    word &= ~(1 << _LCD_EN); PCF_Wr(word); __delay_us(50);
}

/* Envío de comandos o datos al HD44780 ------------------------------- */
static void _lcd_send(uint8_t value, uint8_t mode)
{
    /* Configurar RS según el modo (comando o dato) */
    if (mode == _COMMAND_) 
        word &= ~(1 << _LCD_RS); 
    else 
        word |=  (1 << _LCD_RS);
    
    /* RW siempre en 0 (escribir) */
    word &= ~(1 << _LCD_RW); 
    PCF_Wr(word);
    __delay_us(1);

    /* Enviar nibble alto primero, luego nibble bajo */
    _write_nibble(value >> 4);  
    _write_nibble(value & 0x0F);
}

/* ------------------------------------------------------------------ */
/* API pública mejorada                                               */
/* ------------------------------------------------------------------ */

void i2c_lcd_init(void)
{
    /* Asegurar que el I2C esté completamente inicializado */
    __delay_ms(100);
    
    /* Reset inicial del PCF8574 */
    word = 0;
    PCF_Wr(word);
    __delay_ms(50);
    
    /* Secuencia de inicialización del HD44780 según datasheet */
    /* Esperar más de 15 ms después de VCC sube a 2.7V */
    __delay_ms(50);
    
    /* Configuración inicial a 8 bits */
    word = 0x30;  /* 8-bit mode */
    word |= (1 << _LCD_LED);  /* LED encendido */
    PCF_Wr(word);
    __delay_ms(5);
    
    word |= (1 << _LCD_EN); PCF_Wr(word); __delay_us(1);
    word &= ~(1 << _LCD_EN); PCF_Wr(word); __delay_ms(5);
    
    /* Segunda vez */
    word |= (1 << _LCD_EN); PCF_Wr(word); __delay_us(1);
    word &= ~(1 << _LCD_EN); PCF_Wr(word); __delay_us(200);
    
    /* Tercera vez */
    word |= (1 << _LCD_EN); PCF_Wr(word); __delay_us(1);
    word &= ~(1 << _LCD_EN); PCF_Wr(word); __delay_us(200);
    
    /* Cambiar a modo 4 bits */
    word = 0x20;  /* 4-bit mode */
    word |= (1 << _LCD_LED);
    PCF_Wr(word);
    __delay_us(1);
    
    word |= (1 << _LCD_EN); PCF_Wr(word); __delay_us(1);
    word &= ~(1 << _LCD_EN); PCF_Wr(word); __delay_us(200);
    
    /* Configuración final del LCD */
    i2c_lcd_command(_LCD_FUNTIONSET | _LCD_2LINE | _LCD_5x8DOTS);
    __delay_us(50);
    
    i2c_lcd_command(_LCD_DISPLAYCONTROL | _LCD_DISPLAY_OFF);
    __delay_us(50);
    
    i2c_lcd_clear();
    
    i2c_lcd_command(_LCD_ENTRYMODESET | _LCD_ENTRYLEFT | _LCD_SHIFT_OFF);
    __delay_us(50);
    
    i2c_lcd_command(_LCD_DISPLAYCONTROL | _LCD_DISPLAY_ON | _LCD_CURSOR_OFF | _LCD_BLINK_OFF);
    __delay_us(50);
}

void i2c_lcd_write(uint8_t ch)
{
    _lcd_send(ch, _DATA_);
    __delay_us(50);  /* Tiempo para procesar el carácter */
}

void i2c_lcd_command(uint8_t cmd)
{
    _lcd_send(cmd, _COMMAND_);
    
    /* Delays específicos según el comando */
    if(cmd == _LCD_CLEARDISPLAY || cmd == _LCD_RETURNHOME) {
        __delay_ms(2);  /* Comandos lentos */
    } else {
        __delay_us(50);  /* Comandos normales */
    }
}

void i2c_lcd_puts(char *str)
{
    while(*str)
    {
        i2c_lcd_write(*str++);
    }
}

void i2c_lcd_clear(void)
{
    i2c_lcd_command(_LCD_CLEARDISPLAY);
    __delay_ms(2);
}

void i2c_lcd_goto(uint8_t col, uint8_t row)
{
    uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    if(row < 4 && col < 20) {  /* Validación de límites */
        i2c_lcd_command(_LCD_SET_DDRAM_ADDR | (col + row_offsets[row]));
    }
}

/* Funciones adicionales sin cambios... */
void i2c_lcd_enable_display(void)
{
    i2c_lcd_command(_LCD_DISPLAYCONTROL | _LCD_DISPLAY_ON);
}

void i2c_lcd_disable_display(void)
{
    i2c_lcd_command(_LCD_DISPLAYCONTROL | _LCD_DISPLAY_OFF);
}

void i2c_lcd_enable_blink(void)
{
    i2c_lcd_command(_LCD_DISPLAYCONTROL | _LCD_DISPLAY_ON | _LCD_BLINK_ON);
}

void i2c_lcd_disable_blink(void)
{
    i2c_lcd_command(_LCD_DISPLAYCONTROL | _LCD_DISPLAY_ON | _LCD_BLINK_OFF);
}

void i2c_lcd_enable_cursor(void)
{
    i2c_lcd_command(_LCD_DISPLAYCONTROL | _LCD_DISPLAY_ON | _LCD_CURSOR_ON);
}

void i2c_lcd_disable_cursor(void)
{
    i2c_lcd_command(_LCD_DISPLAYCONTROL | _LCD_DISPLAY_ON | _LCD_CURSOR_OFF);
}

void i2c_lcd_scroll_left(void)
{
    i2c_lcd_command(_LCD_CURSORDISPLAYSHIFT | _LCD_DISPLAY_SHIFT | _LCD_MOVELEFT);
}

void i2c_lcd_scroll_right(void)
{
    i2c_lcd_command(_LCD_CURSORDISPLAYSHIFT | _LCD_DISPLAY_SHIFT | _LCD_MOVERIGHT);
}