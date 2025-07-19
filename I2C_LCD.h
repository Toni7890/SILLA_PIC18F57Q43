#ifndef LCD_LIB_H
#define LCD_LIB_H

#include <xc.h>
#include <stdint.h>

/* Frecuencia del sistema (MCC ya la define; esto es por si se compila aparte) */
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 64000000UL
#endif

/* Driver Melody I²C1 ---------------------------------------------------- */
#include "mcc_generated_files/i2c_host/i2c1.h"

/* Utilidades de retardo */
#define _delay_ms(x)  __delay_ms(x)
#define _delay_us(x)  __delay_us(x)

/* ------------------------ Pines del PCF8574 ---------------------------- */
#define _LCD_RS   0
#define _LCD_RW   1
#define _LCD_EN   2
#define _LCD_LED  3
#define _LCD_D4   4
#define _LCD_D5   5
#define _LCD_D6   6
#define _LCD_D7   7

/* ------------------------ Parámetros de display ------------------------ */
#define _LCD_nCOL_ 20
#define _LCD_nROW_ 4

/* --------------------- Conjunto de comandos HD44780 -------------------- */
#define _COMMAND_             0
#define _DATA_                1

#define _LCD_CLEARDISPLAY     0x01
#define _LCD_RETURNHOME       0x02

/* Entry Mode */
#define _LCD_ENTRYMODESET     0x04
#define _LCD_ENTRYRIGHT       0x02
#define _LCD_ENTRYLEFT        0x00
#define _LCD_SHIFT_ON         0x01
#define _LCD_SHIFT_OFF        0x00

/* Display / Cursor / Blink */
#define _LCD_DISPLAYCONTROL   0x08
#define _LCD_DISPLAY_ON       0x04
#define _LCD_DISPLAY_OFF      0x00
#define _LCD_CURSOR_ON        0x02
#define _LCD_CURSOR_OFF       0x00
#define _LCD_BLINK_ON         0x01
#define _LCD_BLINK_OFF        0x00

/* Cursor / Display Shift */
#define _LCD_CURSORDISPLAYSHIFT 0x10
#define _LCD_DISPLAY_SHIFT      0x08
#define _LCD_CURSOR_SHIFT       0x00
#define _LCD_MOVERIGHT          0x04
#define _LCD_MOVELEFT           0x00

/* Function Set */
#define _LCD_FUNTIONSET 0x20
#define _LCD_4BITMODE   0x00
#define _LCD_2LINE      0x08
#define _LCD_5x8DOTS    0x00

/* Dirección de CGRAM / DDRAM */
#define _LCD_SET_CGRAM_ADDR  0x40
#define _LCD_SET_DDRAM_ADDR  0x80

/* ------------------------- API pública --------------------------------- */
void i2c_lcd_init(void);
void i2c_lcd_write(uint8_t ch);
void i2c_lcd_command(uint8_t cmd);

void i2c_lcd_puts(char *str);
void i2c_lcd_clear(void);
void i2c_lcd_goto(uint8_t col, uint8_t row);

void i2c_lcd_enable_display(void);
void i2c_lcd_disable_display(void);
void i2c_lcd_enable_blink(void);
void i2c_lcd_disable_blink(void);
void i2c_lcd_enable_cursor(void);
void i2c_lcd_disable_cursor(void);
void i2c_lcd_scroll_left(void);
void i2c_lcd_scroll_right(void);

#endif // LCD_LIB_H