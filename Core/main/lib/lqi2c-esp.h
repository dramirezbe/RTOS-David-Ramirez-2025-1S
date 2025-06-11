/**
 * @file lqi2c-esp.h
 */

#ifndef LIQUIDCRYSTAL_ESPIDF_H
#define LIQUIDCRYSTAL_ESPIDF_H

#include <stdio.h>
#include "esp_err.h"
#include "driver/i2c_master.h" // ¡Importante! Usamos la nueva API I2C Master

// Definiciones de los comandos y bits HD44780
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORDISPLAYSHIFT  0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80

// Banderas para LCD_ENTRYMODESET
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// Banderas para LCD_DISPLAYCONTROL
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00

// Banderas para LCD_FUNCTIONSET
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00

// Banderas para el PCF8574 (backlight y pines de control)
#define LCD_BACKLIGHT           0x08
#define LCD_NOBACKLIGHT         0x00
#define LCD_EN                  0x04  // Enable bit
#define LCD_RW                  0x02  // Read/Write bit
#define LCD_RS                  0x01  // Register select bit

/**
 * @brief Estructura para manejar una instancia de la pantalla LCD I2C.
 */
typedef struct {
    i2c_master_bus_handle_t i2c_bus_handle;   // Handle para el bus I2C
    i2c_master_dev_handle_t i2c_dev_handle;   // Handle para el dispositivo I2C (LCD)
    uint8_t display_function; // Banderas de función del display (4bit, 2line, 5x8dots)
    uint8_t display_control;  // Banderas de control del display (display on/off, cursor on/off, blink on/off)
    uint8_t display_mode;     // Banderas de modo de entrada del display (entry left/right, shift increment/decrement)
    uint8_t num_cols;         // Número de columnas
    uint8_t num_rows;         // Número de filas
    uint8_t backlight_state;  // Estado actual del backlight
} lqi2c_t; // Renombrado a lqi2c_t

/**
 * @brief Inicializa una nueva instancia de la pantalla LCD I2C.
 * @param lcd Puntero a la estructura lqi2c_t a inicializar.
 * @param i2c_num Puerto I2C a usar (I2C_NUM_0 o I2C_NUM_1).
 * @param i2c_addr Dirección I2C del módulo LCD (comúnmente 0x27 o 0x3F).
 * @param sda_io Pin GPIO para SDA.
 * @param scl_io Pin GPIO para SCL.
 * @param cols Número de columnas de la LCD.
 * @param rows Número de filas de la LCD.
 */
void lqi2c_init(
    lqi2c_t *lcd,
    i2c_port_t i2c_num,
    uint8_t i2c_addr,
    gpio_num_t sda_io,
    gpio_num_t scl_io,
    uint8_t cols,
    uint8_t rows
);

/**
 * @brief Limpia la pantalla y posiciona el cursor en la esquina superior izquierda.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_clear(lqi2c_t *lcd);

/**
 * @brief Retorna el cursor a la posición inicial (0,0) sin limpiar la pantalla.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_home(lqi2c_t *lcd);

/**
 * @brief Posiciona el cursor en una columna y fila específicas.
 * @param lcd Puntero a la instancia de lqi2c_t.
 * @param col Columna (0-indexed).
 * @param row Fila (0-indexed).
 */
void lqi2c_setCursor(lqi2c_t *lcd, uint8_t col, uint8_t row);

/**
 * @brief Escribe un solo byte (carácter) en la pantalla LCD.
 * @param lcd Puntero a la instancia de lqi2c_t.
 * @param data Byte a escribir.
 */
void lqi2c_write(lqi2c_t *lcd, uint8_t data);

/**
 * @brief Imprime una cadena de caracteres en la pantalla LCD.
 * @param lcd Puntero a la instancia de lqi2c_t.
 * @param str Cadena de texto a imprimir.
 */
void lqi2c_print(lqi2c_t *lcd, const char *str);

/**
 * @brief Enciende el display LCD.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_display(lqi2c_t *lcd);

/**
 * @brief Apaga el display LCD.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_noDisplay(lqi2c_t *lcd);

/**
 * @brief Enciende el cursor (subrayado).
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_cursor(lqi2c_t *lcd);

/**
 * @brief Apaga el cursor.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_noCursor(lqi2c_t *lcd);

/**
 * @brief Enciende el parpadeo del cursor.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_blink(lqi2c_t *lcd);

/**
 * @brief Apaga el parpadeo del cursor.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_noBlink(lqi2c_t *lcd);

/**
 * @brief Mueve el contenido de la pantalla hacia la derecha.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_scrollDisplayRight(lqi2c_t *lcd);

/**
 * @brief Mueve el contenido de la pantalla hacia la izquierda.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_scrollDisplayLeft(lqi2c_t *lcd);

/**
 * @brief Establece la dirección del texto de izquierda a derecha.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_leftToRight(lqi2c_t *lcd);

/**
 * @brief Establece la dirección del texto de derecha a izquierda.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_rightToLeft(lqi2c_t *lcd);

/**
 * @brief Habilita el auto desplazamiento de la pantalla.
 * Si está habilitado, cada nuevo carácter empujará el contenido existente.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_autoscroll(lqi2c_t *lcd);

/**
 * @brief Deshabilita el auto desplazamiento de la pantalla.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_noAutoscroll(lqi2c_t *lcd);

/**
 * @brief Enciende el backlight de la pantalla.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_backlight(lqi2c_t *lcd);

/**
 * @brief Apaga el backlight de la pantalla.
 * @param lcd Puntero a la instancia de lqi2c_t.
 */
void lqi2c_noBacklight(lqi2c_t *lcd);

#endif // LIQUIDCRYSTAL_ESPIDF_H