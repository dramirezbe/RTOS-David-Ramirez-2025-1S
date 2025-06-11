/**
 * @file lqi2c-esp.c
 */

#include "lqi2c-esp.h"
#include "esp_log.h"
#include "driver/i2c_master.h" // ¡Importante! Usamos la nueva API I2C Master
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h> // Para strlen

static const char *TAG = "LCD_I2C_LIB";

// --- Funciones internas de bajo nivel ---

/**
 * @brief Inicializa el bus I2C maestro y el dispositivo para la instancia LCD.
 * @param lcd Puntero a la instancia de lqi2c_t.
 * @param i2c_port Puerto I2C a usar.
 * @param i2c_addr Dirección I2C del módulo LCD.
 * @param sda_io Pin GPIO para SDA.
 * @param scl_io Pin GPIO para SCL.
 */
static void _i2c_master_init_internal(lqi2c_t *lcd, i2c_port_t i2c_port, uint8_t i2c_addr, gpio_num_t sda_io, gpio_num_t scl_io) {
    // Configuración del bus I2C
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .scl_io_num = scl_io,
        .sda_io_num = sda_io,
        .glitch_ignore_cnt = 7, // Valor predeterminado
        // Usar las banderas correctas para pull-ups en ESP-IDF v5.x
        .flags.enable_internal_pullup = true, // Habilitar pull-up interno en SDA y SCL
    };
    esp_err_t ret = i2c_new_master_bus(&bus_config, &lcd->i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        lcd->i2c_bus_handle = NULL; // Marcar como no inicializado
        lcd->i2c_dev_handle = NULL; // Marcar como no inicializado
        return;
    }

    // Configuración del dispositivo I2C (LCD)
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = 100000, // Frecuencia I2C de 100 kHz
    };
    ret = i2c_master_bus_add_device(lcd->i2c_bus_handle, &dev_config, &lcd->i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        // Si falla la adición del dispositivo, liberar el bus (opcional, pero buena práctica)
        i2c_del_master_bus(lcd->i2c_bus_handle);
        lcd->i2c_bus_handle = NULL;
        lcd->i2c_dev_handle = NULL;
        return;
    }
}

/**
 * @brief Envía un byte de datos al PCF8574 usando la nueva API i2c_master_transmit.
 * @param lcd Puntero a la instancia de lqi2c_t.
 * @param data Byte a enviar.
 */
static void _i2c_send_byte_internal(lqi2c_t *lcd, uint8_t data) {
    if (lcd->i2c_dev_handle == NULL) {
        ESP_LOGE(TAG, "I2C device handle is not initialized. Cannot send byte.");
        return;
    }
    uint8_t tx_data[1];
    tx_data[0] = data | lcd->backlight_state;
    // i2c_master_transmit automáticamente maneja el inicio, dirección, datos y parada.
    esp_err_t ret = i2c_master_transmit(lcd->i2c_dev_handle, tx_data, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transmit I2C byte: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Escribe un nibble (4 bits) en la pantalla LCD a través del PCF8574.
 * @param lcd Puntero a la instancia de lqi2c_t.
 * @param val Valor del nibble a enviar (bits 4-7 del byte).
 * @param mode Modo de envío (LCD_RS, LCD_RW).
 */
static void _lcd_write4bits_internal(lqi2c_t *lcd, uint8_t val, uint8_t mode) {
    uint8_t data = (val & 0xF0) | mode;
    _i2c_send_byte_internal(lcd, data | LCD_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
    _i2c_send_byte_internal(lcd, data & ~LCD_EN);
}

/**
 * @brief Envía un comando o dato a la pantalla LCD.
 * @param lcd Puntero a la instancia de lqi2c_t.
 * @param value Byte a enviar.
 * @param mode Modo (LCD_RS para dato, 0 para comando).
 */
static void _lcd_send_internal(lqi2c_t *lcd, uint8_t value, uint8_t mode) {
    _lcd_write4bits_internal(lcd, value & 0xF0, mode);
    _lcd_write4bits_internal(lcd, (value << 4) & 0xF0, mode);
}

/**
 * @brief Envía un comando a la pantalla LCD.
 * @param lcd Puntero a la instancia de lqi2c_t.
 * @param command Comando a enviar.
 */
static void _lcd_command_internal(lqi2c_t *lcd, uint8_t command) {
    _lcd_send_internal(lcd, command, LCD_RS & ~LCD_RW);
}

/**
 * @brief Envía un carácter (dato) a la pantalla LCD.
 * @param lcd Puntero a la instancia de lqi2c_t.
 * @param data Carácter a enviar.
 */
static void _lcd_write_char_internal(lqi2c_t *lcd, uint8_t data) {
    _lcd_send_internal(lcd, data, LCD_RS | ~LCD_RW);
}


// --- Implementación de las funciones públicas (lqi2c) ---

void lqi2c_init(
    lqi2c_t *lcd,
    i2c_port_t i2c_num,
    uint8_t i2c_addr,
    gpio_num_t sda_io,
    gpio_num_t scl_io,
    uint8_t cols,
    uint8_t rows
) {
    if (!lcd) {
        ESP_LOGE(TAG, "LCD instance is NULL");
        return;
    }

    lcd->num_cols = cols;
    lcd->num_rows = rows;
    lcd->backlight_state = LCD_BACKLIGHT; // Backlight encendido por defecto
    lcd->i2c_bus_handle = NULL; // Inicializar handles
    lcd->i2c_dev_handle = NULL;

    _i2c_master_init_internal(lcd, i2c_num, i2c_addr, sda_io, scl_io);

    // Si la inicialización I2C falló, no continuar con los comandos de la LCD
    if (lcd->i2c_dev_handle == NULL) {
        ESP_LOGE(TAG, "I2C initialization failed, aborting LCD init sequence.");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // Resetear LCD a modo 4-bit (secuencia de inicialización)
    _lcd_write4bits_internal(lcd, 0x03 << 4, LCD_RS & ~LCD_RW);
    vTaskDelay(pdMS_TO_TICKS(5));

    _lcd_write4bits_internal(lcd, 0x03 << 4, LCD_RS & ~LCD_RW);
    vTaskDelay(pdMS_TO_TICKS(5));

    _lcd_write4bits_internal(lcd, 0x03 << 4, LCD_RS & ~LCD_RW);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Pasar a modo 4-bit
    _lcd_write4bits_internal(lcd, 0x02 << 4, LCD_RS & ~LCD_RW);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Configuración de la función
    lcd->display_function = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
    if (rows > 1) {
        lcd->display_function |= LCD_2LINE;
    }
    _lcd_command_internal(lcd, LCD_FUNCTIONSET | lcd->display_function);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Encender la pantalla, apagar cursor, apagar parpadeo (estado por defecto)
    lcd->display_control = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    _lcd_command_internal(lcd, LCD_DISPLAYCONTROL | lcd->display_control);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Limpiar pantalla
    lqi2c_clear(lcd);
    vTaskDelay(pdMS_TO_TICKS(2));

    // Modo de entrada: incrementar dirección, no desplazar pantalla (estado por defecto)
    lcd->display_mode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    _lcd_command_internal(lcd, LCD_ENTRYMODESET | lcd->display_mode);
    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_LOGI(TAG, "LCD initialized on I2C port %d, address 0x%02X", i2c_num, i2c_addr);
}

void lqi2c_clear(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    _lcd_command_internal(lcd, LCD_CLEARDISPLAY);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lqi2c_home(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    _lcd_command_internal(lcd, LCD_RETURNHOME);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lqi2c_setCursor(lqi2c_t *lcd, uint8_t col, uint8_t row) {
    if (lcd->i2c_dev_handle == NULL) return;
    uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    if (row >= lcd->num_rows || col >= lcd->num_cols) {
        ESP_LOGE(TAG, "Invalid cursor position: col %d, row %d (max: %dx%d)", col, row, lcd->num_cols, lcd->num_rows);
        return;
    }
    _lcd_command_internal(lcd, LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void lqi2c_write(lqi2c_t *lcd, uint8_t data) {
    if (lcd->i2c_dev_handle == NULL) return;
    _lcd_write_char_internal(lcd, data);
}

void lqi2c_print(lqi2c_t *lcd, const char *str) {
    if (lcd->i2c_dev_handle == NULL) return;
    if (!str) {
        ESP_LOGE(TAG, "String is NULL");
        return;
    }
    size_t len = strlen(str);
    for (size_t i = 0; i < len; i++) {
        _lcd_write_char_internal(lcd, (uint8_t)str[i]);
    }
}

void lqi2c_display(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    lcd->display_control |= LCD_DISPLAYON;
    _lcd_command_internal(lcd, LCD_DISPLAYCONTROL | lcd->display_control);
}

void lqi2c_noDisplay(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    lcd->display_control &= ~LCD_DISPLAYON;
    _lcd_command_internal(lcd, LCD_DISPLAYCONTROL | lcd->display_control);
}

void lqi2c_cursor(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    lcd->display_control |= LCD_CURSORON;
    _lcd_command_internal(lcd, LCD_DISPLAYCONTROL | lcd->display_control);
}

void lqi2c_noCursor(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    lcd->display_control &= ~LCD_CURSORON;
    _lcd_command_internal(lcd, LCD_DISPLAYCONTROL | lcd->display_control);
}

void lqi2c_blink(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    lcd->display_control |= LCD_BLINKON;
    _lcd_command_internal(lcd, LCD_DISPLAYCONTROL | lcd->display_control);
}

void lqi2c_noBlink(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    lcd->display_control &= ~LCD_BLINKON;
    _lcd_command_internal(lcd, LCD_DISPLAYCONTROL | lcd->display_control);
}

void lqi2c_scrollDisplayRight(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    _lcd_command_internal(lcd, LCD_CURSORDISPLAYSHIFT | 0x08 | 0x04);
}

void lqi2c_scrollDisplayLeft(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    _lcd_command_internal(lcd, LCD_CURSORDISPLAYSHIFT | 0x08 | 0x00);
}

void lqi2c_leftToRight(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    lcd->display_mode |= LCD_ENTRYLEFT;
    _lcd_command_internal(lcd, LCD_ENTRYMODESET | lcd->display_mode);
}

void lqi2c_rightToLeft(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    lcd->display_mode &= ~LCD_ENTRYLEFT;
    _lcd_command_internal(lcd, LCD_ENTRYMODESET | lcd->display_mode);
}

void lqi2c_autoscroll(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    lcd->display_mode |= LCD_ENTRYSHIFTINCREMENT;
    _lcd_command_internal(lcd, LCD_ENTRYMODESET | lcd->display_mode);
}

void lqi2c_noAutoscroll(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    lcd->display_mode &= ~LCD_ENTRYSHIFTINCREMENT;
    _lcd_command_internal(lcd, LCD_ENTRYMODESET | lcd->display_mode);
}

void lqi2c_backlight(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    lcd->backlight_state = LCD_BACKLIGHT;
    _i2c_send_byte_internal(lcd, 0x00);
}

void lqi2c_noBacklight(lqi2c_t *lcd) {
    if (lcd->i2c_dev_handle == NULL) return;
    lcd->backlight_state = LCD_NOBACKLIGHT;
    _i2c_send_byte_internal(lcd, 0x00);
}