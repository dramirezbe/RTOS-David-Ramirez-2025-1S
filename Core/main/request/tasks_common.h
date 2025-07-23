/*
 * tasks_common.h
 *
 *  Created on: Oct 17, 2021
 *      Author: kjagu
 */

#ifndef MAIN_TASKS_COMMON_H_
#define MAIN_TASKS_COMMON_H_

// WiFi application task
#define WIFI_APP_TASK_STACK_SIZE			4096
#define WIFI_APP_TASK_PRIORITY				5
#define WIFI_APP_TASK_CORE_ID				0

// HTTP Server task
#define HTTP_SERVER_TASK_STACK_SIZE			8192
#define HTTP_SERVER_TASK_PRIORITY			4
#define HTTP_SERVER_TASK_CORE_ID			0

// HTTP Server Monitor task
#define HTTP_SERVER_MONITOR_STACK_SIZE		4096
#define HTTP_SERVER_MONITOR_PRIORITY		3
#define HTTP_SERVER_MONITOR_CORE_ID			0

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// -- Tipos de datos compartidos --

// Enum para el modo de control del servo
typedef enum {
    CTL_POT,
    CTL_THRES,
    CTL_REG,
    CTL_IDLE
} servo_ctl_mode_state_t;

// Enum para el estado del día según la luz
typedef enum {
    SUNNY,
    CLOUDY,
    AFTERNOON,
    NIGHT
} day_state_t;

// Estructura para los datos de la fotorresistencia enviados a la web
typedef struct {
    day_state_t day_state;
    float current_photores;
} http_photores_data_t;


// -- Colas declaradas externamente --
// Estas colas son creadas en main.c y utilizadas por http_server.c

// Colas para enviar datos desde ctl_task hacia los handlers HTTP (GET)
extern QueueHandle_t http_send_servo_pwm_queue; // Para el porcentaje de apertura del potenciómetro
extern QueueHandle_t http_send_ntc_queue;       // Para la temperatura del NTC
extern QueueHandle_t http_send_photores_queue;  // Para los datos del LDR

// Colas para recibir datos desde los handlers HTTP (POST) hacia ctl_task
extern QueueHandle_t http_receive_servo_ctl_mode;    // Para recibir el nuevo modo de control
extern QueueHandle_t http_receive_thres_values_queue; // Para recibir los umbrales de temperatura

#endif /* MAIN_TASKS_COMMON_H_ */
