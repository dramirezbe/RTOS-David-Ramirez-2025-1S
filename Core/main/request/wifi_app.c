/**
 *@file wifi_app.c
*/
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "lwip/netdb.h"

#include "http_server.h"
#include "tasks_common.h"
#include "wifi_app.h"
#include "freertos/queue.h"

// Tag usado para los mensajes de la consola serie de ESP
static const char TAG[] = "wifi_app";

// Handle de la cola para manipular la cola principal de eventos
static QueueHandle_t wifi_app_queue_handle;

// Objetos netif para la estación y el punto de acceso
esp_netif_t* esp_netif_sta = NULL;
esp_netif_t* esp_netif_ap  = NULL;

/**
 * @brief Manejador de eventos de la aplicación WiFi
 * @param arg datos, aparte de los datos del evento, que se pasan al manejador cuando se le llama
 * @param event_base el ID base del evento para el que se registra el manejador
 * @param event_id el ID del evento para el que se registra el manejador
 * @param event_data datos del evento
 */
static void wifi_app_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "WIFI_EVENT_AP_START");
                break;

            case WIFI_EVENT_AP_STOP:
                ESP_LOGI(TAG, "WIFI_EVENT_AP_STOP");
                break;

            case WIFI_EVENT_AP_STACONNECTED:
                ESP_LOGI(TAG, "WIFI_EVENT_AP_STACONNECTED");
                break;

            case WIFI_EVENT_AP_STADISCONNECTED:
                ESP_LOGI(TAG, "WIFI_EVENT_AP_STADISCONNECTED");
                break;

            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
                break;

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED");
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");
                // **AÑADIDO: Lógica de reconexión automática**
                ESP_LOGI(TAG, "Reconnecting to the AP...");
                esp_wifi_connect();
                break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        switch (event_id)
        {
            case IP_EVENT_STA_GOT_IP:
                ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP");
                // **AÑADIDO: Notificar a la tarea principal que la conexión fue exitosa**
                wifi_app_send_message(WIFI_APP_MSG_STA_CONNECTED_GOT_IP);
                break;
        }
    }
}

/**
 * @brief Inicializa el manejador de eventos de la aplicación WiFi para eventos WiFi e IP.
 */
static void wifi_app_event_handler_init(void)
{
    // Bucle de eventos para el driver WiFi
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Instancias del manejador de eventos para la conexión
    esp_event_handler_instance_t instance_wifi_event;
    esp_event_handler_instance_t instance_ip_event;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_app_event_handler, NULL, &instance_wifi_event));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_app_event_handler, NULL, &instance_ip_event));
}

/**
 * @brief Inicializa la pila TCP y la configuración WiFi por defecto.
 */
static void wifi_app_default_wifi_init(void)
{
    // Inicializa la pila TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());

    // Configuración WiFi por defecto - ¡las operaciones deben estar en este orden!
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    esp_netif_sta = esp_netif_create_default_wifi_sta();
    esp_netif_ap = esp_netif_create_default_wifi_ap();
}

/**
 * @brief Configura los ajustes del punto de acceso WiFi y asigna la IP estática al SoftAP.
 */
static void wifi_app_soft_ap_config(void)
{
    // SoftAP - Configuración del punto de acceso WiFi
    wifi_config_t ap_config =
    {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .password = WIFI_AP_PASSWORD,
            .channel = WIFI_AP_CHANNEL,
            .ssid_hidden = WIFI_AP_SSID_HIDDEN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = WIFI_AP_MAX_CONNECTIONS,
            .beacon_interval = WIFI_AP_BEACON_INTERVAL,
        },
    };

    // Configura el DHCP para el AP
    esp_netif_ip_info_t ap_ip_info;
    memset(&ap_ip_info, 0x00, sizeof(ap_ip_info));

    esp_netif_dhcps_stop(esp_netif_ap);                      ///> Se debe llamar a esto primero
    inet_pton(AF_INET, WIFI_AP_IP, &ap_ip_info.ip);         ///> Asigna la IP estática, GW y máscara de red del AP
    inet_pton(AF_INET, WIFI_AP_GATEWAY, &ap_ip_info.gw);
    inet_pton(AF_INET, WIFI_AP_NETMASK, &ap_ip_info.netmask);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(esp_netif_ap, &ap_ip_info)); ///> Configura estáticamente la interfaz de red
    ESP_ERROR_CHECK(esp_netif_dhcps_start(esp_netif_ap));             ///> Inicia el servidor DHCP del AP

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));              ///> Establece el modo como Punto de Acceso / Modo Estación
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config)); ///> Establece nuestra configuración
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_AP_BANDWIDTH)); ///> Ancho de banda por defecto 20 MHz
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_STA_POWER_SAVE));            ///> Ahorro de energía en "NONE"
}

/**
 * @brief Configura los ajustes del modo estación (STA).
 */
static void wifi_app_sta_config(void)
{
    wifi_config_t sta_config =
    {
        .sta = {
            .ssid = WIFI_STA_SSID,
            .password = WIFI_STA_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
}

/**
 * @brief Tarea principal para la aplicación WiFi
 * @param pvParameters parámetro que se puede pasar a la tarea
 */
static void wifi_app_task(void *pvParameters)
{
    wifi_app_queue_message_t msg;

    // Inicializa el manejador de eventos
    wifi_app_event_handler_init();

    // Inicializa la pila TCP/IP y la configuración WiFi
    wifi_app_default_wifi_init();

    // Configuración del SoftAP
    wifi_app_soft_ap_config();

    // Configuración del modo Estación
    wifi_app_sta_config();

    // Inicia el WiFi
    ESP_ERROR_CHECK(esp_wifi_start());

    // **AÑADIDO: Inicia el proceso de conexión al AP en modo estación**
    ESP_ERROR_CHECK(esp_wifi_connect());

    // Envía el primer mensaje de evento para iniciar el servidor
    wifi_app_send_message(WIFI_APP_MSG_START_HTTP_SERVER);

    for (;;)
    {
        if (xQueueReceive(wifi_app_queue_handle, &msg, (TickType_t)pdMS_TO_TICKS(10)))
        {
            switch (msg.msgID)
            {
                case WIFI_APP_MSG_START_HTTP_SERVER:
                    ESP_LOGI(TAG, "WIFI_APP_MSG_START_HTTP_SERVER");
                    http_server_start();
                    
                    break;

                case WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER:
                    ESP_LOGI(TAG, "WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER");
                    // Aquí podrías añadir lógica si recibes credenciales desde el servidor web
                    break;

                case WIFI_APP_MSG_STA_CONNECTED_GOT_IP:
                    ESP_LOGI(TAG, "WIFI_APP_MSG_STA_CONNECTED_GOT_IP");
                    //rgb_led_wifi_connected(); // Descomentado para indicar conexión exitosa
                    break;

                default:
                    break;
            }
        }
    }
}

BaseType_t wifi_app_send_message(wifi_app_message_e msgID)
{
    wifi_app_queue_message_t msg;
    msg.msgID = msgID;
    return xQueueSend(wifi_app_queue_handle, &msg, (TickType_t)pdMS_TO_TICKS(10));
}

void wifi_app_start(void)
{
    ESP_LOGI(TAG, "STARTING WIFI APPLICATION");

    esp_log_level_set("wifi", ESP_LOG_NONE);

    // Crea la cola de mensajes
    wifi_app_queue_handle = xQueueCreate(3, sizeof(wifi_app_queue_message_t));

    // Inicia la tarea de la aplicación WiFi
    xTaskCreatePinnedToCore(&wifi_app_task, "wifi_app_task", WIFI_APP_TASK_STACK_SIZE, NULL, WIFI_APP_TASK_PRIORITY, NULL, WIFI_APP_TASK_CORE_ID);
}