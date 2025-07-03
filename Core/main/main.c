/**
 * @file main.c
 * @author David Ramírez Betancourth
 *         Santiago Bustamante Montoya
 * @brief Take date and time from the internet with http protocol
 * @details Take date and time from the internet with http protocol and print it in the console
 *
 * Uses FreeRTOS tasks for NTC, LED, and potentiometer, and UART for command input.
 * Uses cJSON to parse the JSON response from the server.
 * Uses esp_http_client to make the HTTP request.
 * Uses esp_wifi to connect to the Wi-Fi network.
 * Uses esp_netif to get the IP address of the device.
 * Uses esp_log to log the messages.
 * Uses nvs_flash to initialize the NVS.
 */

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_http_client.h>
#include <cJSON.h>
#include <esp_tls.h>
#include <esp_netif.h>
#include <lwip/err.h>
#include <lwip/sys.h>

static const char *TAG = "WIFI_STATION";
static const char *HTTP_TAG = "HTTP_CLIENT";

// Variable para indicar si ya tenemos IP
static bool s_wifi_connected = false;

// Handler para eventos de Wi-Fi
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "🔄 WIFI_EVENT_STA_START - Iniciando conexión...");
        esp_wifi_connect();
        ESP_LOGI(TAG, "Conectando a la red...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disconnected = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGW(TAG, "🔌 WIFI_EVENT_STA_DISCONNECTED - Razón: %d", disconnected->reason);
        s_wifi_connected = false;
        esp_wifi_connect(); // Intentar reconectar
        ESP_LOGI(TAG, "Desconectado. Intentando reconectar...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "✅ IP_EVENT_STA_GOT_IP - IP obtenida: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "📡 Máscara de red: " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "🌐 Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
        s_wifi_connected = true; // Wi-Fi conectado y tenemos IP
    }
}

// Handler para eventos del cliente HTTP - MÁS VERBOSE
esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    static char *output_buffer;  // Buffer para almacenar la respuesta
    static int output_len;       // Stores number of bytes read
    
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGE(HTTP_TAG, "❌ HTTP_EVENT_ERROR: %s", esp_err_to_name((esp_err_t)evt->data));
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(HTTP_TAG, "🔗 HTTP_EVENT_ON_CONNECTED - Conectado al servidor");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(HTTP_TAG, "📤 HTTP_EVENT_HEADER_SENT - Headers enviados");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(HTTP_TAG, "📥 HTTP_EVENT_ON_HEADER - %s: %s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(HTTP_TAG, "📊 HTTP_EVENT_ON_DATA - Recibidos %d bytes", evt->data_len);
            ESP_LOGD(HTTP_TAG, "Datos recibidos: %.*s", evt->data_len, (char*)evt->data);
            
            // Si no hay buffer de salida, crear uno
            if (output_buffer == NULL) {
                output_buffer = (char *) malloc(evt->data_len + 1);
                output_len = 0;
                if (output_buffer == NULL) {
                    ESP_LOGE(HTTP_TAG, "❌ Fallo al asignar memoria para buffer de salida");
                    return ESP_FAIL;
                }
            } else {
                // Redimensionar el buffer
                output_buffer = (char *) realloc(output_buffer, output_len + evt->data_len + 1);
                if (output_buffer == NULL) {
                    ESP_LOGE(HTTP_TAG, "❌ Fallo al redimensionar buffer de salida");
                    return ESP_FAIL;
                }
            }
            
            // Copiar los datos al buffer
            memcpy(output_buffer + output_len, evt->data, evt->data_len);
            output_len += evt->data_len;
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(HTTP_TAG, "✅ HTTP_EVENT_ON_FINISH - Petición completada");
            if (output_buffer != NULL) {
                output_buffer[output_len] = '\0'; // Terminar string
                ESP_LOGI(HTTP_TAG, "📝 Respuesta completa (%d bytes):\n%s", output_len, output_buffer);
                
                // Parsear JSON aquí
                cJSON *root = cJSON_Parse(output_buffer);
                if (root) {
                    ESP_LOGI(HTTP_TAG, "✅ JSON parseado correctamente");
                    cJSON *datetime = cJSON_GetObjectItemCaseSensitive(root, "datetime");
                    if (cJSON_IsString(datetime) && (datetime->valuestring != NULL)) {
                        ESP_LOGI(HTTP_TAG, "🕐 Fecha y Hora Actual: %s", datetime->valuestring);
                    } else {
                        ESP_LOGE(HTTP_TAG, "❌ No se pudo encontrar 'datetime' en la respuesta JSON");
                    }
                    
                    // Mostrar más campos si están disponibles
                    cJSON *timezone = cJSON_GetObjectItemCaseSensitive(root, "timezone");
                    if (cJSON_IsString(timezone)) {
                        ESP_LOGI(HTTP_TAG, "🌍 Zona horaria: %s", timezone->valuestring);
                    }
                    
                    cJSON *utc_offset = cJSON_GetObjectItemCaseSensitive(root, "utc_offset");
                    if (cJSON_IsString(utc_offset)) {
                        ESP_LOGI(HTTP_TAG, "⏰ Offset UTC: %s", utc_offset->valuestring);
                    }
                    
                    cJSON_Delete(root);
                } else {
                    ESP_LOGE(HTTP_TAG, "❌ Error al parsear JSON: %s", cJSON_GetErrorPtr());
                }
                
                free(output_buffer);
                output_buffer = NULL;
                output_len = 0;
            }
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGW(HTTP_TAG, "🔌 HTTP_EVENT_DISCONNECTED - Desconectado del servidor");
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
                output_len = 0;
            }
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGI(HTTP_TAG, "🔄 HTTP_EVENT_REDIRECT - Redirigiendo");
            break;
    }
    return ESP_OK;
}

// Función para hacer la petición HTTP y obtener la fecha - MÁS VERBOSE
void fetch_and_print_date(void) {
    ESP_LOGI(HTTP_TAG, "🚀 Iniciando petición HTTP...");
    
    // Verificar estado de la conexión
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL) {
        ESP_LOGE(HTTP_TAG, "❌ No se pudo obtener la interfaz de red");
        return;
    }
    
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        ESP_LOGI(HTTP_TAG, "📡 IP actual: " IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI(HTTP_TAG, "🌐 Gateway: " IPSTR, IP2STR(&ip_info.gw));
    } else {
        ESP_LOGE(HTTP_TAG, "❌ No se pudo obtener información de IP");
    }
    
    // Configuración HTTP más detallada
    esp_http_client_config_t config = {
        .url = "http://worldtimeapi.org/api/timezone/America/Bogota",
        .event_handler = _http_event_handler,
        .user_data = NULL,
        .disable_auto_redirect = false,
        .skip_cert_common_name_check = true,
        .use_global_ca_store = false,
        .timeout_ms = 10000,  // 10 segundos de timeout
        .buffer_size = 512,   // Buffer size para HTTP
        .buffer_size_tx = 1024, // Buffer size para transmisión
    };
    
    ESP_LOGI(HTTP_TAG, "🔧 Configuración del cliente HTTP:");
    ESP_LOGI(HTTP_TAG, "   URL: %s", config.url);
    ESP_LOGI(HTTP_TAG, "   Timeout: %d ms", config.timeout_ms);
    ESP_LOGI(HTTP_TAG, "   Buffer size: %d bytes", config.buffer_size);
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(HTTP_TAG, "❌ Fallo al inicializar cliente HTTP");
        return;
    }
    
    ESP_LOGI(HTTP_TAG, "✅ Cliente HTTP inicializado correctamente");
    ESP_LOGI(HTTP_TAG, "📤 Haciendo petición GET a %s", config.url);
    
    esp_err_t err = esp_http_client_perform(client);
    
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        int64_t content_length = esp_http_client_get_content_length(client);
        
        ESP_LOGI(HTTP_TAG, "✅ Petición HTTP completada exitosamente");
        ESP_LOGI(HTTP_TAG, "📊 Código de estado HTTP: %d", status_code);
        ESP_LOGI(HTTP_TAG, "📏 Content-Length: %lld bytes", content_length);
        
        if (status_code >= 200 && status_code < 300) {
            ESP_LOGI(HTTP_TAG, "✅ Respuesta HTTP exitosa");
        } else if (status_code >= 300 && status_code < 400) {
            ESP_LOGW(HTTP_TAG, "⚠️ Redirección HTTP: %d", status_code);
        } else if (status_code >= 400 && status_code < 500) {
            ESP_LOGE(HTTP_TAG, "❌ Error del cliente HTTP: %d", status_code);
        } else if (status_code >= 500) {
            ESP_LOGE(HTTP_TAG, "❌ Error del servidor HTTP: %d", status_code);
        }
        
    } else {
        ESP_LOGE(HTTP_TAG, "❌ Error al realizar la petición HTTP GET: %s (0x%x)", 
                 esp_err_to_name(err), err);
        
        // Detalles adicionales del error
        switch (err) {
            case ESP_ERR_INVALID_ARG:
                ESP_LOGE(HTTP_TAG, "   Argumento inválido");
                break;
            case ESP_ERR_INVALID_STATE:
                ESP_LOGE(HTTP_TAG, "   Estado inválido");
                break;
            case ESP_ERR_NO_MEM:
                ESP_LOGE(HTTP_TAG, "   Sin memoria disponible");
                break;
            case ESP_ERR_TIMEOUT:
                ESP_LOGE(HTTP_TAG, "   Timeout de conexión");
                break;
            case ESP_FAIL:
                ESP_LOGE(HTTP_TAG, "   Fallo general");
                break;
            default:
                ESP_LOGE(HTTP_TAG, "   Error desconocido: 0x%x", err);
                break;
        }
    }
    
    ESP_LOGI(HTTP_TAG, "🧹 Limpiando cliente HTTP");
    esp_http_client_cleanup(client);
    ESP_LOGI(HTTP_TAG, "✅ Cliente HTTP limpiado");
}

void app_main(void) {
    ESP_LOGI(TAG, "🚀 Iniciando aplicación ESP32 HTTP Client");
    
    // Configurar niveles de log para máximo detalle
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("HTTP_CLIENT", ESP_LOG_DEBUG);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_DEBUG);
    esp_log_level_set("esp-tls", ESP_LOG_DEBUG);
    esp_log_level_set("WIFI_STATION", ESP_LOG_DEBUG);
    
    ESP_LOGI(TAG, "🔧 Configuración de logs establecida");
    
    // 1. Inicializar NVS (Non-Volatile Storage)
    ESP_LOGI(TAG, "📂 Inicializando NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "⚠️ NVS necesita ser borrado, reinicializando...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✅ NVS inicializado correctamente");

    // 2. Inicializar TCP/IP Stack
    ESP_LOGI(TAG, "🌐 Inicializando stack TCP/IP...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_LOGI(TAG, "✅ Stack TCP/IP inicializado");

    // 3. Crear el loop de eventos por defecto
    ESP_LOGI(TAG, "🔄 Creando loop de eventos...");
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI(TAG, "✅ Loop de eventos creado");

    // 4. Crear una instancia de netif para Wi-Fi estación
    ESP_LOGI(TAG, "📡 Creando interfaz de red Wi-Fi...");
    esp_netif_create_default_wifi_sta();
    ESP_LOGI(TAG, "✅ Interfaz de red Wi-Fi creada");

    // 5. Inicializar Wi-Fi
    ESP_LOGI(TAG, "📶 Inicializando Wi-Fi...");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_LOGI(TAG, "✅ Wi-Fi inicializado");

    // 6. Registrar el handler de eventos
    ESP_LOGI(TAG, "📋 Registrando handlers de eventos...");
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_LOGI(TAG, "✅ Handlers de eventos registrados");

    // 7. Configurar Wi-Fi en modo estación
    ESP_LOGI(TAG, "⚙️ Configurando Wi-Fi en modo estación...");
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Javastral",
            .password = "damedane",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    
    ESP_LOGI(TAG, "📋 Configuración Wi-Fi:");
    ESP_LOGI(TAG, "   SSID: %s", (char*)wifi_config.sta.ssid);
    ESP_LOGI(TAG, "   Modo de autenticación: WPA2_PSK");
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_LOGI(TAG, "✅ Wi-Fi configurado en modo estación");

    // 8. Iniciar Wi-Fi
    ESP_LOGI(TAG, "🚀 Iniciando Wi-Fi...");
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "✅ Wi-Fi en modo estación iniciado");

    // Esperar a que el Wi-Fi esté conectado antes de intentar hacer la petición HTTP
    ESP_LOGI(TAG, "⏳ Esperando conexión Wi-Fi...");
    int retry_count = 0;
    while (!s_wifi_connected && retry_count < 20) {
        ESP_LOGI(TAG, "🔄 Esperando conexión Wi-Fi... (%d/20)", retry_count + 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry_count++;
    }

    if (s_wifi_connected) {
        ESP_LOGI(TAG, "✅ Wi-Fi conectado exitosamente. Iniciando petición HTTP...");
        fetch_and_print_date();
    } else {
        ESP_LOGE(TAG, "❌ No se pudo conectar al Wi-Fi después de 20 intentos. No se realizará la petición HTTP.");
    }

    // La aplicación principal continuará ejecutándose aquí
    ESP_LOGI(TAG, "🔄 Entrando en loop principal...");
    while (true) {
        ESP_LOGI(TAG, "💤 Esperando 1 segundos antes de la próxima petición...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        if (s_wifi_connected) {
            ESP_LOGI(TAG, "🔄 Realizando petición HTTP periódica...");
            fetch_and_print_date();
        } else {
            ESP_LOGW(TAG, "⚠️ Wi-Fi desconectado, omitiendo petición HTTP");
        }
    }
}