/**
 * @file http_server.c
 */

#include "esp_http_server.h"
#include "esp_log.h"
#include "sys/param.h"

#include "http_server.h"
#include "tasks_common.h"
#include "wifi_app.h"

#include <cJSON.h>
#include <string.h>

// Tag used for ESP serial console messages
static const char TAG[] = "http_server";

// HTTP server task handle
static httpd_handle_t http_server_handle = NULL;

// HTTP server monitor task handle
static TaskHandle_t task_http_server_monitor = NULL;

// Queue handle used to manipulate the main queue of events
static QueueHandle_t http_server_monitor_queue_handle;

// Embedded files: JQuery, index.html, app.css, app.js and favicon.ico files
extern const uint8_t jquery_3_3_1_min_js_start[]	asm("_binary_jquery_3_3_1_min_js_start");
extern const uint8_t jquery_3_3_1_min_js_end[]		asm("_binary_jquery_3_3_1_min_js_end");
extern const uint8_t index_html_start[]				asm("_binary_index_html_start");
extern const uint8_t index_html_end[]				asm("_binary_index_html_end");
extern const uint8_t app_css_start[]				asm("_binary_app_css_start");
extern const uint8_t app_css_end[]					asm("_binary_app_css_end");
extern const uint8_t app_js_start[]					asm("_binary_app_js_start");
extern const uint8_t app_js_end[]					asm("_binary_app_js_end");
extern const uint8_t favicon_ico_start[]			asm("_binary_favicon_ico_start");
extern const uint8_t favicon_ico_end[]				asm("_binary_favicon_ico_end");

//-----------------------------------UTILS----------------------------
/**
 * @brief Receives HTTP request content, handling content length, memory allocation, and reception.
 *
 * @param req Pointer to the httpd_req_t structure.
 * @param buf_out Pointer to a char* where the allocated buffer containing the received content will be stored.
 * The caller is responsible for freeing this buffer.
 * @return ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t receive_http_content(httpd_req_t *req, char **buf_out) {
    int content_len = req->content_len;

    // 1. Check for valid content length
    if (content_len <= 0) {
        ESP_LOGE(TAG, "Empty or invalid content length received.");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // 2. Allocate memory for the request content
    char* buf = (char*)malloc(content_len + 1);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate memory for request content");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // 3. Receive the content
    int received = 0;
    int ret;
    while (received < content_len) {
        ret = httpd_req_recv(req, buf + received, content_len - received);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                // Continue if it's a timeout, try receiving again
                continue;
            }
            ESP_LOGE(TAG, "Failed to receive request content: %d", ret);
            free(buf); // Free allocated memory on error
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        received += ret;
    }
    buf[received] = '\0'; // Null-terminate the received data

    printf("Received JSON data: %s\n", buf); // Print for debugging

    *buf_out = buf; // Pass the allocated buffer back to the caller
    return ESP_OK;
}

// Helper function to safely extract a float value from a cJSON object
static float get_float_from_json(cJSON *json_obj, const char *key) {
    cJSON *item = cJSON_GetObjectItemCaseSensitive(json_obj, key);
    if (cJSON_IsNumber(item)) {
        return (float)cJSON_GetNumberValue(item);
    } else {
        ESP_LOGW(TAG, "%s value not found or not a number, defaulting to 0", key);
        return 0.0f; // Default value if not found or not a number
    }
}

// static float get_int_from_json(cJSON *json_obj, const char *key) {
//     cJSON *item = cJSON_GetObjectItemCaseSensitive(json_obj, key);
//     if (cJSON_IsNumber(item)) {
//         return (int)cJSON_GetNumberValue(item);
//     } else {
//         ESP_LOGW(TAG, "%s value not found or not a number, defaulting to 0", key);
//         return 0;
//     }
// }

//---------------------------------HTTP------------------------------------

/**
 * HTTP server monitor task used to track events of the HTTP server
 * @param pvParameters parameter which can be passed to the task.
 */
static void http_server_monitor(void *parameter)
{
	http_server_queue_message_t msg;

	for (;;)
	{
		if (xQueueReceive(http_server_monitor_queue_handle, &msg, (TickType_t)pdMS_TO_TICKS(10)))
		{
			switch (msg.msgID)
			{
				case HTTP_MSG_WIFI_CONNECT_INIT:
					ESP_LOGI(TAG, "HTTP_MSG_WIFI_CONNECT_INIT");

					break;

				case HTTP_MSG_WIFI_CONNECT_SUCCESS:
					ESP_LOGI(TAG, "HTTP_MSG_WIFI_CONNECT_SUCCESS");

					break;

				case HTTP_MSG_WIFI_CONNECT_FAIL:
					ESP_LOGI(TAG, "HTTP_MSG_WIFI_CONNECT_FAIL");

					break;

				case HTTP_MSG_OTA_UPDATE_SUCCESSFUL:
					ESP_LOGI(TAG, "HTTP_MSG_OTA_UPDATE_SUCCESSFUL");
					//g_fw_update_status = OTA_UPDATE_SUCCESSFUL;
					//http_server_fw_update_reset_timer();

					break;

				case HTTP_MSG_OTA_UPDATE_FAILED:
					ESP_LOGI(TAG, "HTTP_MSG_OTA_UPDATE_FAILED");
					//g_fw_update_status = OTA_UPDATE_FAILED;

					break;

				default:
					break;
			}
		}
	}
}

/**
 * Jquery get handler is requested when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_jquery_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "Jquery requested");

	httpd_resp_set_type(req, "application/javascript");
	httpd_resp_send(req, (const char *)jquery_3_3_1_min_js_start, jquery_3_3_1_min_js_end - jquery_3_3_1_min_js_start);

	return ESP_OK;
}

/**
 * Sends the index.html page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_index_html_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "index.html requested");

	httpd_resp_set_type(req, "text/html");
	httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);

	return ESP_OK;
}

/**
 * app.css get handler is requested when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_app_css_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "app.css requested");

	httpd_resp_set_type(req, "text/css");
	httpd_resp_send(req, (const char *)app_css_start, app_css_end - app_css_start);

	return ESP_OK;
}

/**
 * app.js get handler is requested when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_app_js_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "app.js requested");

	httpd_resp_set_type(req, "application/javascript");
	httpd_resp_send(req, (const char *)app_js_start, app_js_end - app_js_start);

	return ESP_OK;
}

/**
 * Sends the .ico (icon) file when accessing the web page.
 * @param req HTTP request for which the uri needs to be handled.
 * @return ESP_OK
 */
static esp_err_t http_server_favicon_ico_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "favicon.ico requested");

	httpd_resp_set_type(req, "image/x-icon");
	httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_end - favicon_ico_start);

	return ESP_OK;
}

/**
 * @brief Handler para /windowPercentage.json
 * Envía el porcentaje de apertura actual de la ventana (leído del potenciómetro).
 */
static esp_err_t http_server_get_window_percentage_handler(httpd_req_t *req)
{
    float pot_percentage = 0;
    // Espiamos la cola para obtener el último valor sin eliminarlo.
    // Usamos un timeout corto por si la cola está momentáneamente vacía.
    xQueuePeek(http_send_servo_pwm_queue, &pot_percentage, pdMS_TO_TICKS(10));

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "percentage", pot_percentage);
    const char *json_string = cJSON_PrintUnformatted(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    cJSON_Delete(root);
    free((void *)json_string);

    return ESP_OK;
}


/**
 * @brief Handler para /ambientTemp.json
 * Envía la temperatura ambiente actual (leída del NTC).
 */
static esp_err_t http_server_get_ambient_temp_handler(httpd_req_t *req)
{
    float ntc_temp = 0.0f;
    xQueuePeek(http_send_ntc_queue, &ntc_temp, pdMS_TO_TICKS(10));

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "temp", ntc_temp);
    const char *json_string = cJSON_PrintUnformatted(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    cJSON_Delete(root);
    free((void *)json_string);

    return ESP_OK;
}


/**
 * @brief Handler para /lightSensor.json
 * Envía la resistencia del LDR y el estado del día calculado.
 */
static esp_err_t http_server_get_light_sensor_handler(httpd_req_t *req)
{
    http_photores_data_t photores_data = {0};
    xQueuePeek(http_send_photores_queue, &photores_data, pdMS_TO_TICKS(10));

    // Convertir el enum day_state a una cadena de texto
    const char *day_state_str;
    switch (photores_data.day_state) {
        case SUNNY:     day_state_str = "SUNNY"; break;
        case CLOUDY:    day_state_str = "CLOUDY"; break;
        case AFTERNOON: day_state_str = "AFTERNOON"; break;
        case NIGHT:     day_state_str = "NIGHT"; break;
        default:        day_state_str = "UNKNOWN"; break;
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "resistance", photores_data.current_photores);
    cJSON_AddStringToObject(root, "dayState", day_state_str);
    const char *json_string = cJSON_PrintUnformatted(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    cJSON_Delete(root);
    free((void *)json_string);

    return ESP_OK;
}

/**
 * @brief Handler para /currentHour.json (Placeholder)
 * Envía una hora placeholder. A implementar con un cliente NTP.
 */
static esp_err_t http_server_get_current_hour_handler(httpd_req_t *req)
{
    // TODO: Obtener la hora real de un servicio NTP.
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "hour", "18:30:05");
    const char *json_string = cJSON_PrintUnformatted(root);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    cJSON_Delete(root);
    free((void *)json_string);

    return ESP_OK;
}


/**
 * @brief Handler para /setMode.json
 * Recibe el nuevo modo de control del servo y lo envía a ctl_task.
 */
static esp_err_t http_server_set_mode_handler(httpd_req_t *req)
{
    char *buf = NULL;
    if (receive_http_content(req, &buf) != ESP_OK) {
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON *mode_json = cJSON_GetObjectItemCaseSensitive(root, "mode");
    if (cJSON_IsString(mode_json) && (mode_json->valuestring != NULL)) {
        servo_ctl_mode_state_t mode_to_send = CTL_IDLE; // Default

        if (strcmp(mode_json->valuestring, "potentiometer") == 0) {
            mode_to_send = CTL_POT;
        } else if (strcmp(mode_json->valuestring, "temp_threshold") == 0) {
            mode_to_send = CTL_THRES;
        } else if (strcmp(mode_json->valuestring, "registers") == 0) {
            mode_to_send = CTL_REG;
        }
        
        // Enviar el nuevo modo a la tarea de control
        xQueueSend(http_receive_servo_ctl_mode, &mode_to_send, pdMS_TO_TICKS(10));
    }
    
    cJSON_Delete(root);
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/**
 * @brief Handler para /setTempFullyOpenRange.json
 * Recibe los nuevos umbrales de temperatura y los envía a ctl_task.
 */
static esp_err_t http_server_set_temp_range_handler(httpd_req_t *req)
{
    char *buf = NULL;
    if (receive_http_content(req, &buf) != ESP_OK) {
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    float thres_values[2] = {0.0f, 0.0f};
    thres_values[0] = get_float_from_json(root, "minTemp");
    thres_values[1] = get_float_from_json(root, "maxTemp");

    // Enviar el array con los dos umbrales a la tarea de control
    xQueueSend(http_receive_thres_values_queue, &thres_values, pdMS_TO_TICKS(10));

    cJSON_Delete(root);
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}



/**
 * Sets up the default httpd server configuration.
 * @return http server instance handle if successful, NULL otherwise.
 */
static httpd_handle_t http_server_configure(void)
{
	// Generate the default configuration
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();

	// Create the message queue
	http_server_monitor_queue_handle = xQueueCreate(3, sizeof(http_server_queue_message_t));

	// Create HTTP server monitor task
	xTaskCreatePinnedToCore(&http_server_monitor, "http_server_monitor", HTTP_SERVER_MONITOR_STACK_SIZE, NULL, HTTP_SERVER_MONITOR_PRIORITY, &task_http_server_monitor, HTTP_SERVER_MONITOR_CORE_ID);


	// The core that the HTTP server will run on
	config.core_id = HTTP_SERVER_TASK_CORE_ID;

	// Adjust the default priority to 1 less than the wifi application task
	config.task_priority = HTTP_SERVER_TASK_PRIORITY;

	// Bump up the stack size (default is 4096)
	config.stack_size = HTTP_SERVER_TASK_STACK_SIZE;

	// Increase uri handlers
	config.max_uri_handlers = 20;


	// Increase the timeout limits
	config.recv_wait_timeout = 10;
	config.send_wait_timeout = 10;

	ESP_LOGI(TAG,
			"http_server_configure: Starting server on port: '%d' with task priority: '%d'",
			config.server_port,
			config.task_priority);

	// Start the httpd server
	if (httpd_start(&http_server_handle, &config) == ESP_OK)
	{
		ESP_LOGI(TAG, "http_server_configure: Registering URI handlers");

		// register index.html handler
		httpd_uri_t index_html = {
				.uri = "/",
				.method = HTTP_GET,
				.handler = http_server_index_html_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &index_html);

		// register query handler
		httpd_uri_t jquery_js = {
				.uri = "/jquery-3.3.1.min.js",
				.method = HTTP_GET,
				.handler = http_server_jquery_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &jquery_js);


		// register app.css handler
		httpd_uri_t app_css = {
				.uri = "/app.css",
				.method = HTTP_GET,
				.handler = http_server_app_css_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &app_css);

		// register app.js handler
		httpd_uri_t app_js = {
				.uri = "/app.js",
				.method = HTTP_GET,
				.handler = http_server_app_js_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &app_js);

		// register favicon.ico handler
		httpd_uri_t favicon_ico = {
				.uri = "/favicon.ico",
				.method = HTTP_GET,
				.handler = http_server_favicon_ico_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &favicon_ico);

		httpd_uri_t window_percentage_uri = { .uri = "/windowPercentage.json", .method = HTTP_GET, .handler = http_server_get_window_percentage_handler };
        httpd_register_uri_handler(http_server_handle, &window_percentage_uri);

        httpd_uri_t ambient_temp_uri = { .uri = "/ambientTemp.json", .method = HTTP_GET, .handler = http_server_get_ambient_temp_handler };
        httpd_register_uri_handler(http_server_handle, &ambient_temp_uri);

        httpd_uri_t light_sensor_uri = { .uri = "/lightSensor.json", .method = HTTP_GET, .handler = http_server_get_light_sensor_handler };
        httpd_register_uri_handler(http_server_handle, &light_sensor_uri);

        httpd_uri_t current_hour_uri = { .uri = "/currentHour.json", .method = HTTP_GET, .handler = http_server_get_current_hour_handler };
        httpd_register_uri_handler(http_server_handle, &current_hour_uri);

        // --- Handlers para la API (POST) ---
        httpd_uri_t set_mode_uri = { .uri = "/setMode.json", .method = HTTP_POST, .handler = http_server_set_mode_handler };
        httpd_register_uri_handler(http_server_handle, &set_mode_uri);

        httpd_uri_t set_temp_range_uri = { .uri = "/setTempFullyOpenRange.json", .method = HTTP_POST, .handler = http_server_set_temp_range_handler };
        httpd_register_uri_handler(http_server_handle, &set_temp_range_uri);

	


		return http_server_handle;
	}

	return NULL;
}

void http_server_start(void)
{
	if (http_server_handle == NULL)
	{
		http_server_handle = http_server_configure();
	}
}

void http_server_stop(void)
{
	if (http_server_handle)
	{
		httpd_stop(http_server_handle);
		ESP_LOGI(TAG, "http_server_stop: stopping HTTP server");
		http_server_handle = NULL;
	}
	if (task_http_server_monitor)
	{
		vTaskDelete(task_http_server_monitor);
		ESP_LOGI(TAG, "http_server_stop: stopping HTTP server monitor");
		task_http_server_monitor = NULL;
	}
}

BaseType_t http_server_monitor_send_message(http_server_message_e msgID)
{
	http_server_queue_message_t msg;
	msg.msgID = msgID;
	return xQueueSend(http_server_monitor_queue_handle, &msg, (TickType_t)pdMS_TO_TICKS(10));
}

void http_server_fw_update_reset_callback(void *arg)
{
	ESP_LOGI(TAG, "http_server_fw_update_reset_callback: Timer timed-out, restarting the device");
	esp_restart();
}