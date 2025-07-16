/**
 * @file http_server.c
 */

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "sys/param.h"
#include "driver/gpio.h"

#include "http_server.h"
#include "tasks_common.h"
#include "wifi_app.h"
//#include "rgb_led.h"
//#include "freertos/queue.h"

#include <cJSON.h>

// Tag used for ESP serial console messages
static const char TAG[] = "http_server";

// HTTP server task handle
static httpd_handle_t http_server_handle = NULL;

// HTTP server monitor task handle
static TaskHandle_t task_http_server_monitor = NULL;

// Queue handle used to manipulate the main queue of events
static QueueHandle_t http_server_monitor_queue_handle;

static uint8_t s_led_state = 0;




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

static float get_int_from_json(cJSON *json_obj, const char *key) {
    cJSON *item = cJSON_GetObjectItemCaseSensitive(json_obj, key);
    if (cJSON_IsNumber(item)) {
        return (int)cJSON_GetNumberValue(item);
    } else {
        ESP_LOGW(TAG, "%s value not found or not a number, defaulting to 0", key);
        return 0;
    }
}

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
 * NTC sensor readings JSON handler responds with NTC sensor data
 * @param req HTTP request for which the uri needs to be handled
 * @return ESP_OK
 */

extern double temperature;
static esp_err_t http_server_get_ntc_sensor_readings_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "/ntcSensor.json requested");

	cJSON *root;
    char *json_string = NULL;
    double humidity = 40.5;

    root = cJSON_CreateObject();

    if (cJSON_AddNumberToObject(root, "temp", temperature) == NULL) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }
    if (cJSON_AddNumberToObject(root, "humidity", humidity) == NULL) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    json_string = cJSON_Print(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    cJSON_Delete(root);
    cJSON_free(json_string);

    return ESP_OK;
}

static esp_err_t http_server_toogle_led_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "/toogle_led.json requested");

	s_led_state = !s_led_state;
	gpio_set_level(BLINK_GPIO, s_led_state);

	// Cerrar la conexion
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, NULL, 0);
    
	return ESP_OK;
}


extern bool uart_on;
static esp_err_t http_server_toogle_uart_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/toogle_uart.json requested");

    char *buf = NULL; 
   
    if (receive_http_content(req, &buf) != ESP_OK) {
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(buf);
    free(buf);

    if (root == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            ESP_LOGE(TAG, "Error parsing JSON before: %s", error_ptr);
        }
        ESP_LOGE(TAG, "Failed to parse JSON data");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON *uart_status = cJSON_GetObjectItemCaseSensitive(root, "uart_on");

    if (cJSON_IsBool(uart_status)) {
        if(cJSON_IsTrue(uart_status)) {
            uart_on = true;
            printf("UART ON parsed\n");
        } else {
            uart_on = false;
            printf("UART OFF parsed\n");
        }
    }

    cJSON_Delete(root);

    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, NULL, 0);

    return ESP_OK;
}
extern float temp_levels[3][2];
static esp_err_t http_server_temp_threshold_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "/temp_threshold.json requested (POST)");
	
    char *buf = NULL; 
   
    if (receive_http_content(req, &buf) != ESP_OK) {
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(buf);
    free(buf);

    float red_min = get_float_from_json(root, "red_min");
    float red_max = get_float_from_json(root, "red_max");
    float green_min = get_float_from_json(root, "green_min");
    float green_max = get_float_from_json(root, "green_max");
    float blue_min = get_float_from_json(root, "blue_min");
    float blue_max = get_float_from_json(root, "blue_max");

    printf("Values parsed: RedMin=%.2f, RedMax=%.2f, GreenMin=%.2f, GreenMax=%.2f, BlueMin=%.2f, BlueMax=%.2f",
             red_min, red_max, green_min, green_max, blue_min, blue_max);

	

	temp_levels[0][0] = red_min;
	temp_levels[0][1] = red_max;
	temp_levels[1][0] = green_min;
	temp_levels[1][1] = green_max;
	temp_levels[2][0] = blue_min;
	temp_levels[2][1] = blue_max;

    cJSON_Delete(root);

    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, NULL, 0);

    return ESP_OK;
}

extern QueueHandle_t rgb_event_queue;
static esp_err_t http_server_rgb_values_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/rgb_values.json requested (POST)");

	char *buf = NULL; 
   
    if (receive_http_content(req, &buf) != ESP_OK) {
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(buf);
    free(buf);

    int red_val = get_int_from_json(root, "red_val");
    int green_val = get_int_from_json(root, "green_val");
    int blue_val = get_int_from_json(root, "blue_val");

    printf("Values parsed: Red=%d, Green=%d, Blue=%d\n", red_val, green_val, blue_val);

	rgb_values_t rgb_values = {.red_val = red_val, .green_val = green_val, .blue_val = blue_val};

    cJSON_Delete(root);

    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_send(req, NULL, 0);

	xQueueSend(rgb_event_queue, &rgb_values, (TickType_t)pdMS_TO_TICKS(10));

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

		// register ntcSensor.json handler
		httpd_uri_t ntc_sensor_json = {
				.uri = "/ntcSensor.json",
				.method = HTTP_GET,
				.handler = http_server_get_ntc_sensor_readings_json_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &ntc_sensor_json);
		
		// register toogle_led handler
		httpd_uri_t toogle_led = {
				.uri = "/toogle_led.json",
				.method = HTTP_POST,
				.handler = http_server_toogle_led_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &toogle_led);
		
		//turn off UART
		
		httpd_uri_t toogle_uart = {
				.uri = "/toogle_uart.json",
				.method = HTTP_POST,
				.handler = http_server_toogle_uart_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &toogle_uart);

		// register rgb_receiver handler
		httpd_uri_t rgb_values = {
				.uri = "/rgb_values.json",
				.method = HTTP_POST,
				.handler = http_server_rgb_values_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &rgb_values);

		// register temp_threshold handler
		httpd_uri_t temp_threshold = {
				.uri = "/temp_threshold.json",
				.method = HTTP_POST,
				.handler = http_server_temp_threshold_handler,
				.user_ctx = NULL
		};
		httpd_register_uri_handler(http_server_handle, &temp_threshold);

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