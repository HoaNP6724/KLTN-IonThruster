/**
 * @file web_app_main.c
 * @brief He thong ESP32 Web Server dieu khien:
 *        1. Module xung XY-LPWM qua UART1
 *        2. Cam bien gio RS485 Modbus-RTU qua UART2
 *        3. Dieu khien ON/OFF Relay qua GPIO
 *
 * LUU Y: File nay duoc tao doc lap, KHONG chinh sua cac file main.c,
 *        lpwm_backup.c va wind_sensor_backup.c hien co.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "cJSON.h"

static const char *TAG = "WEB_APP_SYSTEM";

/* =========================================================
 * 1. CAU HINH PHAN CUNG
 * ========================================================= */
/* --- UART1: Module PWM XY-LPWM --- */
#define PWM_UART_PORT       UART_NUM_1
#define PWM_UART_TX_PIN     GPIO_NUM_18
#define PWM_UART_RX_PIN     GPIO_NUM_19
#define PWM_UART_BAUD       9600

/* --- UART2: Cam bien gio RS485 Modbus --- */
#define RS485_UART_PORT     UART_NUM_2
#define RS485_TX_PIN        GPIO_NUM_17    /* DI cua MAX485 */
#define RS485_RX_PIN        GPIO_NUM_16    /* RO cua MAX485 */
#define RS485_DE_RE_PIN     GPIO_NUM_4     /* DE + RE cua MAX485 */
#define RS485_BAUD_RATE     9600
#define MODBUS_SLAVE_ADDR   0x03
#define REG_WIND_SPEED      0x0000

/* --- GPIO Relay --- */
#define RELAY_GPIO_PIN      GPIO_NUM_23
#define RELAY_ACTIVE_LEVEL  1  /* 1: Muc HIGH dong relay, 0 neu la relay Active Low */

/* --- Cấu hình WiFi SoftAP --- */
#define WIFI_AP_SSID        "ESP32-KLTN-HUB"
#define WIFI_AP_PASS        "12345678"
#define WIFI_AP_MAX_CONN    4
#define WIFI_AP_CHANNEL     1

#define BUF_SIZE            256
#define RESP_TIMEOUT_MS     1500

/* =========================================================
 * 2. BIEN TRANG THAI HE THONG TOAN CUC
 * ========================================================= */
static float g_latest_wind_speed = 0.0f;
static int   g_latest_wind_level = 0;
static bool  g_relay_state = false;
static char  g_last_pwm_freq[16] = "F100";
static char  g_last_pwm_duty[16] = "D050";
static char  g_last_pwm_resp[128] = "OK (Ready)";

/* =========================================================
 * 3. KHOI TAO PHAN CUNG: RELAY & UART
 * ========================================================= */
static void relay_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_GPIO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(RELAY_GPIO_PIN, !RELAY_ACTIVE_LEVEL); // Ban dau tat relay
    g_relay_state = false;
    ESP_LOGI(TAG, "Relay initialized on GPIO%d (Active %s)", 
             RELAY_GPIO_PIN, RELAY_ACTIVE_LEVEL ? "HIGH" : "LOW");
}

static void set_relay_state(bool on) {
    g_relay_state = on;
    int level = on ? RELAY_ACTIVE_LEVEL : (!RELAY_ACTIVE_LEVEL);
    gpio_set_level(RELAY_GPIO_PIN, level);
    ESP_LOGI(TAG, "Relay turned %s (GPIO%d = %d)", on ? "ON" : "OFF", RELAY_GPIO_PIN, level);
}

/* Khoi tao UART1 cho XY-LPWM */
static void pwm_uart_init(void) {
    uart_config_t cfg = {
        .baud_rate = PWM_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(PWM_UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PWM_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(PWM_UART_PORT, PWM_UART_TX_PIN, PWM_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART1 (PWM) initialized: TX=%d, RX=%d", PWM_UART_TX_PIN, PWM_UART_RX_PIN);
}

/* Khoi tao UART2 cho RS485 Modbus */
static void rs485_init(void) {
    uart_config_t cfg = {
        .baud_rate = RS485_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RS485_UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART_PORT, RS485_TX_PIN, RS485_RX_PIN,
                                 RS485_DE_RE_PIN, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(RS485_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));
    ESP_LOGI(TAG, "UART2 (RS485) initialized: TX=%d, RX=%d, RTS=%d",
             RS485_TX_PIN, RS485_RX_PIN, RS485_DE_RE_PIN);
}

/* =========================================================
 * 4. XU LY MODBUS-RTU DOC CAM BIEN GIO
 * ========================================================= */
static uint16_t modbus_crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

static int modbus_read_wind_register(uint16_t *out_val) {
    uint8_t request[] = {
        MODBUS_SLAVE_ADDR,
        0x03, /* Read Holding Registers */
        (uint8_t)(REG_WIND_SPEED >> 8),
        (uint8_t)(REG_WIND_SPEED & 0xFF),
        0x00, 0x01
    };
    uint8_t frame[16];
    memcpy(frame, request, sizeof(request));
    uint16_t crc = modbus_crc16(frame, sizeof(request));
    frame[sizeof(request)] = (uint8_t)(crc & 0xFF);
    frame[sizeof(request) + 1] = (uint8_t)((crc >> 8) & 0xFF);
    uint8_t total_len = sizeof(request) + 2;

    uart_flush_input(RS485_UART_PORT);
    uart_write_bytes(RS485_UART_PORT, frame, total_len);
    uart_wait_tx_done(RS485_UART_PORT, pdMS_TO_TICKS(100));

    uint8_t response[32] = {0};
    int received = 0;
    int64_t deadline = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS + 500;
    while ((int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS < deadline) {
        int n = uart_read_bytes(RS485_UART_PORT, response + received,
                                sizeof(response) - received, pdMS_TO_TICKS(40));
        if (n > 0) {
            received += n;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (received >= 7) break;
    }

    if (received < 7) return -1;
    if (response[0] != MODBUS_SLAVE_ADDR || response[1] != 0x03) return -1;

    uint16_t crc_calc = modbus_crc16(response, received - 2);
    uint16_t crc_recv = (uint16_t)response[received - 2] | ((uint16_t)response[received - 1] << 8);
    if (crc_calc != crc_recv) return -1;

    *out_val = ((uint16_t)response[3] << 8) | response[4];
    return 0;
}

/* Task doc cam bien gio ngam */
static void wind_sensor_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(1500));
    while (1) {
        uint16_t raw_val = 0;
        if (modbus_read_wind_register(&raw_val) == 0) {
            g_latest_wind_speed = raw_val / 10.0f;
            if (g_latest_wind_speed < 0.3f) g_latest_wind_level = 0;
            else if (g_latest_wind_speed < 1.6f) g_latest_wind_level = 1;
            else if (g_latest_wind_speed < 3.4f) g_latest_wind_level = 2;
            else if (g_latest_wind_speed < 5.5f) g_latest_wind_level = 3;
            else if (g_latest_wind_speed < 8.0f) g_latest_wind_level = 4;
            else if (g_latest_wind_speed < 10.8f) g_latest_wind_level = 5;
            else if (g_latest_wind_speed < 13.9f) g_latest_wind_level = 6;
            else g_latest_wind_level = 7;
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

/* =========================================================
 * 5. XU LY GUI NHAN UART1 CHO XY-LPWM
 * ========================================================= */
static bool pwm_send_command(const char *cmd, char *out_resp, size_t out_max) {
    uart_flush(PWM_UART_PORT);
    uart_write_bytes(PWM_UART_PORT, cmd, strlen(cmd));

    char resp[64] = {0};
    int len = 0;
    int64_t deadline = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS + RESP_TIMEOUT_MS;
    while ((int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS < deadline) {
        int n = uart_read_bytes(PWM_UART_PORT, (uint8_t *)(resp + len),
                                sizeof(resp) - len - 1, pdMS_TO_TICKS(80));
        if (n > 0) {
            len += n;
            resp[len] = '\0';
            if (strstr(resp, "DOWN") || strstr(resp, "FALL")) break;
        }
    }

    if (len > 0) {
        snprintf(out_resp, out_max, "%s", resp);
        return (strstr(resp, "DOWN") != NULL);
    }
    snprintf(out_resp, out_max, "TIMEOUT");
    return false;
}

static void pwm_read_info(char *out_resp, size_t out_max) {
    uart_flush(PWM_UART_PORT);
    uart_write_bytes(PWM_UART_PORT, "read", 4);

    char resp[64] = {0};
    int len = 0;
    int64_t deadline = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS + RESP_TIMEOUT_MS;
    while ((int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS < deadline) {
        int n = uart_read_bytes(PWM_UART_PORT, (uint8_t *)(resp + len),
                                sizeof(resp) - len - 1, pdMS_TO_TICKS(100));
        if (n > 0) {
            len += n;
            resp[len] = '\0';
            vTaskDelay(pdMS_TO_TICKS(150));
            int n2 = uart_read_bytes(PWM_UART_PORT, (uint8_t *)(resp + len),
                                     sizeof(resp) - len - 1, pdMS_TO_TICKS(100));
            if (n2 > 0) len += n2;
            resp[len] = '\0';
            break;
        }
    }

    if (len > 0) snprintf(out_resp, out_max, "%s", resp);
    else snprintf(out_resp, out_max, "No Response");
}

/* =========================================================
 * 6. WEB SERVER HANDLERS (HTTP REST API)
 * ========================================================= */

/* Nhúng file index.html qua mảng ký tự hoặc đọc từ phân vùng */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/* Handler: GET / (Trang chủ) */
static esp_err_t index_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Nếu có nhúng file binary index.html qua CMake:
    const size_t index_len = index_html_end - index_html_start;
    if (index_len > 0) {
        return httpd_resp_send(req, (const char *)index_html_start, index_len);
    }

    // Trang fallback nếu chưa nhúng binary
    const char *fallback_html =
        "<!DOCTYPE html><html><head><meta charset='utf-8'><title>ESP32 Hub</title></head>"
        "<body style='font-family:sans-serif;background:#0f172a;color:#fff;padding:30px;'>"
        "<h2>ESP32 Web Server Active</h2>"
        "<p>Vui long mo file <b>web/index.html</b> tren may tinh va ket noi toi IP nay de xem Dashboard day du.</p>"
        "</body></html>";
    return httpd_resp_send(req, fallback_html, HTTPD_RESP_USE_STRLEN);
}

/* Handler: GET /api/status */
static esp_err_t api_status_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", "ok");
    cJSON_AddNumberToObject(root, "wind_speed", g_latest_wind_speed);
    cJSON_AddNumberToObject(root, "wind_level", g_latest_wind_level);
    cJSON_AddNumberToObject(root, "relay", g_relay_state ? 1 : 0);
    cJSON_AddStringToObject(root, "pwm_freq", g_last_pwm_freq);
    cJSON_AddStringToObject(root, "pwm_duty", g_last_pwm_duty);
    cJSON_AddStringToObject(root, "pwm_resp", g_last_pwm_resp);

    char *json_str = cJSON_PrintUnformatted(root);
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    free(json_str);
    cJSON_Delete(root);
    return ESP_OK;
}

/* Handler: GET /api/wind */
static esp_err_t api_wind_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", "ok");
    cJSON_AddNumberToObject(root, "speed", g_latest_wind_speed);
    cJSON_AddNumberToObject(root, "level", g_latest_wind_level);

    char *json_str = cJSON_PrintUnformatted(root);
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    free(json_str);
    cJSON_Delete(root);
    return ESP_OK;
}

/* Handler: POST /api/relay */
static esp_err_t api_relay_post_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char buf[128] = {0};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON *json = cJSON_Parse(buf);
    if (json) {
        cJSON *state_item = cJSON_GetObjectItem(json, "state");
        if (state_item && cJSON_IsNumber(state_item)) {
            set_relay_state(state_item->valueint == 1);
        }
        cJSON_Delete(json);
    }

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "status", "ok");
    cJSON_AddNumberToObject(resp, "relay", g_relay_state ? 1 : 0);

    char *json_str = cJSON_PrintUnformatted(resp);
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    free(json_str);
    cJSON_Delete(resp);
    return ESP_OK;
}

/* Handler: POST /api/pwm */
static esp_err_t api_pwm_post_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char buf[256] = {0};
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    bool success = true;
    char combined_resp[128] = {0};

    cJSON *json = cJSON_Parse(buf);
    if (json) {
        cJSON *freq_item = cJSON_GetObjectItem(json, "freq");
        cJSON *duty_item = cJSON_GetObjectItem(json, "duty");

        if (freq_item && cJSON_IsString(freq_item)) {
            snprintf(g_last_pwm_freq, sizeof(g_last_pwm_freq), "%s", freq_item->valuestring);
            char r[32] = {0};
            bool ok = pwm_send_command(g_last_pwm_freq, r, sizeof(r));
            if (!ok) success = false;
            snprintf(combined_resp, sizeof(combined_resp), "Freq: %s", r);
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        if (duty_item && cJSON_IsString(duty_item)) {
            snprintf(g_last_pwm_duty, sizeof(g_last_pwm_duty), "%s", duty_item->valuestring);
            char r[32] = {0};
            bool ok = pwm_send_command(g_last_pwm_duty, r, sizeof(r));
            if (!ok) success = false;
            strncat(combined_resp, " | Duty: ", sizeof(combined_resp) - strlen(combined_resp) - 1);
            strncat(combined_resp, r, sizeof(combined_resp) - strlen(combined_resp) - 1);
        }

        snprintf(g_last_pwm_resp, sizeof(g_last_pwm_resp), "%.127s", combined_resp);
        cJSON_Delete(json);
    }

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "status", success ? "ok" : "warn");
    cJSON_AddStringToObject(resp, "message", combined_resp);
    cJSON_AddStringToObject(resp, "freq", g_last_pwm_freq);
    cJSON_AddStringToObject(resp, "duty", g_last_pwm_duty);

    char *json_str = cJSON_PrintUnformatted(resp);
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    free(json_str);
    cJSON_Delete(resp);
    return ESP_OK;
}

/* Handler: GET /api/pwm/read */
static esp_err_t api_pwm_read_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char raw_resp[64] = {0};
    pwm_read_info(raw_resp, sizeof(raw_resp));

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "status", "ok");
    cJSON_AddStringToObject(resp, "raw", raw_resp);
    cJSON_AddStringToObject(resp, "freq", g_last_pwm_freq);
    cJSON_AddStringToObject(resp, "duty", g_last_pwm_duty);

    char *json_str = cJSON_PrintUnformatted(resp);
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    free(json_str);
    cJSON_Delete(resp);
    return ESP_OK;
}

/* Khoi chay HTTP Server */
static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 10;
    config.stack_size = 8192;

    ESP_LOGI(TAG, "Starting HTTP server on port: %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // GET /
        httpd_uri_t uri_index = { .uri = "/", .method = HTTP_GET, .handler = index_get_handler };
        httpd_register_uri_handler(server, &uri_index);

        // GET /api/status
        httpd_uri_t uri_status = { .uri = "/api/status", .method = HTTP_GET, .handler = api_status_get_handler };
        httpd_register_uri_handler(server, &uri_status);

        // GET /api/wind
        httpd_uri_t uri_wind = { .uri = "/api/wind", .method = HTTP_GET, .handler = api_wind_get_handler };
        httpd_register_uri_handler(server, &uri_wind);

        // POST /api/relay
        httpd_uri_t uri_relay = { .uri = "/api/relay", .method = HTTP_POST, .handler = api_relay_post_handler };
        httpd_register_uri_handler(server, &uri_relay);

        // POST /api/pwm
        httpd_uri_t uri_pwm = { .uri = "/api/pwm", .method = HTTP_POST, .handler = api_pwm_post_handler };
        httpd_register_uri_handler(server, &uri_pwm);

        // GET /api/pwm/read
        httpd_uri_t uri_pwm_read = { .uri = "/api/pwm/read", .method = HTTP_GET, .handler = api_pwm_read_handler };
        httpd_register_uri_handler(server, &uri_pwm_read);

        return server;
    }
    ESP_LOGE(TAG, "Error starting HTTP server!");
    return NULL;
}

/* =========================================================
 * 7. KHOI TAO WIFI SOFT-AP
 * ========================================================= */
static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    if (strlen(WIFI_AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi SoftAP khoi tao thanh cong! SSID: %s | Pass: %s", WIFI_AP_SSID, WIFI_AP_PASS);
    ESP_LOGI(TAG, "Dia chi IP mac dinh cua Web Server: http://192.168.4.1/");
}

/* =========================================================
 * 8. ENTRY POINT CHO WEB SERVER SYSTEM
 * ========================================================= */
void web_app_system_start(void) {
    // 1. Khoi tao NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Khoi tao phan cung
    relay_init();
    pwm_uart_init();
    rs485_init();

    // 3. Khoi tao WiFi & Web Server
    wifi_init_softap();
    start_webserver();

    // 4. Khoi chay task doc cam bien gio ngam
    xTaskCreate(wind_sensor_task, "wind_sensor_bg", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "=== HE THONG WEB SERVER SAN SANG HOAT DONG ===");
}

void app_main(void) {
    web_app_system_start();
}

