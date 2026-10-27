/**
 * @file main.c
 * @brief Doc cam bien gio RS485 qua giao thuc Modbus-RTU (co che do scan dia
 * chi)
 *
 * Ket noi phan cung:
 *   ESP32 GPIO17 (TX2) -----> DI  (MAX485)
 *   ESP32 GPIO16 (RX2) <----- RO  (MAX485)
 *   ESP32 GPIO4        -----> DE + RE (MAX485)
 *   ESP32 GND          -----> GND (MAX485)
 *   MAX485 A           <----> Day Vang  (485-A cam bien)
 *   MAX485 B           <----> Day Xanh la (485-B cam bien)
 *
 * Thong so cam bien:
 *   - Giao thuc: Modbus-RTU
 *   - Baud rate: 9600 (mac dinh)
 *   - Dia chi thiet bi: 0x01 (mac dinh)
 *   - Register 0x0000: Toc do gio tuc thoi (gia tri / 10 = thuc, don vi m/s)
 *   - Register 0x0001: Cap do gio 0~12
 *
 * CHE DO HOAT DONG:
 *   SCAN_MODE = 1 -> Quet tat ca dia chi 0x01~0x0F de tim cam bien
 *   SCAN_MODE = 0 -> Doc dinh ky toc do gio moi 2 giay
 */

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* =========================================================
 * Cau hinh phan cung
 * ========================================================= */
#define RS485_UART_PORT UART_NUM_2
#define RS485_TX_PIN GPIO_NUM_17   /* DI  cua MAX485 */
#define RS485_RX_PIN GPIO_NUM_16   /* RO  cua MAX485 */
#define RS485_DE_RE_PIN GPIO_NUM_4 /* DE + RE cua MAX485 */
#define RS485_BAUD_RATE 9600

/* =========================================================
 * Cau hinh Modbus
 * ========================================================= */
#define MODBUS_SLAVE_ADDR 0x03     /* Dia chi mac dinh cua cam bien */
#define REG_WIND_SPEED 0x0000      /* Register toc do gio */
#define REG_WIND_LEVEL 0x0001      /* Register cap do gio */
#define MODBUS_FUNC_READ_REGS 0x03 /* Function code: Read Holding Registers */

#define BUF_SIZE 256
#define MODBUS_RESPONSE_TIMEOUT_MS 500

static const char *TAG = "WIND_SENSOR";

/* =========================================================
 * Tinh CRC16 chuan Modbus-RTU
 * ========================================================= */
static uint16_t modbus_crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/* =========================================================
 * Khoi tao UART2 o che do Hardware RS485 Half-Duplex
 * Phan cung ESP32 tu dong dieu khien chan RTS (DE/RE)
 * ========================================================= */
static void rs485_init(void) {
  /* Cau hinh UART2 cho RS485 */
  uart_config_t uart_cfg = {
      .baud_rate = RS485_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(
      uart_driver_install(RS485_UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(RS485_UART_PORT, &uart_cfg));
  /* Gan chan TX, RX va RTS (DE/RE) cho UART phan cung */
  ESP_ERROR_CHECK(uart_set_pin(RS485_UART_PORT, RS485_TX_PIN, RS485_RX_PIN,
                               RS485_DE_RE_PIN, UART_PIN_NO_CHANGE));
  /* Bat che do Hardware RS485 Half-Duplex */
  ESP_ERROR_CHECK(uart_set_mode(RS485_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));

  ESP_LOGI(TAG,
           "RS485 da khoi tao (Hardware Half-Duplex) | UART2 | TX=GPIO%d | "
           "RX=GPIO%d | RTS(DE/RE)=GPIO%d | %d baud",
           RS485_TX_PIN, RS485_RX_PIN, RS485_DE_RE_PIN, RS485_BAUD_RATE);
}

/* =========================================================
 * Gui frame Modbus-RTU va nhan phan hoi
 *   request : bo dem lenh (CHUA co CRC)
 *   req_len : so byte lenh
 *   response: bo dem nhan phan hoi
 *   resp_max: kich thuoc bo dem
 *   return  : so byte nhan duoc, -1 neu loi
 * ========================================================= */
static int modbus_send_recv(const uint8_t *request, uint8_t req_len,
                            uint8_t *response, uint8_t resp_max) {
  uint8_t frame[16];
  memcpy(frame, request, req_len);

  /* Tinh va gan CRC vao cuoi frame (Little-endian: Low truoc, High sau) */
  uint16_t crc = modbus_crc16(frame, req_len);
  frame[req_len] = (uint8_t)(crc & 0xFF);
  frame[req_len + 1] = (uint8_t)((crc >> 8) & 0xFF);
  uint8_t total_len = req_len + 2;

  /* In frame gui de debug */
  printf("[TX] ");
  for (int i = 0; i < total_len; i++)
    printf("%02X ", frame[i]);
  printf("\n");

  /* Xoa buffer RX, gui du lieu (Hardware tu dong bat RTS len cao khi truyen va
   * ha xuong khi xong) */
  uart_flush_input(RS485_UART_PORT);
  uart_write_bytes(RS485_UART_PORT, frame, total_len);
  uart_wait_tx_done(RS485_UART_PORT, pdMS_TO_TICKS(100));

  int received = 0;
  int64_t deadline = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS +
                     MODBUS_RESPONSE_TIMEOUT_MS;

  while ((int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS < deadline) {
    int n = uart_read_bytes(RS485_UART_PORT, response + received,
                            resp_max - received, pdMS_TO_TICKS(50));
    if (n > 0) {
      received += n;
      vTaskDelay(pdMS_TO_TICKS(20)); /* Doi them de dam bao nhan het frame */
    }
    if (received >= 7)
      break; /* Da nhan du response toi thieu */
  }

  if (received == 0) {
    ESP_LOGW(TAG, "Khong nhan duoc phan hoi! Kiem tra: day A/B, baud rate, dia "
                  "chi slave.");
    return -1;
  }

  /* In frame nhan de debug */
  printf("[RX] ");
  for (int i = 0; i < received; i++)
    printf("%02X ", response[i]);
  printf("\n");

  /* Kiem tra CRC phan hoi */
  if (received >= 4) {
    uint16_t crc_calc = modbus_crc16(response, received - 2);
    uint16_t crc_recv = (uint16_t)response[received - 2] |
                        ((uint16_t)response[received - 1] << 8);
    if (crc_calc != crc_recv) {
      ESP_LOGW(TAG, "Loi CRC! Tinh: 0x%04X | Nhan: 0x%04X", crc_calc, crc_recv);
      return -1;
    }
  }

  return received;
}

/* =========================================================
 * Doc mot register 16-bit tu cam bien Modbus
 *   return: 0 thanh cong, -1 loi
 * ========================================================= */
static int modbus_read_register(uint8_t slave_addr, uint16_t reg_addr,
                                uint16_t *value) {
  /* Frame: [addr][func][reg_hi][reg_lo][count_hi][count_lo] */
  uint8_t request[] = {
      slave_addr,
      MODBUS_FUNC_READ_REGS,
      (uint8_t)(reg_addr >> 8),
      (uint8_t)(reg_addr & 0xFF),
      0x00,
      0x01 /* Doc 1 register */
  };

  uint8_t response[32] = {0};
  int len =
      modbus_send_recv(request, sizeof(request), response, sizeof(response));

  if (len < 7)
    return -1;

  /* Xac nhan dia chi slave va function code */
  if (response[0] != slave_addr || response[1] != MODBUS_FUNC_READ_REGS) {
    ESP_LOGW(TAG, "Phan hoi khong hop le: addr=0x%02X func=0x%02X", response[0],
             response[1]);
    return -1;
  }

  /* response[2] = so byte data | response[3..4] = gia tri (Big-endian) */
  *value = ((uint16_t)response[3] << 8) | response[4];
  return 0;
}

/* =========================================================
 * Task chinh: doc cam bien gio dinh ky moi 2 giay
 * ========================================================= */
static void wind_sensor_task(void *arg) {
  ESP_LOGI(TAG, "=== Bat dau doc cam bien gio RS485 ===");
  ESP_LOGI(TAG, "Modbus-RTU | Slave addr: 0x%02X | Baud: %d", MODBUS_SLAVE_ADDR,
           RS485_BAUD_RATE);

  vTaskDelay(pdMS_TO_TICKS(1000)); /* Cho cam bien on dinh khi moi cap nguon */

  while (1) {
    uint16_t raw_speed = 0;
    uint16_t wind_level = 0;

    /* --- Doc toc do gio --- */
    if (modbus_read_register(MODBUS_SLAVE_ADDR, REG_WIND_SPEED, &raw_speed) ==
        0) {
      float wind_speed_ms = raw_speed / 10.0f; /* Chia 10 theo tai lieu */
      ESP_LOGI(TAG, "TOC DO GIO : %.1f m/s  (raw=0x%04X)", wind_speed_ms,
               raw_speed);
    } else {
      ESP_LOGE(TAG, "LOI doc register toc do gio (0x%04X)", REG_WIND_SPEED);
    }

    vTaskDelay(pdMS_TO_TICKS(200)); /* Khoang cach giua hai lenh Modbus */

    // /* --- Doc cap do gio --- */
    // if (modbus_read_register(MODBUS_SLAVE_ADDR, REG_WIND_LEVEL, &wind_level)
    // ==
    //     0) {
    //   ESP_LOGI(TAG, "CAP DO GIO : %d  (thang Beaufort 0-12)", wind_level);
    // } else {
    //   ESP_LOGE(TAG, "LOI doc register cap do gio (0x%04X)", REG_WIND_LEVEL);
    // }

    printf("--------------------------------------------------\n");
    vTaskDelay(pdMS_TO_TICKS(2000)); /* Doc lai sau 2 giay */
  }
}

/* =========================================================
 * Entry point
 * ========================================================= */
void app_main(void) {
  rs485_init();
  xTaskCreate(wind_sensor_task, "wind_sensor", 4096, NULL, 5, NULL);
}
