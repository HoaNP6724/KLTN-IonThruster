/**
 * @file main.c
 * @brief Dieu khien module PWM XY-LPWM bang cach nhap tu ban phim (Serial
 * Monitor)
 *
 * Ket noi:
 *   ESP32 GPIO18 (TX1) <-----> RXD module
 *   ESP32 GPIO19 (RX1) <-----> TXD module
 *   ESP32 GND          <-----> GND module
 *
 * Cach su dung (Serial Monitor 115200 bps):
 *   Nhap: F<gia_tri>   -> dat tan so  (vi du: F1.00, F10.0, F100, F1.0.0)
 *   Nhap: D<gia_tri>   -> dat duty %%  (vi du: D050 = 50%%)
 *   Nhap: read         -> doc thong so hien tai
 */

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>


/* --- UART1: giao tiep voi module PWM (khong bi trung UART2 cua RS485) --- */
#define PWM_UART_PORT UART_NUM_1
#define PWM_UART_TX_PIN GPIO_NUM_18
#define PWM_UART_RX_PIN GPIO_NUM_19
#define PWM_UART_BAUD 9600

/* --- UART0: doc input tu Serial Monitor --- */
#define CON_UART_PORT UART_NUM_0
#define CON_UART_BAUD 115200

#define BUF_SIZE 256
#define RESP_TIMEOUT_MS 2000

static const char *TAG = "PWM";

/* =========================================================
 * Khoi tao UART
 * ========================================================= */
static void uart_init(void) {
  /* UART1 - module PWM */
  uart_config_t cfg1 = {
      .baud_rate = PWM_UART_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(
      uart_driver_install(PWM_UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(PWM_UART_PORT, &cfg1));
  ESP_ERROR_CHECK(uart_set_pin(PWM_UART_PORT, PWM_UART_TX_PIN, PWM_UART_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  /* UART0 - console (co the da duoc khoi tao boi IDF, cai dat lai baud) */
  uart_config_t cfg0 = {
      .baud_rate = CON_UART_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(
      uart_driver_install(CON_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(CON_UART_PORT, &cfg0));
}

/* =========================================================
 * Gui lenh den module, cho phan hoi DOWN/FALL
 * ========================================================= */
static void pwm_send(const char *cmd) {
  char resp[32] = {0};
  int len = 0;

  uart_flush(PWM_UART_PORT);
  uart_write_bytes(PWM_UART_PORT, cmd, strlen(cmd));

  int64_t deadline =
      (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS + RESP_TIMEOUT_MS;
  while ((int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS < deadline) {
    int n = uart_read_bytes(PWM_UART_PORT, (uint8_t *)(resp + len),
                            sizeof(resp) - len - 1, pdMS_TO_TICKS(100));
    if (n > 0) {
      len += n;
      resp[len] = '\0';
      if (strstr(resp, "DOWN")) {
        printf("  -> OK (DOWN)\n");
        return;
      } else if (strstr(resp, "FALL")) {
        printf("  -> FAIL (FALL)\n");
        return;
      }
    }
  }
  printf("  -> TIMEOUT (kiem tra ket noi!)\n");
}

/* =========================================================
 * Doc thong so tu module - in tat ca du lieu nhan duoc
 * (Module tra ve gia tri tan so/duty, khong phai DOWN/FALL)
 * ========================================================= */
static void pwm_read(void) {
  char resp[64] = {0};
  int len = 0;
  bool got_data = false;

  uart_flush(PWM_UART_PORT);
  uart_write_bytes(PWM_UART_PORT, "read", 4);
  printf("  Gui: \"read\"\n");

  /* Doc du lieu trong 2 giay, in tat ca bytes nhan duoc */
  int64_t deadline =
      (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS + RESP_TIMEOUT_MS;
  while ((int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS < deadline) {
    int n = uart_read_bytes(PWM_UART_PORT, (uint8_t *)(resp + len),
                            sizeof(resp) - len - 1, pdMS_TO_TICKS(100));
    if (n > 0) {
      len += n;
      resp[len] = '\0';
      got_data = true;
      /* Tiep tuc doc them trong 200ms de nhan du phan hoi */
      vTaskDelay(pdMS_TO_TICKS(200));
      /* Doc phan con lai neu co */
      int n2 = uart_read_bytes(PWM_UART_PORT, (uint8_t *)(resp + len),
                               sizeof(resp) - len - 1, pdMS_TO_TICKS(100));
      if (n2 > 0) {
        len += n2;
        resp[len] = '\0';
      }
      break;
    }
  }

  if (got_data) {
    /* In du lieu hex va ASCII de debug */
    printf("  -> Phan hoi (%d bytes): ", len);
    for (int i = 0; i < len; i++) {
      if (resp[i] >= 0x20 && resp[i] < 0x7F)
        printf("%c", resp[i]);
      else
        printf("[%02X]", (uint8_t)resp[i]);
    }
    printf("\n");
  } else {
    printf("  -> Khong co phan hoi (module co the khong ho tro lenh read)\n");
  }
}

/* =========================================================
 * In huong dan su dung
 * ========================================================= */
static void print_help(void) {
  printf("\n============================================\n");
  printf("  DIEU KHIEN MODULE PWM XY-LPWM\n");
  printf("============================================\n");
  printf("  Dat tan so:\n");
  printf("    F001 ~ F999        -> 1 Hz ~ 999 Hz\n");
  printf("    F1.00 ~ F9.99      -> 1.00 kHz ~ 9.99 kHz\n");
  printf("    F10.0 ~ F99.9      -> 10.0 kHz ~ 99.9 kHz\n");
  printf("    F1.0.0 ~ F1.5.0    -> 100 kHz ~ 150 kHz\n");
  printf("  Dat duty cycle:\n");
  printf("    D001 ~ D100        -> 1%% ~ 100%%\n");
  printf("  Doc thong so: read\n");
  printf("  Tro giup:      help\n");
  printf("============================================\n");
  printf("  Vi du: F1.00  roi nhan Enter\n");
  printf("         D050   roi nhan Enter\n");
  printf("============================================\n\n");
}

/* =========================================================
 * Task chinh: doc input tu console, gui lenh
 * ========================================================= */
static void console_task(void *arg) {
  char input[32] = {0};
  int idx = 0;
  uint8_t ch;

  print_help();
  printf("Nhap lenh: ");
  fflush(stdout);

  while (1) {
    /* Doc tung ky tu tu UART0 */
    int n = uart_read_bytes(CON_UART_PORT, &ch, 1, pdMS_TO_TICKS(20));
    if (n <= 0)
      continue;

    /* Echo ky tu vua nhap */
    uart_write_bytes(CON_UART_PORT, (char *)&ch, 1);

    if (ch == '\r' || ch == '\n') {
      printf("\n");
      input[idx] = '\0';

      /* Xu ly lenh */
      if (idx == 0) {
        /* Enter rong, bo qua */
      } else if (strcasecmp(input, "help") == 0) {
        print_help();
      } else if (strcasecmp(input, "read") == 0) {
        pwm_read();
      } else if (input[0] == 'F' || input[0] == 'f') {
        printf("  Gui: \"%s\"\n", input);
        /* Chuan hoa chu hoa */
        input[0] = 'F';
        pwm_send(input);
      } else if (input[0] == 'D' || input[0] == 'd') {
        printf("  Gui: \"%s\"\n", input);
        input[0] = 'D';
        pwm_send(input);
      } else {
        printf("  Lenh khong hop le! Nhap 'help' de xem huong dan.\n");
      }

      /* Reset input */
      idx = 0;
      memset(input, 0, sizeof(input));
      printf("Nhap lenh: ");
      fflush(stdout);

    } else if (ch == 127 || ch == '\b') {
      /* Backspace */
      if (idx > 0) {
        idx--;
        input[idx] = '\0';
        printf("\b \b");
        fflush(stdout);
      }
    } else if (idx < (int)(sizeof(input) - 1)) {
      input[idx++] = (char)ch;
    }
  }
}

/* =========================================================
 * Entry point
 * ========================================================= */
void app_main(void) {
  uart_init();
  ESP_LOGI(TAG, "Module PWM ready | TX1=GPIO%d | RX1=GPIO%d", PWM_UART_TX_PIN,
           PWM_UART_RX_PIN);
  xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);
}
