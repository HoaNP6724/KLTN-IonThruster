#ifndef WEB_SERVER_SYSTEM_H
#define WEB_SERVER_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Khoi chay toan bo he thong Web Server:
 *        - Khoi tao NVS Flash
 *        - Khoi tao GPIO Relay (GPIO 23)
 *        - Khoi tao UART1 (XY-LPWM TX=18, RX=19)
 *        - Khoi tao UART2 RS485 (TX=17, RX=16, DE/RE=4)
 *        - Phat WiFi Access Point "ESP32-KLTN-HUB" (Pass: 12345678, IP: 192.168.4.1)
 *        - Chay HTTP Web Server & cac REST API (/api/status, /api/wind, /api/pwm, /api/relay)
 *        - Chay task doc cam bien gio ngam dinh ky
 */
void web_app_system_start(void);

#ifdef __cplusplus
}
#endif

#endif // WEB_SERVER_SYSTEM_H
