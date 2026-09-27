#ifdef ARDUINO

#include "hal/proto_uart.h"

#include <Arduino.h>

#include "meister_config.h"

namespace meister {
namespace hal {
namespace proto_uart {
namespace {

// UART0 (Serial) か UART2 (Serial2) かを MSTE_UART_BACKEND_USB0 で決める。
HardwareSerial& port() {
#if MSTE_UART_BACKEND_USB0
  return Serial;
#else
  return Serial2;
#endif
}

}  // namespace

void begin() {
#if MSTE_UART_BACKEND_USB0
  // UART0 がプロトコル線になる。コンソール出力は MSTE_LOG で無効化されている。
  Serial.begin(MSTE_UART_BAUD);
#else
  Serial.begin(115200);  // コンソール（UART0）
  Serial2.begin(MSTE_UART_BAUD, SERIAL_8N1, MSTE_UART_RX_PIN, MSTE_UART_TX_PIN);
#endif
}

int available() { return port().available(); }

int read() { return port().read(); }

size_t write(const uint8_t* data, size_t len) { return port().write(data, len); }

}  // namespace proto_uart
}  // namespace hal
}  // namespace meister

#endif  // ARDUINO
