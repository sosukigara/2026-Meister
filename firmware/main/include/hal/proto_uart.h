/*
 * proto_uart.h — プロトコル UART のリンク層
 *
 * どのポート（Serial2 / UART0）を使うか、ボーレートとピンは
 * meister_config.h の MSTE_* が決める。呼び出し側はその選択を知る必要なし。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace meister {
namespace hal {
namespace proto_uart {

/// 受信・送信ポートを初期化する
void begin();

/// 受信可能なバイト数
int available();

/// 1 バイト読む。空なら -1
int read();

/// 送信する。書けたバイト数を返す
size_t write(const uint8_t* data, size_t len);

}  // namespace proto_uart
}  // namespace hal
}  // namespace meister
