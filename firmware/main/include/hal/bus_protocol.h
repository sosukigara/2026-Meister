/*
 * bus_protocol.h — STS/SCS バスサーボのパケット codec
 *
 * Arduino 非依存。ここだけが pio test -e native でホスト側テストできる。
 * 仕様は hal/feetech_sts_registers.h を参照。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "hal/feetech_sts_registers.h"

namespace meister {
namespace bus {

using feetech::Endianness;
using feetech::ErrorBit;
using feetech::Instruction;

// ===========================================================================
// 送信パケットの組み立て
//
// 戻り値: 書き込んだバイト数。バッファ不足・パラメータ不正なら 0。
//         0xFF 0xFF ヘッダとチェックサムは本モジュールが計算する。
// ===========================================================================

/// PING（存在確認）。応答は length=0 の status
/// 例: FF FF 01 02 01 FB
size_t BuildPing(uint8_t* buf, size_t cap, uint8_t id);

/// READ（計測レジスタの読み出し）
/// 例: FF FF 05 04 02 37 01 BC
size_t BuildRead(uint8_t* buf, size_t cap, uint8_t id,
                 uint8_t address, uint8_t length);

/// WRITE（命令レジスタの書き込み。Torque Enable が ON のときだけ効く）
/// 例: FF FF 05 05 03 2A E8 03 DD   ← 長さバイトは無い点に注意
size_t BuildWrite(uint8_t* buf, size_t cap, uint8_t id,
                  uint8_t address, const uint8_t* data, uint8_t length,
                  Endianness endianness);

/// SYNC_WRITE（ブロードキャストで複数 ID に同時書き込み）
/// 例: FF FF FE <len> 83 <addr> <data_len> <id1> <d1l> <d1h> <id2> ... <chk>
///   ids[i] / values[i] は同じ個数。values は 2 バイト（リトルエンディアン固定）。
size_t BuildSyncWrite(uint8_t* buf, size_t cap, uint8_t address,
                      const uint8_t* ids, const uint16_t* values, size_t count);

/// 16 ビット値を 2 バイトに分割する（バイト順は endianness に従う）
void SplitWord(uint16_t value, uint8_t* out, Endianness endianness);

/// 2 バイトから 16 ビット値を復元する
uint16_t JoinWord(const uint8_t* bytes, Endianness endianness);

// ===========================================================================
// 応答パケット（status）の解析
// ===========================================================================

/// 解析結果。`data` は `buf` を指す（コピーしない）。次のパケットで上書きされる。
struct Status {
  uint8_t id = 0;
  uint8_t error = 0;        ///< ErrorBit の合成
  const uint8_t* data = nullptr;
  uint8_t data_length = 0;
  bool has_error() const { return error != 0; }
};

/// 1 パケットを検証して解析する。
/// 戻り値: 成功時 true。ヘッダ/ID/チェックサム不一致や長さ不足なら false。
///   *out は成功時のみ有効。
bool ParseStatus(const uint8_t* buf, size_t len, Status* out);

/// 1 パケットの総バイト数を返す。ヘッダ不一致なら 0。
/// 「何バイト待てば 1 パケット揃うか」を調べるのに使う。
size_t StatusPacketSize(const uint8_t* buf, size_t len);

/// 応答の error ビットから文言を引く（ログ用）
const char* ErrorName(uint8_t error_bit);

// ===========================================================================
// ストリーム parser
//
// 応答は 1 バイトずつ届くので、バッファに溜めてから ParseStatus する。
// 壊れたパケットは捨てて、次の 0xFF 0xFF へ再同期する。
// ===========================================================================
class StatusParser {
 public:
  void feed(const uint8_t* data, size_t len);

  /// 解析できたパケットを順に out へ格納する。戻り値は格納数。
  /// 値（Status::data）は内部バッファを指すので、呼び出し側でコピーすること。
  size_t drain(Status* out, size_t max_items);

  /// まだ解析されていないバイト数
  size_t pending() const { return buffer_length_; }

  /// 破棄した（壊れた）パケット数。ベンチ時の健全性判定に使う。
  uint32_t dropped() const { return dropped_; }

  void clear();

 private:
  static constexpr size_t kCapacity = feetech::kMaxPacketSize * 2;
  uint8_t buffer_[kCapacity] = {};
  size_t buffer_length_ = 0;
  uint32_t dropped_ = 0;
};

/// チェックサム計算（~(ID..PARAM 末尾の単純和) & 0xFF）
uint8_t ComputeChecksum(const uint8_t* data, size_t len);

}  // namespace bus
}  // namespace meister
