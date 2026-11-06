/*
 * bus_protocol.cpp — STS/SCS バスサーボのパケット codec
 *
 * 出典: Feetech 公式 Python SDK `feetech-servo-sdk` 1.0.0
 *       scservo_def.py / protocol_packet_handler.py
 * チェックサムは XOR ではなく bit 否定の単純和。
 */
#include "hal/bus_protocol.h"

#include <string.h>

#include "hal/feetech_sts_registers.h"

namespace meister {
namespace bus {
namespace {

/// 応答として現れてよい ID（一斉送信の 0xFE は応答しない）
bool IsValidId(uint8_t id) {
  return id >= feetech::kIdMin && id <= feetech::kIdMax;
}

/// 1 フレームの総バイト数（length フィールドが不正なときは 0）
size_t PacketSize(const uint8_t* buf) {
  const size_t size =
      static_cast<size_t>(buf[feetech::kOffsetLength]) +
      feetech::kLengthFieldSize;
  if (size < feetech::kMinPacketSize || size > feetech::kMaxPacketSize) {
    return 0;
  }
  return size;
}

/// length フィールド = instruction(1) + parameters + checksum(1)
size_t Finalize(uint8_t* buf, uint8_t id, uint8_t length, size_t total) {
  buf[0] = feetech::kHeader0;
  buf[1] = feetech::kHeader1;
  buf[2] = id;
  buf[3] = length;
  buf[total - 1] = ComputeChecksum(buf + feetech::kOffsetId,
                                   total - 1 - feetech::kOffsetId);
  return total;
}

/// length フィールド = instruction(1) + parameters + checksum(1)
constexpr size_t kLengthFieldOverhead = 2;

constexpr uint8_t kSyncWriteDataLen = 2;   ///< SYNC_WRITE の 1 要素のバイト数
constexpr size_t kSyncWriteEntrySize = 3;  ///< id + 下位 + 上位

}  // namespace

uint8_t ComputeChecksum(const uint8_t* data, size_t len) {
  if (data == nullptr) {
    return 0;
  }
  uint32_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum += data[i];
  }
  return static_cast<uint8_t>(~sum);
}

void SplitWord(uint16_t value, uint8_t* out, Endianness endianness) {
  if (out == nullptr) {
    return;
  }
  if (endianness == Endianness::kLittle) {
    out[0] = static_cast<uint8_t>(value & 0xFF);
    out[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  } else {
    out[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    out[1] = static_cast<uint8_t>(value & 0xFF);
  }
}

uint16_t JoinWord(const uint8_t* bytes, Endianness endianness) {
  if (bytes == nullptr) {
    return 0;
  }
  if (endianness == Endianness::kLittle) {
    return static_cast<uint16_t>(bytes[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(bytes[1]) << 8);
  }
  return static_cast<uint16_t>(static_cast<uint16_t>(bytes[0]) << 8) |
         static_cast<uint16_t>(bytes[1]);
}

size_t BuildPing(uint8_t* buf, size_t cap, uint8_t id) {
  const size_t total = feetech::kLengthFieldSize + kLengthFieldOverhead;
  if (buf == nullptr || !IsValidId(id) || cap < total) {
    return 0;
  }
  buf[feetech::kOffsetCmd] = feetech::kInstPing;
  return Finalize(buf, id, static_cast<uint8_t>(kLengthFieldOverhead), total);
}

size_t BuildRead(uint8_t* buf, size_t cap, uint8_t id,
                 uint8_t address, uint8_t length) {
  // READ のパラメータは address + length（WRITE と違い length バイトがある）
  const size_t total = feetech::kLengthFieldSize + kLengthFieldOverhead + 2;
  if (buf == nullptr || !IsValidId(id) || cap < total) {
    return 0;
  }
  buf[feetech::kOffsetCmd] = feetech::kInstRead;
  buf[feetech::kOffsetParam0] = address;
  buf[feetech::kOffsetParam0 + 1] = length;
  return Finalize(buf, id, static_cast<uint8_t>(kLengthFieldOverhead + 2), total);
}

size_t BuildWrite(uint8_t* buf, size_t cap, uint8_t id,
                  uint8_t address, const uint8_t* data, uint8_t length,
                  Endianness endianness) {
  // WRITE には length バイトが無い。パラメータは address + data だけ
  const size_t param_size = 1 + static_cast<size_t>(length);
  const size_t total = feetech::kLengthFieldSize + kLengthFieldOverhead + param_size;
  if (buf == nullptr || !IsValidId(id) || cap < total ||
      total > feetech::kMaxPacketSize) {
    return 0;
  }
  if (data == nullptr && length > 0) {
    return 0;
  }
  buf[feetech::kOffsetCmd] = feetech::kInstWrite;
  buf[feetech::kOffsetParam0] = address;
  if (length > 0) {
    memcpy(&buf[feetech::kOffsetParam0 + 1], data, length);
    // ビッグエンディ指定時は 16 ビット語ごとにバイトを交換する
    // （呼び出し側がホスト順に 16 ビット値を持つ場合の変換用）
    if (endianness == Endianness::kBig && (length % 2) == 0) {
      for (size_t i = feetech::kOffsetParam0 + 1; i + 1 < total - 1; i += 2) {
        const uint8_t tmp = buf[i];
        buf[i] = buf[i + 1];
        buf[i + 1] = tmp;
      }
    }
  }
  return Finalize(buf, id,
                  static_cast<uint8_t>(kLengthFieldOverhead + param_size), total);
}

size_t BuildSyncWrite(uint8_t* buf, size_t cap, uint8_t address,
                      const uint8_t* ids, const uint16_t* values, size_t count) {
  if (buf == nullptr || ids == nullptr || values == nullptr || count == 0) {
    return 0;
  }
  // パラメータ = address + data_len + count * (id + 2 バイト)
  const size_t param_size = 2 + count * kSyncWriteEntrySize;
  const size_t total = feetech::kLengthFieldSize + kLengthFieldOverhead + param_size;
  if (total > feetech::kMaxPacketSize) {
    return 0;
  }
  if (cap < total) {
    return 0;
  }
  size_t p = feetech::kOffsetParam0;
  buf[feetech::kOffsetCmd] = feetech::kInstSyncWrite;
  buf[p++] = address;
  buf[p++] = kSyncWriteDataLen;
  for (size_t i = 0; i < count; ++i) {
    buf[p++] = ids[i];
    SplitWord(values[i], &buf[p], feetech::kEndianness);
    p += kSyncWriteDataLen;
  }
  return Finalize(buf, feetech::kIdBroadcast,
                  static_cast<uint8_t>(kLengthFieldOverhead + param_size), total);
}

size_t StatusPacketSize(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len < feetech::kOffsetLength + 1) {
    return 0;
  }
  if (buf[0] != feetech::kHeader0 || buf[1] != feetech::kHeader1) {
    return 0;
  }
  return PacketSize(buf);
}

bool ParseStatus(const uint8_t* buf, size_t len, Status* out) {
  if (buf == nullptr || out == nullptr) {
    return false;
  }
  if (len < feetech::kMinPacketSize) {
    return false;
  }
  if (buf[0] != feetech::kHeader0 || buf[1] != feetech::kHeader1) {
    return false;
  }
  if (!IsValidId(buf[feetech::kOffsetId])) {
    return false;
  }
  const size_t size = PacketSize(buf);
  if (size == 0 || len < size) {
    return false;
  }
  if (ComputeChecksum(buf + feetech::kOffsetId,
                      size - 1 - feetech::kOffsetId) != buf[size - 1]) {
    return false;
  }
  out->id = buf[feetech::kOffsetId];
  out->error = buf[feetech::kOffsetCmd];
  out->data_length = static_cast<uint8_t>(size - feetech::kOffsetParam0 - 1);
  // データ 0 バイトの応答は data を指さない
  out->data = (out->data_length > 0) ? (buf + feetech::kOffsetParam0) : nullptr;
  return true;
}

const char* ErrorName(uint8_t error_bit) {
  if ((error_bit & feetech::kErrVoltage) != 0) {
    return "電圧異常";
  }
  if ((error_bit & feetech::kErrAngle) != 0) {
    return "角度超過";
  }
  if ((error_bit & feetech::kErrOverheat) != 0) {
    return "過熱";
  }
  if ((error_bit & feetech::kErrOverCurrent) != 0) {
    return "過電流";
  }
  if ((error_bit & feetech::kErrOverload) != 0) {
    return "オーバーロード";
  }
  return "不明";
}

void StatusParser::feed(const uint8_t* data, size_t len) {
  if (data == nullptr || len == 0) {
    return;
  }
  if (len >= kCapacity) {
    // 追加では収まらないので最新 kCapacity バイトだけ残す
    memcpy(buffer_, data + (len - kCapacity), kCapacity);
    buffer_length_ = kCapacity;
    return;
  }
  if (buffer_length_ + len > kCapacity) {
    // 容量超過。先頭から捨てる
    const size_t drop = buffer_length_ + len - kCapacity;
    memmove(buffer_, buffer_ + drop, buffer_length_ - drop);
    buffer_length_ -= drop;
  }
  memcpy(buffer_ + buffer_length_, data, len);
  buffer_length_ += len;
}

size_t StatusParser::drain(Status* out, size_t max_items) {
  if (out == nullptr) {
    return 0;
  }
  size_t count = 0;
  size_t pos = 0;
  while (count < max_items) {
    size_t head = pos;
    while (head + 1 < buffer_length_ &&
           !(buffer_[head] == feetech::kHeader0 &&
             buffer_[head + 1] == feetech::kHeader1)) {
      ++head;
    }
    if (head + 1 >= buffer_length_) {
      // フレームの頭がない。末尾が 0xFF なら次の feed と繋がりうるので残す
      if (buffer_length_ > 0 &&
          buffer_[buffer_length_ - 1] == feetech::kHeader0) {
        pos = buffer_length_ - 1;
      } else {
        pos = buffer_length_;
      }
      break;
    }
    // 間にあったノイズを捨ててヘッダへジャンプする（ノイズ自体は数えない）
    pos = head;

    if (buffer_length_ - pos < feetech::kMinPacketSize) {
      break;  // 短すぎ。保持して次の feed を待つ
    }
    const size_t size = PacketSize(&buffer_[pos]);
    if (size == 0) {
      // length が範囲外 → 破棄したフレーム候補として数える
      ++dropped_;
      ++pos;
      continue;
    }
    if (buffer_length_ - pos < size) {
      break;  // 未達。保持して次の feed を待つ
    }
    Status status;
    if (!ParseStatus(&buffer_[pos], size, &status)) {
      ++dropped_;
      pos += size;
      continue;
    }
    // Status::data は内部バッファを指す。呼び出し側でコピーすること
    out[count++] = status;
    pos += size;
  }
  if (pos > 0) {
    memmove(buffer_, buffer_ + pos, buffer_length_ - pos);
    buffer_length_ -= pos;
  }
  return count;
}

void StatusParser::clear() {
  buffer_length_ = 0;  // dropped_ は健全性の記録なので残す
}

}  // namespace bus
}  // namespace meister
