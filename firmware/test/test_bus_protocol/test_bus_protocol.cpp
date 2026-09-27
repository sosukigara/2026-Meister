/*
 * test_bus_protocol.cpp — STS/SCS バス codec のホスト側ユニットテスト（Unity）
 *
 * 実行: `pio test -e native`（firmware/ ディレクトリ内で）
 *
 * 期待バイト列は Feetech 公式 Python SDK `feetech-servo-sdk` 1.0.0 を
 * 実際に走らせて採った golden。手で算出した値ではない。
 * WRITE には length バイトが無い点、length フィールドは
 * instruction + parameters + checksum の点数である点に注意。
 */
#include <unity.h>

#include <string.h>

#include "hal/bus_protocol.h"
#include "hal/feetech_sts_registers.h"

using namespace meister;
using meister::bus::Status;
using meister::bus::StatusParser;
using meister::feetech::Endianness;

namespace {

// 応答の golden（チェックサムを含む実バイト列）
const uint8_t kStatusA[7] = {0xFF, 0xFF, 0x05, 0x03, 0x08, 0x5A, 0x95};
const uint8_t kStatusB[6] = {0xFF, 0xFF, 0x06, 0x02, 0x00, 0xF7};

}  // namespace

// ---------------------------------------------------------------------------
// 送信パケット（SDK 出力との 1 バイトずつの一致）
// ---------------------------------------------------------------------------

void test_ping_matches_sdk_golden(void) {
  uint8_t buf[250];
  const size_t n = bus::BuildPing(buf, sizeof(buf), 0x01);
  const uint8_t expected[6] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
  TEST_ASSERT_EQUAL_UINT(6, n);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buf, 6);
}

void test_read_packet_layout(void) {
  uint8_t buf[250];
  const uint8_t expected1[8] = {0xFF, 0xFF, 0x05, 0x04, 0x02, 0x37, 0x01, 0xBC};
  const size_t n1 = bus::BuildRead(buf, sizeof(buf), 0x05, 55, 1);
  TEST_ASSERT_EQUAL_UINT(8, n1);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected1, buf, 8);

  const uint8_t expected2[8] = {0xFF, 0xFF, 0x05, 0x04, 0x02, 0x37, 0x02, 0xBB};
  const size_t n2 = bus::BuildRead(buf, sizeof(buf), 0x05, 55, 2);
  TEST_ASSERT_EQUAL_UINT(8, n2);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected2, buf, 8);
}

void test_write_packet_layout(void) {
  uint8_t buf[250];

  // 1 バイト書き込み。length フィールドは 4（length バイトが無い）
  const uint8_t data1[1] = {0x00};
  const uint8_t expected1[8] = {0xFF, 0xFF, 0x05, 0x04, 0x03, 0x2A, 0x00, 0xC9};
  const size_t n1 =
      bus::BuildWrite(buf, sizeof(buf), 0x05, 42, data1, 1, feetech::kEndianness);
  TEST_ASSERT_EQUAL_UINT(8, n1);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected1, buf, 8);

  // 2 バイト書き込み
  const uint8_t data2[2] = {0xE8, 0x03};
  const uint8_t expected2[9] = {0xFF, 0xFF, 0x05, 0x05, 0x03, 0x2A, 0xE8, 0x03, 0xDD};
  const size_t n2 =
      bus::BuildWrite(buf, sizeof(buf), 0x05, 42, data2, 2, feetech::kEndianness);
  TEST_ASSERT_EQUAL_UINT(9, n2);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected2, buf, 9);
}

void test_write_big_endian_swaps_words(void) {
  // ビッグエンディ指定は 16 ビット語ごとにバイトを交換する
  uint8_t buf[250];
  const uint8_t host_order[2] = {0x03, 0xE8};
  const uint8_t expected[9] = {0xFF, 0xFF, 0x05, 0x05, 0x03, 0x2A, 0xE8, 0x03, 0xDD};
  const size_t n =
      bus::BuildWrite(buf, sizeof(buf), 0x05, 42, host_order, 2, Endianness::kBig);
  TEST_ASSERT_EQUAL_UINT(9, n);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buf, 9);
}

void test_sync_write_uses_broadcast_id(void) {
  uint8_t buf[250];
  const uint8_t ids[2] = {0x05, 0x06};
  const uint16_t values[2] = {0x03E8, 0x0000};
  const uint8_t expected[14] = {0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x2A, 0x02,
                                0x05, 0xE8, 0x03, 0x06, 0x00, 0x00, 0x52};
  const size_t n = bus::BuildSyncWrite(buf, sizeof(buf), 42, ids, values, 2);
  TEST_ASSERT_EQUAL_UINT(14, n);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buf, 14);
  // 一斉送信 ID であること
  TEST_ASSERT_EQUAL_UINT8(0xFE, buf[2]);
  // 値がリトルエンディアンで並ぶこと
  TEST_ASSERT_EQUAL_UINT8(0xE8, buf[8]);
  TEST_ASSERT_EQUAL_UINT8(0x03, buf[9]);
}

// ---------------------------------------------------------------------------
// チェックサム
// ---------------------------------------------------------------------------

void test_checksum_is_bitwise_not_of_sum(void) {
  // ~(0x01 + 0x02 + 0x01) & 0xFF = 0xFB。XOR ではない
  const uint8_t body[3] = {0x01, 0x02, 0x01};
  TEST_ASSERT_EQUAL_UINT8(0xFB, bus::ComputeChecksum(body, sizeof(body)));

  // 単純和が 0xFF のとき bit 否定は 0x00
  const uint8_t sum_ff[3] = {0xFF, 0xFF, 0x01};
  TEST_ASSERT_EQUAL_UINT8(0x00, bus::ComputeChecksum(sum_ff, sizeof(sum_ff)));

  // 検証側の不変条件: ID から最終バイトの直前までの ~sum が末尾バイトになる
  uint8_t buf[250];
  const size_t n = bus::BuildPing(buf, sizeof(buf), 0x01);
  TEST_ASSERT_EQUAL_UINT(6, n);
  TEST_ASSERT_EQUAL_UINT8(buf[n - 1], bus::ComputeChecksum(buf + 2, n - 1 - 2));
  // 全 6 バイトを 1 度に通すと ~sum になるので 0xFF にはならない（XOR でもない）
  TEST_ASSERT_EQUAL_UINT8(0x02, bus::ComputeChecksum(buf, n));
}

// ---------------------------------------------------------------------------
// 応答パケット
// ---------------------------------------------------------------------------

void test_status_packet_size(void) {
  TEST_ASSERT_EQUAL_UINT(7, bus::StatusPacketSize(kStatusA, sizeof(kStatusA)));
  TEST_ASSERT_EQUAL_UINT(6, bus::StatusPacketSize(kStatusB, sizeof(kStatusB)));
  const uint8_t bad[7] = {0x00, 0xFF, 0x05, 0x03, 0x08, 0x5A, 0x95};
  TEST_ASSERT_EQUAL_UINT(0, bus::StatusPacketSize(bad, sizeof(bad)));
  // length が大きすぎるフレームは不正
  uint8_t too_long[7];
  memcpy(too_long, kStatusA, sizeof(too_long));
  too_long[3] = 0xFF;
  TEST_ASSERT_EQUAL_UINT(0, bus::StatusPacketSize(too_long, sizeof(too_long)));
}

void test_parse_status_returns_error_and_data(void) {
  Status st;
  TEST_ASSERT_TRUE(bus::ParseStatus(kStatusA, sizeof(kStatusA), &st));
  TEST_ASSERT_EQUAL_UINT8(0x05, st.id);
  TEST_ASSERT_EQUAL_UINT8(0x08, st.error);  // 過電流
  TEST_ASSERT_TRUE(st.has_error());
  TEST_ASSERT_EQUAL_UINT8(1, st.data_length);
  TEST_ASSERT_EQUAL_UINT8(0x5A, st.data[0]);

  // データ 0 バイトの応答
  TEST_ASSERT_TRUE(bus::ParseStatus(kStatusB, sizeof(kStatusB), &st));
  TEST_ASSERT_EQUAL_UINT8(0x06, st.id);
  TEST_ASSERT_FALSE(st.has_error());
  TEST_ASSERT_EQUAL_UINT8(0, st.data_length);
  TEST_ASSERT_NULL(st.data);
}

void test_reject_wrong_header(void) {
  uint8_t bad[7];
  memcpy(bad, kStatusA, sizeof(bad));
  bad[0] = 0x00;
  Status st;
  TEST_ASSERT_FALSE(bus::ParseStatus(bad, sizeof(bad), &st));
  bad[0] = 0xFF;
  bad[1] = 0x00;
  TEST_ASSERT_FALSE(bus::ParseStatus(bad, sizeof(bad), &st));
}

void test_reject_bad_checksum(void) {
  uint8_t bad[7];
  memcpy(bad, kStatusA, sizeof(bad));
  bad[6] ^= 0xFF;
  Status st;
  TEST_ASSERT_FALSE(bus::ParseStatus(bad, sizeof(bad), &st));
}

void test_reject_truncated(void) {
  Status st;
  TEST_ASSERT_FALSE(bus::ParseStatus(kStatusA, 1, &st));
  TEST_ASSERT_FALSE(bus::ParseStatus(kStatusA, 6, &st));  // 最終バイト未達
}

void test_reject_out_of_range_id(void) {
  uint8_t bad[7];
  Status st;
  memcpy(bad, kStatusA, sizeof(bad));
  bad[2] = 0x00;  // ID 0 は範囲外
  bad[6] = bus::ComputeChecksum(bad + 2, 4);
  TEST_ASSERT_FALSE(bus::ParseStatus(bad, sizeof(bad), &st));
  memcpy(bad, kStatusA, sizeof(bad));
  bad[2] = 0xFE;  // 一斉送信 ID は応答しない
  bad[6] = bus::ComputeChecksum(bad + 2, 4);
  TEST_ASSERT_FALSE(bus::ParseStatus(bad, sizeof(bad), &st));
}

void test_error_name(void) {
  TEST_ASSERT_EQUAL_STRING("不明", bus::ErrorName(0x00));
  TEST_ASSERT_EQUAL_STRING("電圧異常", bus::ErrorName(0x01));
  TEST_ASSERT_EQUAL_STRING("角度超過", bus::ErrorName(0x02));
  TEST_ASSERT_EQUAL_STRING("過熱", bus::ErrorName(0x04));
  TEST_ASSERT_EQUAL_STRING("過電流", bus::ErrorName(0x08));
  TEST_ASSERT_EQUAL_STRING("オーバーロード", bus::ErrorName(0x20));
  // 複数ビットは最初に一致したもの
  TEST_ASSERT_EQUAL_STRING("電圧異常", bus::ErrorName(0x2F));
  // 未定義ビットだけなら不明
  TEST_ASSERT_EQUAL_STRING("不明", bus::ErrorName(0x10));
}

// ---------------------------------------------------------------------------
// ストリーム parser
// ---------------------------------------------------------------------------

void test_status_parser_resyncs_after_garbage(void) {
  // フレームの頭を持たないノイズ 3 バイトは無視される
  const uint8_t noise3[3] = {0x11, 0x22, 0x33};
  uint8_t stream[3 + 7 + 6];
  memcpy(stream, noise3, sizeof(noise3));
  memcpy(stream + 3, kStatusA, sizeof(kStatusA));
  memcpy(stream + 10, kStatusB, sizeof(kStatusB));

  StatusParser parser;
  parser.feed(stream, sizeof(stream));
  Status out[4];
  const size_t n = parser.drain(out, 4);
  TEST_ASSERT_EQUAL_UINT(2, n);
  TEST_ASSERT_EQUAL_UINT8(0x05, out[0].id);
  TEST_ASSERT_EQUAL_UINT8(0x06, out[1].id);
  TEST_ASSERT_EQUAL_UINT(0, parser.dropped());
  TEST_ASSERT_EQUAL_UINT(0, parser.pending());

  // フレームの頭に見えるノイズ 5 バイトは破棄した候補として 1 件に数える
  const uint8_t noise5[5] = {0xFF, 0xFF, 0x00, 0x00, 0x00};
  uint8_t stream2[5 + 7 + 6];
  memcpy(stream2, noise5, sizeof(noise5));
  memcpy(stream2 + 5, kStatusA, sizeof(kStatusA));
  memcpy(stream2 + 12, kStatusB, sizeof(kStatusB));

  StatusParser parser2;
  parser2.feed(stream2, sizeof(stream2));
  Status out2[4];
  const size_t n2 = parser2.drain(out2, 4);
  TEST_ASSERT_EQUAL_UINT(2, n2);
  TEST_ASSERT_EQUAL_UINT(1, parser2.dropped());
}

void test_status_parser_counts_corrupt_frame(void) {
  uint8_t stream[7 + 6];
  memcpy(stream, kStatusA, sizeof(kStatusA));
  stream[6] ^= 0xFF;  // チェックサム破損
  memcpy(stream + 7, kStatusB, sizeof(kStatusB));

  StatusParser parser;
  parser.feed(stream, sizeof(stream));
  Status out[4];
  TEST_ASSERT_EQUAL_UINT(1, parser.drain(out, 4));
  TEST_ASSERT_EQUAL_UINT(1, parser.dropped());
  TEST_ASSERT_EQUAL_UINT8(0x06, out[0].id);
}

void test_status_parser_keeps_partial_frame(void) {
  // フレームが途中で切れても捨てない
  StatusParser parser;
  parser.feed(kStatusA, 4);
  Status out[4];
  TEST_ASSERT_EQUAL_UINT(0, parser.drain(out, 4));
  TEST_ASSERT_EQUAL_UINT(4, parser.pending());
  TEST_ASSERT_EQUAL_UINT(0, parser.dropped());

  parser.feed(kStatusA + 4, 3);
  TEST_ASSERT_EQUAL_UINT(1, parser.drain(out, 4));
  TEST_ASSERT_EQUAL_UINT(0, parser.pending());
  TEST_ASSERT_EQUAL_UINT8(0x08, out[0].error);
}

void test_status_parser_respects_max_items(void) {
  uint8_t stream[7 + 6];
  memcpy(stream, kStatusA, sizeof(kStatusA));
  memcpy(stream + 7, kStatusB, sizeof(kStatusB));

  StatusParser parser;
  parser.feed(stream, sizeof(stream));
  Status out[4];
  TEST_ASSERT_EQUAL_UINT(1, parser.drain(out, 1));
  TEST_ASSERT_EQUAL_UINT(6, parser.pending());  // 2 つ目の応答だけが残る
  TEST_ASSERT_EQUAL_UINT(1, parser.drain(out, 1));
  TEST_ASSERT_EQUAL_UINT(0, parser.pending());
}

void test_status_parser_data_points_into_internal_buffer(void) {
  // Status::data は内部バッファを指す。次の feed で上書きされる
  StatusParser parser;
  parser.feed(kStatusA, sizeof(kStatusA));
  Status first[1];
  TEST_ASSERT_EQUAL_UINT(1, parser.drain(first, 1));
  const uint8_t* first_data = first[0].data;
  TEST_ASSERT_EQUAL_UINT8(0x5A, first_data[0]);

  parser.feed(kStatusB, sizeof(kStatusB));
  Status second[1];
  TEST_ASSERT_EQUAL_UINT(1, parser.drain(second, 1));
  // 1 つ前の Status::data は 2 つ目の応答の最終バイトで書き換えられている
  TEST_ASSERT_EQUAL_UINT8(0xF7, first_data[0]);
}

void test_status_parser_clear_keeps_dropped(void) {
  uint8_t stream[7 + 6];
  memcpy(stream, kStatusA, sizeof(kStatusA));
  stream[6] ^= 0xFF;
  memcpy(stream + 7, kStatusB, sizeof(kStatusB));

  StatusParser parser;
  parser.feed(stream, sizeof(stream));
  Status out[4];
  TEST_ASSERT_EQUAL_UINT(1, parser.drain(out, 4));
  const uint32_t dropped = parser.dropped();
  TEST_ASSERT_TRUE(dropped > 0);

  parser.feed(kStatusA, 3);  // 途中まで溜める
  parser.clear();
  TEST_ASSERT_EQUAL_UINT(0, parser.pending());
  TEST_ASSERT_EQUAL_UINT(dropped, parser.dropped());
}

void test_status_parser_feed_overflow_drops_from_front(void) {
  const size_t kCap = 2 * feetech::kMaxPacketSize;
  uint8_t noise[600];
  memset(noise, 0x11, sizeof(noise));
  StatusParser parser;
  parser.feed(noise, sizeof(noise));
  TEST_ASSERT_EQUAL_UINT(kCap, parser.pending());
  Status out[4];
  TEST_ASSERT_EQUAL_UINT(0, parser.drain(out, 4));
  TEST_ASSERT_EQUAL_UINT(0, parser.pending());
}

// ---------------------------------------------------------------------------
// 組み立ての拒否条件
// ---------------------------------------------------------------------------

void test_build_rejects_bad_id_and_small_buffer(void) {
  uint8_t buf[250];
  const uint8_t data[2] = {0xE8, 0x03};
  const uint8_t ids[1] = {0x05};
  const uint16_t values[1] = {0x03E8};

  // ID 0 と 0xFF は範囲外
  TEST_ASSERT_EQUAL_UINT(0, bus::BuildPing(buf, sizeof(buf), 0x00));
  TEST_ASSERT_EQUAL_UINT(0, bus::BuildPing(buf, sizeof(buf), 0xFF));
  TEST_ASSERT_EQUAL_UINT(0, bus::BuildRead(buf, sizeof(buf), 0x00, 55, 1));
  TEST_ASSERT_EQUAL_UINT(0, bus::BuildRead(buf, sizeof(buf), 0xFF, 55, 1));
  TEST_ASSERT_EQUAL_UINT(0, bus::BuildWrite(buf, sizeof(buf), 0x00, 42, data, 2,
                                            feetech::kEndianness));
  TEST_ASSERT_EQUAL_UINT(0, bus::BuildWrite(buf, sizeof(buf), 0xFF, 42, data, 2,
                                            feetech::kEndianness));

  // バッファが 1 バイト足りない
  TEST_ASSERT_EQUAL_UINT(0, bus::BuildPing(buf, 5, 0x01));
  TEST_ASSERT_EQUAL_UINT(0, bus::BuildRead(buf, 7, 0x05, 55, 1));
  TEST_ASSERT_EQUAL_UINT(0, bus::BuildWrite(buf, 8, 0x05, 42, data, 2,
                                            feetech::kEndianness));
  TEST_ASSERT_EQUAL_UINT(0, bus::BuildSyncWrite(buf, 10, 42, ids, values, 1));
  // 1 バイト足りていれば通る
  TEST_ASSERT_EQUAL_UINT(6, bus::BuildPing(buf, 6, 0x01));
  TEST_ASSERT_EQUAL_UINT(11, bus::BuildSyncWrite(buf, 11, 42, ids, values, 1));

  // データ無しの WRITE と 0 件の同期書き込みは 0
  TEST_ASSERT_EQUAL_UINT(0, bus::BuildWrite(buf, sizeof(buf), 0x05, 42, nullptr,
                                            2, feetech::kEndianness));
  TEST_ASSERT_EQUAL_UINT(0, bus::BuildSyncWrite(buf, sizeof(buf), 42, ids, values, 0));
}

// ---------------------------------------------------------------------------
// バイト順と角度変換
// ---------------------------------------------------------------------------

void test_split_join_word_roundtrip(void) {
  const uint16_t values[5] = {0x0000, 0x0001, 0x00FF, 0x1234, 0xFFFF};
  uint8_t bytes[2];
  for (size_t i = 0; i < 5; ++i) {
    bus::SplitWord(values[i], bytes, Endianness::kLittle);
    TEST_ASSERT_EQUAL_UINT16(values[i], bus::JoinWord(bytes, Endianness::kLittle));
    bus::SplitWord(values[i], bytes, Endianness::kBig);
    TEST_ASSERT_EQUAL_UINT16(values[i], bus::JoinWord(bytes, Endianness::kBig));
  }
  // 分割の並びが逆であること
  bus::SplitWord(0x1234, bytes, Endianness::kLittle);
  TEST_ASSERT_EQUAL_UINT8(0x34, bytes[0]);
  TEST_ASSERT_EQUAL_UINT8(0x12, bytes[1]);
  bus::SplitWord(0x1234, bytes, Endianness::kBig);
  TEST_ASSERT_EQUAL_UINT8(0x12, bytes[0]);
  TEST_ASSERT_EQUAL_UINT8(0x34, bytes[1]);
}

void test_degrees_steps_roundtrip(void) {
  // kStepsPerRev は «未確認» 値なので具体的な角度は固定しない
  const uint16_t steps[5] = {0, 1, 1024, 2048, 4095};
  for (size_t i = 0; i < 5; ++i) {
    const float deg = feetech::StepsToDegrees(steps[i]);
    const uint16_t back = feetech::DegreesToSteps(deg);
    const int diff = static_cast<int>(back) - static_cast<int>(steps[i]);
    TEST_ASSERT_TRUE(diff <= 1 && diff >= -1);
  }
  // 範囲外は丸める
  TEST_ASSERT_EQUAL_UINT16(0, feetech::DegreesToSteps(-10.0f));
  TEST_ASSERT_EQUAL_UINT16(feetech::reg::kStepsPerRev - 1,
                           feetech::DegreesToSteps(1000.0f));
}

// ---------------------------------------------------------------------------

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_ping_matches_sdk_golden);
  RUN_TEST(test_read_packet_layout);
  RUN_TEST(test_write_packet_layout);
  RUN_TEST(test_write_big_endian_swaps_words);
  RUN_TEST(test_sync_write_uses_broadcast_id);
  RUN_TEST(test_checksum_is_bitwise_not_of_sum);

  RUN_TEST(test_status_packet_size);
  RUN_TEST(test_parse_status_returns_error_and_data);
  RUN_TEST(test_reject_wrong_header);
  RUN_TEST(test_reject_bad_checksum);
  RUN_TEST(test_reject_truncated);
  RUN_TEST(test_reject_out_of_range_id);
  RUN_TEST(test_error_name);

  RUN_TEST(test_status_parser_resyncs_after_garbage);
  RUN_TEST(test_status_parser_counts_corrupt_frame);
  RUN_TEST(test_status_parser_keeps_partial_frame);
  RUN_TEST(test_status_parser_respects_max_items);
  RUN_TEST(test_status_parser_data_points_into_internal_buffer);
  RUN_TEST(test_status_parser_clear_keeps_dropped);
  RUN_TEST(test_status_parser_feed_overflow_drops_from_front);

  RUN_TEST(test_build_rejects_bad_id_and_small_buffer);
  RUN_TEST(test_split_join_word_roundtrip);
  RUN_TEST(test_degrees_steps_roundtrip);

  return UNITY_END();
}
