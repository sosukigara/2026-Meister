"""Meister ESP32 シリアルブリッジノード.

ROS 2 の /cmd_vel (target velocity) を受信し、ステアリング舵角 (CMD_STEERING_ANGLE)
とモータ PWM 速度 (CMD_MOTOR_VELOCITY) フレームに変換して UART 経由で ESP32 へ
送る。ESP32 からの FB_STATE フィードバックを受信し /esp32/state として配信する。

コマンドが cmd_timeout 秒途絶えたら安全のため速度 0 を送る (ウォッチドッグ)。
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray

from meister_serial_bridge.kinematics import twist_to_actuators
from meister_serial_bridge.protocol import (
    TYPE_FB_ERROR,
    TYPE_FB_STATE,
    FrameParser,
    encode_motor_velocity,
    encode_steering_angle,
)

NUM_CHANNELS = 6


class SerialBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__('serial_bridge')

        # ---- パラメータ ----
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheelbase', 0.30)          # m (前後軸間距離の目安)
        self.declare_parameter('max_linear_vel', 1.0)      # m/s (vel=+1000 相当)
        self.declare_parameter('max_steering_deg', 30.0)   # 度 (±900 tenths 相当)
        self.declare_parameter('steer_invert', False)      # サーボ取り付け向き反転
        self.declare_parameter('drive_invert', False)      # モータ取り付け向き反転
        self.declare_parameter('cmd_timeout', 0.5)         # 秒 (途絶えたら停止)

        self._serial_port = self.get_parameter('serial_port').value
        self._baud = int(self.get_parameter('baud').value)
        self._wheelbase = float(self.get_parameter('wheelbase').value)
        self._max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self._max_steering_deg = float(self.get_parameter('max_steering_deg').value)
        self._steer_invert = bool(self.get_parameter('steer_invert').value)
        self._drive_invert = bool(self.get_parameter('drive_invert').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # ---- シリアルポート ----
        self._serial = None
        self._parser = FrameParser()
        self._open_serial()

        # ---- 状態 ----
        self._tx_lock = threading.Lock()
        self._last_cmd_time = self.get_clock().now()
        self._zero_sent = False
        self._rx_error_count = 0

        # ---- ROS I/F ----
        self._sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._state_pub = self.create_publisher(Int16MultiArray, '/esp32/state', 10)
        self._watchdog = self.create_timer(0.1, self._on_watchdog)
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info(
            f'serial bridge: port={self._serial_port} baud={self._baud} '
            f'wheelbase={self._wheelbase} max_vel={self._max_linear_vel} '
            f'max_steer={self._max_steering_deg}deg'
        )

    # ------------------------------------------------------------------
    # シリアル
    # ------------------------------------------------------------------
    def _open_serial(self) -> None:
        try:
            import serial  # pyserial
            # serial_for_url は 'loop://' 等のテスト用 URL も扱える
            self._serial = serial.serial_for_url(self._serial_port, self._baud, timeout=0.1)
            self.get_logger().info(f'opened {self._serial_port} @ {self._baud}')
        except Exception as exc:  # ポート未接続でも起動は継続 (リトライする)
            self._serial = None
            self.get_logger().warn(f'serial open failed ({exc}); retrying...')

    def _write_frames(self, frames: list[bytes]) -> None:
        if self._serial is None:
            self._open_serial()
            if self._serial is None:
                return
        try:
            with self._tx_lock:
                self._serial.write(b''.join(frames))
                self._serial.flush()
        except Exception as exc:
            self.get_logger().error(f'serial write failed: {exc}')
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    # ------------------------------------------------------------------
    # コールバック
    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        steer_tenths, vel_permille = twist_to_actuators(
            vx=msg.linear.x,
            wz=msg.angular.z,
            wheelbase=self._wheelbase,
            max_linear_vel=self._max_linear_vel,
            max_steering_deg=self._max_steering_deg,
            steer_invert=self._steer_invert,
            drive_invert=self._drive_invert,
        )
        self._write_frames([
            encode_steering_angle([steer_tenths] * NUM_CHANNELS),
            encode_motor_velocity([vel_permille] * NUM_CHANNELS),
        ])
        self._last_cmd_time = self.get_clock().now()
        self._zero_sent = False
        self.get_logger().debug(
            f'cmd_vel -> steer={steer_tenths / 10.0:.1f}deg vel={vel_permille / 10.0:.1f}%'
        )

    def _on_watchdog(self) -> None:
        """cmd_vel が途切れたら速度 0 を送る (1 回だけ)。"""
        if self._zero_sent:
            return
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed < self._cmd_timeout:
            return
        self._write_frames([
            encode_steering_angle([0] * NUM_CHANNELS),
            encode_motor_velocity([0] * NUM_CHANNELS),
        ])
        self._zero_sent = True
        self.get_logger().warn('cmd_vel timeout -> stop', throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # 受信ループ (FB_STATE / FB_ERROR)
    # ------------------------------------------------------------------
    def _rx_loop(self) -> None:
        while rclpy.ok():
            serial = self._serial
            if serial is None:
                self._open_serial()
                if self._serial is None:
                    import time as _time
                    _time.sleep(2.0)
                    continue
            try:
                data = serial.read(64)
            except Exception:
                import time as _time
                _time.sleep(0.5)
                continue
            if not data:
                continue
            for frame in self._parser.feed(data):
                self.get_logger().debug(
                    f'rx frame: type=0x{frame.type_id:02x} len={len(frame.payload)}')
                if frame.type_id == TYPE_FB_STATE:
                    enc = [frame.get_int16(i * 2) for i in range(6)]
                    state = frame.get_u8(12)
                    error_flags = frame.get_u8(13)
                    msg = Int16MultiArray()
                    msg.data = enc + [state, error_flags]
                    self._state_pub.publish(msg)
                elif frame.type_id == TYPE_FB_ERROR:
                    self._rx_error_count += 1
                    self.get_logger().warn(
                        f'ESP32 error frame: code={frame.payload[0]}',
                        throttle_duration_sec=5.0,
                    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了時に停止させてから閉じる
        try:
            node._write_frames([
                encode_steering_angle([0] * NUM_CHANNELS),
                encode_motor_velocity([0] * NUM_CHANNELS),
            ])
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
