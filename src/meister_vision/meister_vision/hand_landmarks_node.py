"""ROS2 手 keypoint 推定ノード。

画像トピック (既定: /camera/image_raw) を購読し、手の keypoint を
geometry_msgs/PoseArray で配信する。レート制限と購読者ゼロ時の短絡は
detection_node と同じ方針。

手という概念は、yolov8n.onnx (COCO 80 クラス) からは出せないため、別に
配置された手 keypoint モデルだけを読む。

publish 型を geometry_msgs/PoseArray にした理由
----------------------------------------------
vision_msgs 4.1.1 (この環境の /opt/ros/jazzy) には Keypoint2DArray が
無い (BoundingBox2DArray / Detection2DArray / Classification のみ)。
keypoint に合致する既存型が無いので、型に正確なものを作ると interface
パッケージの新規追加になり、setup.py / package.xml の改変も必要になる。
どちらもこの作業のスコープ外なので、既存型で表現する。

  採った型が持つ性質:
    - Header が残るので stamp / frame_id (camera_link) が使えて、入力
      画像との時間同期に使える。
    - poses が空 = 「手なし」という一意な意味を持てる。ND の曖昧な
      センチネル値を用意しなくて済む。
  失うもの:
    - landmark ごとの score が配線できない。下流が信頼度を得るには
      復元の手間が要る。誤読される値を載せるより欠落を選ぶ判断をした
      (confidence は推定器内にあるが、既存型に載る場所がない)。
    - orientation が未使用。恒等クォータニオンを明示的に入れる。

推定器は差し込み可 (detector 引数) なので、モデルファイルが無い
テスト環境でもノードの配線だけを検証できる。

パラメータ:
  model_path         (str,   既定: "")     手 keypoint モデルパス
  image_topic        (str,   既定: "/camera/image_raw")
  score_threshold    (float, 既定: 0.2)    手とみなす最小スコア
  rate               (float, 既定: 3.0)    最大処理レート [Hz]
  inference_threads  (int,   既定: 2)      ONNX 推論スレッド数
  hand_timeout_sec   (float, 既定: 0.5)    手なしを確定するまでの猶予 [秒]

トピック:
  購読:  image_topic      (sensor_msgs/Image)
  配信:  hand_landmarks   (geometry_msgs/PoseArray)

手なしの配信契約
--------------
検出が外れたフレームでは何も配信しない。直前の検出から
hand_timeout_sec を超えても手が写っていなければ、poses が空の配列を
1 度だけ配信して、以降は無音にする。

  - 毎フレーム空配列を流さない理由: 購読側は「手が見えない」と「手が
    写っているのに keypoint が 1 つも無い」を区別できない。配信件数が
    カメラレートに比例して増え、件数を数える購読側はそれを「手が消えた」
    の合図として扱わざるを得なくなる。1 度だけ流せば合図として一意に
    読める。
  - 一度も手が見えていない間は無音の理由: 捨てるべき古い状態が無い。
  - 空配列を 2 度目以降送らない理由: 1 度目で購読側は既に「手なし」を
    認識済みなので、2 度目以降では状態が変わらない。
  - 猶予 hand_timeout_sec の既定 0.5 秒の根拠: 手首搭載カメラは
    5-10 Hz で動く。猶予は「処理済みのフレーム」に対するもので、許容
    する連続脱落フレーム数は hand_timeout_sec × rate になるため 0.5 秒
    は 2.5-5 フレーム分に相当する。この程度の脱落は手元の影やアームの
    振動で普通に起きるため、「手が映っていない」は猶予が尽きた後に
    だけ言う。

モデルが無い場合は __init__ が FileNotFoundError を投げ、main() はそれを
握り潰さずプロセスを落とす。「手なし」を無期限に待ち続けると上流が異常を
取り逃すため、起動時に判明する問題を起動時に出す。
"""
from __future__ import annotations

import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg

# エンコーディング変換は detection_node と同一に保つ。二重に持つと
# yuv422_yuy2 や numpy 2 の cv_bridge 非対応対応の片方だけが直され、
# 二つのノードで同じ画像に違う結果が出る (R2)。
from .detection_node import _bgr_from_image_msg
from .hand_landmark_detector import HandLandmarkDetector, HandLandmarks


class HandLandmarksNode(Node):
    """画像を購読して手の keypoint を配信するノード。

    Args:
        detector: 差し込む推定器。None なら自身のパラメータから構築する。
            モデルが無い環境でノードの配線だけを検証するために使う。
    """

    def __init__(self, detector: Optional[HandLandmarkDetector] = None) -> None:
        super().__init__("hand_landmarks")

        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("score_threshold", 0.2)
        self.declare_parameter("rate", 3.0)
        self.declare_parameter("inference_threads", 2)
        self.declare_parameter("hand_timeout_sec", 0.5)

        model_path = self.get_parameter(
            "model_path").get_parameter_value().string_value or None
        image_topic = self.get_parameter("image_topic").value
        score_threshold = float(self.get_parameter("score_threshold").value)
        rate = float(self.get_parameter("rate").value)
        self._min_interval = 1.0 / rate if rate > 0.0 else 0.0
        self._hand_timeout_sec = float(
            self.get_parameter("hand_timeout_sec").value)

        if detector is None:
            self._detector = HandLandmarkDetector(
                model_path=model_path,
                score_threshold=score_threshold,
                intra_op_num_threads=int(
                    self.get_parameter("inference_threads").value),
            )
            self.get_logger().info(
                f"手 keypoint モデルを読み込みました: {self._detector.model_path}")
        else:
            self._detector = detector
            self.get_logger().info(
                "外部から渡された推定器を使用します (モデル未読み込み)")

        self._pub = self.create_publisher(PoseArray, "hand_landmarks", 10)
        self._sub_image = self.create_subscription(
            ImageMsg, image_topic, self._image_callback, 10)
        self.get_logger().info(
            f"画像トピック '{image_topic}' を購読開始 "
            f"(最大 {rate} Hz, 手なし猶予 {self._hand_timeout_sec} 秒)")

        self._last_process_time = 0.0
        self._process_count = 0
        # 検出時刻は処理完了時刻で持つ。処理中の時刻だと、推論に時間のか
        # かったフレームで猶予を消費し、落ちかけを「消えた」と誤読する。
        self._last_detection_time: Optional[float] = None
        self._clear_published = False

    @staticmethod
    def _build_pose_array(
        msg: ImageMsg, result: Optional[HandLandmarks],
    ) -> PoseArray:
        """推定結果を PoseArray に変換する。

        result が None でも配列を返す。空を「手なし」の唯一の符号にできる
        のは、detect() が「K 点ある / 0 点」の 2 値しか返さないため。
        点数が減る途中状態が無いので、空が「壊れた検出」と読まれる余地
        が無い。

        orientation を恒等クォータニオンで埋めるのは、Pose の既定値が
        全ゼロでクォータニオンとしては「長さが 0 の不正な回転」を表すた
        め。有効な回転ではないクォータニオンを渡すと下流の変換が破綻する。
        """
        array = PoseArray()
        array.header = msg.header
        if result is None:
            return array
        for x, y in result.landmarks:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            array.poses.append(pose)
        return array

    def _clear_due(self, now: float) -> bool:
        """今のフレームで「手なし」を 1 度だけ通知してよいか。

        1 度だけ、という不変条件。満たすうちは「消えた」の合図を増やさない。
        """
        if self._last_detection_time is None:
            return False
        if self._clear_published:
            return False
        return now - self._last_detection_time >= self._hand_timeout_sec

    def _image_callback(self, msg: ImageMsg) -> None:
        if self._pub.get_subscription_count() == 0:
            return
        now = time.monotonic()
        if now - self._last_process_time < self._min_interval:
            return
        self._last_process_time = now
        self._process_count += 1

        try:
            bgr = _bgr_from_image_msg(msg)
            result = self._detector.detect(bgr)
        except Exception as exc:  # noqa: BLE001
            # 1 枚の変換失敗で購読を黙って落とさない。不正なエンコーディングも
            # ここで吸収し、ノードは動き続ける。
            self.get_logger().error(f"手 keypoint 推定に失敗しました: {exc}")
            return

        seen_at = time.monotonic()
        if result is None:
            # 契約 (a): 猶予が切れるまでは無音、通知済みなら二度と流さない。
            # 毎フレーム空配列を流すと「手が消えた」が頻度で曖昧になるため。
            if not self._clear_due(seen_at):
                return
            self._clear_published = True
            self._pub.publish(self._build_pose_array(msg, None))
            self.get_logger().debug(
                f"[{self._process_count}] 手なしを確定 (poses 0 で通知)")
            return

        self._last_detection_time = seen_at
        self._clear_published = False
        self._pub.publish(self._build_pose_array(msg, result))
        self.get_logger().debug(
            f"[{self._process_count}] {len(result.landmarks)} keypoint")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[HandLandmarksNode] = None
    try:
        node = HandLandmarksNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # モデル未配置で __init__ が失敗しても rclpy の文脈を片付ける
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
