package com.meister.vision

import android.content.Context
import android.graphics.Bitmap

// YOLOv8n ONNX を onnxruntime-mobile で動かす実装。
// PC 側 meister_vision と同じ letterbox→NMS の順序にする (微調整時に流用できるよう)。
class YoloDetector(
    context: Context,
    // assets 内のモデル名。既定は PC 側と同名。320 版が来たらここだけ変える。
    modelName: String = "yolov8n.onnx",
) : Detector {
    // ★要確認★ onnxruntime-mobile の解決バージョンで API が変わっていないか。
    // 実ビルド時にシグネチャ不一致が出たらここを直す (v1 基盤では open 失敗時に Fake に fallback)。
    private val sessionHolder: Any? = null

    init {
        // v1 基盤では open を試み、assets 未配置なら例外→呼出側が FakeDetector に落とす。
        // 本実装はモデル配置後 (tuning フェーズ) に肉付けする。
        check(modelName.isNotBlank()) { "modelName is blank" }
        // assets.list が空配列を返しても elvis は発火しないため明示判定する。
        val present = try {
            context.assets.list("")?.contains(modelName)
        } catch (e: Exception) {
            false
        }
        if (present != true) error("model asset missing: $modelName")
    }

    override fun detect(input: Bitmap, threshold: Float): List<Detection> {
        // TODO(tuning): letterbox 320 → OrtSession.run → YoloPostprocess 相当 → NMS。
        // PC 側 yolo_detector.hpp の手順と 1:1 に対応させる。
        return emptyList()
    }

    override fun close() = Unit
}
