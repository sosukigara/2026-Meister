package com.meister.vision

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import kotlin.math.max

// 解析画像座標の Detection をプレビュー上に拡大描画する。
// PreviewView (FILL) と同じ centerCrop 換算にしないと枠がずれる。
class DetectionOverlayView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    private var items: List<Detection> = emptyList()
    private var frameW = 1f
    private var frameH = 1f
    private var mirror = false

    private val boxPaint = Paint().apply {
        color = Color.GREEN
        style = Paint.Style.STROKE
        strokeWidth = 4f
    }
    private val textPaint = Paint().apply {
        color = Color.GREEN
        textSize = 42f
    }

    fun setDetections(detections: List<Detection>, imageWidth: Int, imageHeight: Int) {
        items = detections
        frameW = imageWidth.toFloat().coerceAtLeast(1f)
        frameH = imageHeight.toFloat().coerceAtLeast(1f)
        postInvalidate()
    }

    fun setMirror(mirror: Boolean) {
        this.mirror = mirror
        postInvalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val scale = max(width / frameW, height / frameH)
        val dx = (width - frameW * scale) / 2f
        val dy = (height - frameH * scale) / 2f
        for (d in items) {
            val left = dx + d.x1 * scale
            val right = dx + d.x2 * scale
            val top = dy + d.y1 * scale
            val bottom = dy + d.y2 * scale
            // フロントカメラは表示が鏡像なので左右を入れ替える。
            val l = if (mirror) width - right else left
            val r = if (mirror) width - left else right
            canvas.drawRect(l, top, r, bottom, boxPaint)
            val tx = if (mirror) width - right else left
            canvas.drawText("${d.className} ${"%.2f".format(d.score)}", tx, top - 8f, textPaint)
        }
    }
}
