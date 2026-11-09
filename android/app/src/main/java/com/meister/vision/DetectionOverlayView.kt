package com.meister.vision

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View

// 解析画像座標の Detection をプレビュー上に拡大描画する。
class DetectionOverlayView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    private var items: List<Detection> = emptyList()
    private var frameW = 1f
    private var frameH = 1f

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

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val sx = width / frameW
        val sy = height / frameH
        for (d in items) {
            canvas.drawRect(d.x1 * sx, d.y1 * sy, d.x2 * sx, d.y2 * sy, boxPaint)
            canvas.drawText("${d.className} ${"%.2f".format(d.score)}", d.x1 * sx, d.y1 * sy - 8f, textPaint)
        }
    }
}
