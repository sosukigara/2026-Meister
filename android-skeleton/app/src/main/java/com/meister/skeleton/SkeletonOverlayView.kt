package com.meister.skeleton

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import com.google.mediapipe.tasks.vision.handlandmarker.HandLandmarker
import kotlin.math.max

// 解析画像座標の HandResult をプレビュー上に拡大描画する。
// PreviewView (FILL) と同じ centerCrop 換算にしないと骨格がずれる。
class SkeletonOverlayView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
) : View(context, attrs) {

    private var items: List<HandResult> = emptyList()
    private var frameW = 1f
    private var frameH = 1f
    private var mirror = false

    private val linePaint = Paint().apply {
        color = Color.GREEN
        style = Paint.Style.STROKE
        strokeWidth = 9f
    }
    private val dotPaint = Paint().apply {
        color = Color.YELLOW
        style = Paint.Style.FILL
    }
    private val textPaint = Paint().apply {
        color = Color.GREEN
        textSize = 48f
        // 手の上に重ねても読めるよう縁取りする。
        setShadowLayer(6f, 0f, 0f, Color.BLACK)
    }

    fun setHands(hands: List<HandResult>, imageWidth: Int, imageHeight: Int) {
        items = hands
        frameW = imageWidth.toFloat().coerceAtLeast(1f)
        frameH = imageHeight.toFloat().coerceAtLeast(1f)
        postInvalidate()
    }

    fun setMirror(mirror: Boolean) {
        this.mirror = mirror
        postInvalidate()
    }

    private fun mapX(x: Float, scale: Float, dx: Float): Float {
        val px = dx + x * scale
        return if (mirror) width - px else px
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val scale = max(width / frameW, height / frameH)
        val dx = (width - frameW * scale) / 2f
        val dy = (height - frameH * scale) / 2f
        for (hand in items) {
            val pts = hand.points
            // 骨格線は MediaPipe 既定の接続を使う (自前 table を持たない)。
            for (c in HandLandmarker.HAND_CONNECTIONS) {
                val a = pts.getOrNull(c.start()) ?: continue
                val b = pts.getOrNull(c.end()) ?: continue
                canvas.drawLine(
                    mapX(a.x, scale, dx), dy + a.y * scale,
                    mapX(b.x, scale, dx), dy + b.y * scale,
                    linePaint,
                )
            }
            for (p in pts) {
                canvas.drawCircle(mapX(p.x, scale, dx), dy + p.y * scale, 11f, dotPaint)
            }
            val box = hand.bbox
            val lx = mapX(box.left / frameW, scale, dx)
            canvas.drawText(
                "${hand.janken.label} ${"%.2f".format(hand.score)}",
                lx,
                dy + (box.top / frameH) * scale - 8f,
                textPaint,
            )
        }
    }
}
