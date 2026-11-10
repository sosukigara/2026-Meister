package com.meister.vision

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.ImageFormat
import android.graphics.Matrix
import android.graphics.Rect
import android.graphics.YuvImage
import androidx.camera.core.ImageProxy
import java.io.ByteArrayOutputStream

// ImageProxy (YUV_420_888) → RGB Bitmap。回転も補正する。
fun ImageProxy.toRgbBitmap(): Bitmap {
    val yuvImage = toYuvImage()
    val out = ByteArrayOutputStream()
    yuvImage.compressToJpeg(Rect(0, 0, width, height), 100, out)
    val bytes = out.toByteArray()
    // 壊れたフレームで null が返る。呼び出し側が catch して 1 フレーム捨てる。
    val raw = BitmapFactory.decodeByteArray(bytes, 0, bytes.size)
        ?: throw IllegalStateException("jpeg decode failed")
    val rotation = imageInfo.rotationDegrees
    if (rotation == 0) return raw
    val matrix = Matrix().apply { postRotate(rotation.toFloat()) }
    return Bitmap.createBitmap(raw, 0, 0, raw.width, raw.height, matrix, true)
}

// rowStride / pixelStride が詰まっていない端末もあるため 1px ずつ詰める。
private fun ImageProxy.toYuvImage(): YuvImage {
    val yPlane = planes[0]
    val uPlane = planes[1]
    val vPlane = planes[2]
    val ySize = width * height
    val nv21 = ByteArray(ySize + width * height / 2)

    val yBuffer = yPlane.buffer
    for (row in 0 until height) {
        yBuffer.position(row * yPlane.rowStride)
        yBuffer.get(nv21, row * width, width)
    }

    val uBuffer = uPlane.buffer
    val vBuffer = vPlane.buffer
    var pos = ySize
    for (row in 0 until height / 2) {
        for (col in 0 until width / 2) {
            val v = vBuffer.get(row * vPlane.rowStride + col * vPlane.pixelStride)
            val u = uBuffer.get(row * uPlane.rowStride + col * uPlane.pixelStride)
            nv21[pos++] = v
            nv21[pos++] = u
        }
    }
    return YuvImage(nv21, ImageFormat.NV21, width, height, null)
}
