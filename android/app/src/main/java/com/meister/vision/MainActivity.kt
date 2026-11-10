package com.meister.vision

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.widget.SeekBar
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
// toRgbBitmap は同パッケージ (ImageUtil.kt) の拡張関数。
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import com.meister.vision.databinding.ActivityMainBinding
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

// v1 基盤: プレビュー + Detector 推論 + 枠描画 + FPS/ms/件数 HUD。
// 通信なし。モデル未配置なら FakeDetector で配線確認できる。
class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private var detector: Detector = FakeDetector()
    private var lensFacing = CameraSelector.LENS_FACING_BACK
    private val counter = FpsCounter()
    private val analysisExecutor = Executors.newSingleThreadExecutor()
    @Volatile private var threshold = 0.25f

    private val permissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
            if (granted) startCamera()
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        detector = openDetector()

        binding.thresholdBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(bar: SeekBar?, value: Int, fromUser: Boolean) {
                threshold = value / 100f
                binding.thresholdLabel.text = "thr %.2f".format(threshold)
            }
            override fun onStartTrackingTouch(bar: SeekBar?) = Unit
            override fun onStopTrackingTouch(bar: SeekBar?) = Unit
        })
        binding.switchCameraButton.setOnClickListener {
            lensFacing = if (lensFacing == CameraSelector.LENS_FACING_BACK) {
                CameraSelector.LENS_FACING_FRONT
            } else {
                CameraSelector.LENS_FACING_BACK
            }
            startCamera()
        }

        if (checkSelfPermission(Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED
        ) {
            startCamera()
        } else {
            permissionLauncher.launch(Manifest.permission.CAMERA)
        }
    }

    private fun openDetector(): Detector {
        return try {
            YoloDetector(this)
        } catch (e: Exception) {
            FakeDetector()
        }
    }

    private fun startCamera() {
        val providerFuture = ProcessCameraProvider.getInstance(this)
        providerFuture.addListener({
            val provider = providerFuture.get()
            val preview = Preview.Builder().build().also {
                it.setSurfaceProvider(binding.previewView.surfaceProvider)
            }
            val selector = CameraSelector.Builder().requireLensFacing(lensFacing).build()
            // 最新フレーム優先で捨てる。溜めると遅延が発散する。
            val analysis = ImageAnalysis.Builder()
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .build()
                .also { it.setAnalyzer(analysisExecutor, ::analyze) }
            provider.unbindAll()
            // フロントカメラのプレビューは左右反転するので枠も反転させる。
            binding.overlay.setMirror(lensFacing == CameraSelector.LENS_FACING_FRONT)
            provider.bindToLifecycle(this, selector, preview, analysis)
        }, ContextCompat.getMainExecutor(this))
    }

    private fun analyze(image: ImageProxy) {
        try {
            val bitmap = image.toRgbBitmap()
            try {
                // letterbox は Detector 側 (YoloDetector) がやる。ここでは縮小しない。
                val start = System.nanoTime()
                val results = detector.detect(bitmap, threshold)
                val ms = (System.nanoTime() - start) / 1_000_000
                counter.tick(ms)
                runOnUiThread {
                    binding.overlay.setDetections(results, bitmap.width, bitmap.height)
                    binding.hudFps.text = "%.1f fps".format(counter.fps())
                    binding.hudMs.text = "%d ms".format(counter.lastInferenceMs())
                    binding.hudCount.text = "%d objs".format(results.size)
                }
            } finally {
                // フレーム毎の Bitmap は必ず捨てる。溜めると 10 分回した時に落ちる。
                bitmap.recycle()
            }
        } catch (e: Exception) {
            // 解析失敗は 1 フレーム捨てて継続する。落とさない。
        } finally {
            image.close()
        }
    }

    override fun onDestroy() {
        // 飛行中の analyze が session を触り終わってから閉じる。逆だと native 側で落ちる。
        analysisExecutor.shutdown()
        try {
            analysisExecutor.awaitTermination(5, TimeUnit.SECONDS)
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        }
        detector.close()
        super.onDestroy()
    }
}
