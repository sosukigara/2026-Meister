package com.meister.vision

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.util.Log
import android.util.Size
import android.widget.SeekBar
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
// toRgbBitmap は同パッケージ (ImageUtil.kt) の拡張関数。
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import com.meister.vision.databinding.ActivityMainBinding
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

// v1 基盤: プレビュー + Detector 推論 + 枠描画 + FPS/ms/件数 HUD。
// 通信なし。モデル未配置なら FakeDetector で配線確認できる。
class MainActivity : AppCompatActivity() {

    companion object {
        // ADB 取得用タグ。`adb logcat -d MeisterVision:I *:S` で抜く。
        const val LOG_TAG = "MeisterVision"
        // HUD 要約ログの間隔。1Hz で十分かつ安い。
        const val LOG_INTERVAL_NS = 1_000_000_000L
        // 前処理は letterbox 320 用。フル解像度で受けると縮小が無駄に重い。
        const val ANALYSIS_WIDTH = 640
        const val ANALYSIS_HEIGHT = 480
    }

    private lateinit var binding: ActivityMainBinding
    // 準備できるまで null。その間 analyze はフレームを捨てる (プレビューは動く)。
    @Volatile private var detector: Detector? = null
    // どれが動いているか (YOLO/FAKE) と fallback 理由。謎検出の切り分け用。
    private var detectorStatus = "loading..."
    private var lensFacing = CameraSelector.LENS_FACING_BACK
    private val counter = FpsCounter()
    private val cpu = CpuMonitor()
    private val analysisExecutor = Executors.newSingleThreadExecutor()
    // 幻の person 報告があるため端末既定は 0.5 に上げた (PC 側は 0.25 のまま)。スライダーで変えられる。
    @Volatile private var threshold = 0.5f
    private var lastLogNs = 0L
    @Volatile private var lastCpu = Float.NaN
    @Volatile private var lastLatMs = 0L
    private var errorCount = 0

    private val permissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
            if (granted) startCamera()
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        binding.hudDet.text = detectorStatus
        // 重い初期化+疎通確認は裏でやる。表でやると低端末で ANR になる。
        analysisExecutor.execute { initDetector() }

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

    private fun initDetector() {
        val d = try {
            YoloDetector(this).also {
                it.validate()
                detectorStatus = it.label()
            }
        } catch (e: Exception) {
            detectorStatus = "FAKE (${e.message})"
            FakeDetector()
        }
        detector = d
        Log.i(LOG_TAG, "detector ready: $detectorStatus")
        runOnUiThread { binding.hudDet.text = detectorStatus }
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
                .setResolutionSelector(
                    ResolutionSelector.Builder().setResolutionStrategy(
                        ResolutionStrategy(
                            Size(ANALYSIS_WIDTH, ANALYSIS_HEIGHT),
                            ResolutionStrategy.FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER,
                        ),
                    ).build(),
                )
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
        // 受信→推論完了の端to端。
        val t0 = System.nanoTime()
        try {
            // 準備前は捨てる。Detector が null のまま推論はしない。
            val active = detector ?: return
            val tJpeg0 = System.nanoTime()
            val bitmap = image.toRgbBitmap()
            val tJpeg = (System.nanoTime() - tJpeg0) / 1_000_000
            try {
                // letterbox は Detector 側 (YoloDetector) がやる。ここでは縮小しない。
                val results = active.detect(bitmap, threshold)
                val ms = (System.nanoTime() - t0) / 1_000_000
                lastLatMs = ms
                counter.tick(ms)
                val yolo = active as? YoloDetector
                val tPre = yolo?.lastPreMs ?: -1
                val tInfer = yolo?.lastInferMs ?: -1
                // ADB デバッグ用に 1Hz で要約を出す。画面 HUD と同じ値。
                val now = System.nanoTime()
                if (now - lastLogNs >= LOG_INTERVAL_NS) {
                    lastLogNs = now
                    lastCpu = cpu.sample()
                    val top = results.firstOrNull()
                    val topStr = if (top != null) {
                        " top=${top.className} ${"%.2f".format(top.score)}" +
                            " [${top.x1.toInt()},${top.y1.toInt()},${top.x2.toInt()},${top.y2.toInt()}]"
                    } else {
                        " top=none"
                    }
                    Log.i(
                        LOG_TAG,
                        "det=$detectorStatus fps=${"%.1f".format(counter.fps())}" +
                            " ms=${counter.lastInferenceMs()}" +
                            " jpeg=$tJpeg pre=$tPre infer=$tInfer" +
                            " lat=$lastLatMs" +
                            " cpu=${if (lastCpu.isNaN()) "--" else "%.1f".format(lastCpu)}" +
                            " objs=${results.size}$topStr thr=$threshold",
                    )
                }
                runOnUiThread {
                    binding.overlay.setDetections(results, bitmap.width, bitmap.height)
                    binding.hudFps.text = "%.1f fps".format(counter.fps())
                    binding.hudMs.text = "%d ms".format(counter.lastInferenceMs())
                    binding.hudCount.text = "%d objs".format(results.size)
                    binding.hudCpu.text = if (lastCpu.isNaN()) "-- cpu" else "%.1f%% cpu".format(lastCpu)
                    binding.hudLat.text = "%d ms lat".format(lastLatMs)
                }
            } finally {
                // フレーム毎の Bitmap は必ず捨てる。溜めると 10 分回した時に落ちる。
                bitmap.recycle()
            }
        } catch (e: Exception) {
            // 解析失敗は 1 フレーム捨てて継続する。落とさない。
            // 連続失敗の切り分け用に初回と30回毎だけ出す。
            errorCount++
            if (errorCount == 1 || errorCount % 30 == 0) {
                Log.e(LOG_TAG, "analyze failed x$errorCount", e)
            }
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
        detector?.close()
        super.onDestroy()
    }
}
