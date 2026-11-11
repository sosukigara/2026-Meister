package com.meister.skeleton

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
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import com.meister.skeleton.databinding.ActivityMainBinding
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

// 骨格専用: プレビュー + 手骨格推論 + 骨格描画 + FPS/ms/件数/CPU/遅延 HUD。
// 通信なし。モデル未配置なら FakeSkeleton で配線確認できる。
class MainActivity : AppCompatActivity() {

    companion object {
        // ADB 取得用タグ。`adb logcat -d MeisterSkeleton:I *:S` で抜く。
        const val LOG_TAG = "MeisterSkeleton"
        // HUD 要約ログの間隔。1Hz で十分かつ安い。
        const val LOG_INTERVAL_NS = 1_000_000_000L
        // 25fps寄せの解析上限。画質優先で上げない。
        const val ANALYSIS_WIDTH = 640
        const val ANALYSIS_HEIGHT = 480
    }

    private lateinit var binding: ActivityMainBinding
    // 準備できるまで null。その間 analyze はフレームを捨てる (プレビューは動く)。
    @Volatile private var detector: HandLandmarkerDetector? = null
    @Volatile private var fake: FakeSkeleton? = null
    // どれが動いているかと fallback 理由。謎表示の切り分け用。
    private var detectorStatus = "loading..."
    private var lensFacing = CameraSelector.LENS_FACING_BACK
    private val counter = FpsCounter()
    private val cpu = CpuMonitor()
    private val analysisExecutor = Executors.newSingleThreadExecutor()
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
        // 重い初期化は裏でやる。表でやると低端末で ANR になる。
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
        try {
            val d = HandLandmarkerDetector(this)
            detector = d
            detectorStatus = d.label()
        } catch (e: Exception) {
            detectorStatus = "FAKE (${e.message})"
            fake = FakeSkeleton()
        }
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
            // フロントカメラのプレビューは左右反転するので骨格も反転させる。
            binding.overlay.setMirror(lensFacing == CameraSelector.LENS_FACING_FRONT)
            provider.bindToLifecycle(this, selector, preview, analysis)
        }, ContextCompat.getMainExecutor(this))
    }

    private fun analyze(image: ImageProxy) {
        // 受信→推論完了の端to端。25fps目標の証拠表示用。
        val t0 = System.nanoTime()
        try {
            // Bitmap経路。MediaImageBuilder は RGBA_8888 しか受けないため YUV 直結はできない。
            val bitmap = image.toRgbBitmap()
            try {
                val results: List<HandResult> = detector?.detectHands(
                    bitmap,
                    threshold,
                    image.imageInfo.timestamp / 1_000_000,
                )
                    ?: fake?.detectHands(bitmap.width, bitmap.height, threshold)
                    ?: return
                val ms = (System.nanoTime() - t0) / 1_000_000
                lastLatMs = ms
                counter.tick(ms)
                // ADB デバッグ用に 1Hz で要約を出す。画面 HUD と同じ値。
                val now = System.nanoTime()
                if (now - lastLogNs >= LOG_INTERVAL_NS) {
                    lastLogNs = now
                    lastCpu = cpu.sample()
                    val top = results.firstOrNull()
                    val topStr = if (top != null) {
                        " top=${top.janken.label} ${"%.2f".format(top.score)}"
                    } else {
                        " top=none"
                    }
                    Log.i(
                        LOG_TAG,
                        "det=$detectorStatus fps=${"%.1f".format(counter.fps())}" +
                            " ms=${counter.lastInferenceMs()}" +
                            " lat=$lastLatMs" +
                            " cpu=${if (lastCpu.isNaN()) "--" else "%.1f".format(lastCpu)}" +
                            " hands=${results.size}$topStr thr=$threshold",
                    )
                }
                runOnUiThread {
                    binding.overlay.setHands(results, bitmap.width, bitmap.height)
                    binding.hudFps.text = "%.1f fps".format(counter.fps())
                    binding.hudMs.text = "%d ms".format(counter.lastInferenceMs())
                    binding.hudCount.text = "%d hands".format(results.size)
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
