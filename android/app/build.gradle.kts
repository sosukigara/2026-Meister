plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
}

android {
    namespace = "com.meister.vision"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.meister.vision"
        // OPPO A5 2020 は Android 10 想定。実機の「設定→端末情報」で確定する (Q31 宿題)。
        minSdk = 29
        targetSdk = 35
        versionCode = 7
        versionName = "0.2.6"
        // 実機 (Snapdragon 665 = arm64) のみ。onnxruntime の全 ABI 同梱を避けて APK を軽くする。
        ndk {
            abiFilters += "arm64-v8a"
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions {
        jvmTarget = "17"
    }
    buildFeatures {
        viewBinding = true
    }
}

dependencies {
    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.appcompat)
    implementation(libs.androidx.activity.ktx)
    implementation(libs.androidx.lifecycle.runtime)
    implementation(libs.camerax.core)
    implementation(libs.camerax.camera2)
    implementation(libs.camerax.lifecycle)
    implementation(libs.camerax.view)
    // full 版 (標準 onnx 可)。mobile 版は ORT_INVALID_ARGUMENT で読めない。API は AAR の javap で確認済み。
    implementation(libs.onnxruntime.android)
    testImplementation(libs.junit)
}
