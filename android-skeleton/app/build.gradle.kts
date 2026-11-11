plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
}

android {
    namespace = "com.meister.skeleton"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.meister.skeleton"
        // OPPO A5 2020 は Android 10 想定。既存 android/ と同じ。
        minSdk = 29
        targetSdk = 35
        versionCode = 1
        versionName = "0.1.0"
        // 実機 (Snapdragon 665 = arm64) のみ。
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
    implementation(libs.tasks.vision)
    testImplementation(libs.junit)
}
