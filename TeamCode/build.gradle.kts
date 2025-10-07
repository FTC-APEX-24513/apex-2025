plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
}

android {
    namespace = "edu.exeter.apex.ftc.teamcode"
    //noinspection GradleDependency
    compileSdk = 30

    signingConfigs {
        create("release") {
            val apkStoreFile = System.getenv("APK_SIGNING_STORE_FILE")
            if (apkStoreFile != null) {
                keyAlias = System.getenv("APK_SIGNING_KEY_ALIAS")
                keyPassword = System.getenv("APK_SIGNING_KEY_PASSWORD")
                storeFile = file(System.getenv("APK_SIGNING_STORE_FILE"))
                storePassword = System.getenv("APK_SIGNING_STORE_PASSWORD")
            } else {
                keyAlias = "androiddebugkey"
                keyPassword = "android"
                storeFile = rootProject.file("libs/ftc.debug.keystore")
                storePassword = "android"
            }
        }

        getByName("debug") {
            keyAlias = "androiddebugkey"
            keyPassword = "android"
            storeFile = rootProject.file("libs/ftc.debug.keystore")
            storePassword = "android"
        }
    }

    defaultConfig {
        signingConfig = signingConfigs.getByName("debug")

        applicationId = "edu.exeter.apex.ftc.teamcode"
        minSdk = 24
        //noinspection ExpiredTargetSdkVersion
        targetSdk = 28
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            signingConfig = signingConfigs.getByName("release")
            isMinifyEnabled = false
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
            ndk {
                abiFilters.addAll(listOf("armeabi-v7a", "arm64-v8a"))
            }
        }
        debug {
            isDebuggable = true
            isJniDebuggable = true
            ndk {
                abiFilters.addAll(listOf("armeabi-v7a", "arm64-v8a"))
            }
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
    kotlinOptions {
        jvmTarget = "11"
    }
    packaging {
        jniLibs {
            pickFirsts.add("**/*.so")
            useLegacyPackaging = true
        }
    }
}

dependencies {

    implementation(libs.core.ktx)
    implementation(libs.androidx.appcompat)
    testImplementation(libs.junit)
    androidTestImplementation(libs.ext.junit)

    implementation(project(":FtcRobotController"))

    implementation(libs.ftc.inspection)
    implementation(libs.ftc.blocks)
    implementation(libs.ftc.robotcore)
    implementation(libs.ftc.robotserver)
    implementation(libs.ftc.onbotjava)
    implementation(libs.ftc.hardware)
    implementation(libs.ftc.ftccommon)
    implementation(libs.ftc.vision)

    implementation(libs.pedro.pathing.ftc)
    implementation(libs.pedro.pathing.telemetry)
    implementation(libs.fullpanels)
    implementation(libs.dairy.core)
    implementation(libs.dairy.mercurial)
}