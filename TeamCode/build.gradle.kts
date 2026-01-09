import java.util.regex.Pattern

plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
    alias(libs.plugins.ksp)
    alias(libs.plugins.dairy.sloth.load)
}

repositories {
}

android {
    namespace = "org.firstinspires.ftc.teamcode"

    compileSdk = 36

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
        applicationId = "com.qualcomm.ftcrobotcontroller"
        minSdk = 24
        //noinspection ExpiredTargetSdkVersion
        targetSdk = 28

        /**
         * We keep the versionCode and versionName of robot controller applications in sync with
         * the master information published in the AndroidManifest.xml file of the FtcRobotController
         * module. This helps avoid confusion that might arise from having multiple versions of
         * a robot controller app simultaneously installed on a robot controller device.
         *
         * We accomplish this with the help of a funky little Groovy script that maintains that
         * correspondence automatically.
         *
         * @see <a href="http://developer.android.com/tools/building/configuring-gradle.html">Configure Your Build</a>
         * @see <a href="http://developer.android.com/tools/publishing/versioning.html">Versioning Your App</a>
         */
        val manifestFile = project(":FtcRobotController").file("src/main/AndroidManifest.xml")
        val manifestText = manifestFile.readText()
        //
        val vCodePattern = Pattern.compile("versionCode=\"(\\d+(\\.\\d+)*)\"")
        var matcher = vCodePattern.matcher(manifestText)
        matcher.find()
        val vCode = Integer.parseInt(matcher.group(1))
        //
        val vNamePattern = Pattern.compile("versionName=\"(.*)\"")
        matcher = vNamePattern.matcher(manifestText);
        matcher.find()
        val vName = matcher.group(1)
        //
        versionCode = vCode
        versionName = vName
    }

    // http://google.github.io/android-gradle-dsl/current/com.android.build.gradle.internal.dsl.BuildType.html
    buildTypes {
        release {
            signingConfig = signingConfigs.getByName("release")
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
        isCoreLibraryDesugaringEnabled = true
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }

    ndkVersion = "21.3.6528147"

    packagingOptions.jniLibs.useLegacyPackaging = true
    packagingOptions.jniLibs.pickFirsts.add("**/*.so")
}

kotlin {
    compilerOptions {
        jvmTarget.set(org.jetbrains.kotlin.gradle.dsl.JvmTarget.JVM_11)
    }
}

dependencies {
    // Android
    coreLibraryDesugaring(libs.desugar.jdk.libs)
    implementation(libs.androidx.appcompat)
    implementation(libs.androidx.core.ktx)

    // Dairy (FrozenMilk)
    implementation(libs.dairy.mercurial)

    // FTC
    annotationProcessor(files("lib/OpModeAnnotationProcessor.jar"))
    implementation(project(":FtcRobotController"))
    implementation(libs.ftc.blocks)
    implementation(libs.ftc.common)
    implementation(libs.ftc.hardware)
    implementation(libs.ftc.inspection)
    implementation(libs.ftc.onbotjava)
    implementation(libs.ftc.robotcore)
    implementation(libs.ftc.robotserver)
    implementation(libs.ftc.vision)

    // Kotlin
    implementation(libs.kotlin.inject.runtime)
    ksp(libs.kotlin.inject.compiler)

    // Pedro Pathing
    implementation(libs.pedro.pathing)
    implementation(libs.pedro.telemetry)

    // Testing
    androidTestImplementation(libs.junit.ext)
    testImplementation(libs.junit)

    // FTC Dashboard
    implementation(libs.ftc.dashboard)
}
