plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
    alias(libs.plugins.ksp)
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
        minSdkVersion(24)
        //noinspection ExpiredTargetSdkVersion
        targetSdkVersion(28)

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
//        val manifestFile = project(":FtcRobotController").file("src/main/AndroidManifest.xml");
//        val manifestText = manifestFile.getText()
//        //
//        val vCodePattern = Pattern.compile("versionCode=\"(\\d+(\\.\\d+)*)\"")
//        val matcher = vCodePattern.matcher(manifestText)
//        matcher.find()
//        val vCode = Integer.parseInt(matcher.group(1))
//        //
//        val vNamePattern = Pattern.compile("versionName=\"(.*)\"")
//        matcher = vNamePattern.matcher(manifestText);
//        matcher.find()
//        val vName = matcher.group(1)
//        //
//        versionCode = vCode
//        versionName = vName
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
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }

    kotlinOptions {
        jvmTarget = "11"
    }

    ndkVersion = "21.3.6528147"

    packagingOptions.jniLibs.useLegacyPackaging = true
    packagingOptions.jniLibs.pickFirsts.add("**/*.so")
}

dependencies {

    implementation(libs.core.ktx)
    implementation(libs.androidx.appcompat)
    testImplementation(libs.junit)
    androidTestImplementation(libs.ext.junit)

    implementation(project(":FtcRobotController"))
    annotationProcessor(files("lib/OpModeAnnotationProcessor.jar"))

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
    implementation(libs.bylazar.fullpanels)
    implementation(libs.dairy.mercurial)

    // kotlin-inject
    implementation(libs.kotlin.inject.runtime)
    ksp(libs.kotlin.inject.compiler)
}
