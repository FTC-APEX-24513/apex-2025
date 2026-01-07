import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

plugins {
    alias(libs.plugins.android.library)
    alias(libs.plugins.kotlin.android)
}

android {
    namespace = "com.qualcomm.ftcrobotcontroller"

    compileSdk = 36

    defaultConfig {
        minSdk = 24
        buildConfigField(
            "String", "APP_BUILD_TIME", '"' + (SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSSZ", Locale.ROOT).format(
                Date()
            )) + '"'
        )
    }

    lint {
        targetSdk = 28
    }

    testOptions {
        targetSdk = 28
    }

    buildFeatures {
        buildConfig = true
    }

    compileOptions {
        isCoreLibraryDesugaringEnabled = true
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
}

kotlin {
    compilerOptions {
        jvmTarget.set(org.jetbrains.kotlin.gradle.dsl.JvmTarget.JVM_11)
    }
}

dependencies {

    coreLibraryDesugaring(libs.desugar.jdk.libs)
    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.appcompat)
    testImplementation(libs.junit)
    androidTestImplementation(libs.junit.ext)

    implementation(libs.ftc.inspection)
    implementation(libs.ftc.blocks)
    implementation(libs.ftc.robotcore)
    implementation(libs.ftc.robotserver)
    implementation(libs.ftc.onbotjava)
    implementation(libs.ftc.hardware)
    implementation(libs.ftc.common)
    implementation(libs.ftc.vision)

}
