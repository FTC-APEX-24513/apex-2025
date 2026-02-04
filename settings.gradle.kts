pluginManagement {
    repositories {
        google {
            content {
                includeGroupByRegex("com\\.android.*")
                includeGroupByRegex("com\\.google.*")
                includeGroupByRegex("androidx.*")
            }
        }
        mavenCentral()
        gradlePluginPortal()
        maven { url = uri("https://repo.dairy.foundation/releases") }
    }
}
plugins {
    id("org.gradle.toolchains.foojay-resolver-convention") version "0.8.0"
}
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
        maven { url = uri("https://maven.pedropathing.com/") }
        maven { url = uri("https://repo.dairy.foundation/releases") }
//        maven { url = uri("https://mymaven.bylazar.com/releases") }
    }
}

rootProject.name = "apex-2025"
include(":TeamCode")
include(":FtcRobotController")
