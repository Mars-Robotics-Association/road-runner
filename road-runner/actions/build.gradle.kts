import java.net.URI

val libVersion: String by rootProject.extra

plugins {
    id("org.jetbrains.kotlin.jvm") version "2.1.10"

    `java-library`

    id("org.jetbrains.dokka") version "1.9.20"

    id("org.jlleitschuh.gradle.ktlint") version "12.1.2"

    `maven-publish`
}

repositories {
    mavenCentral()
    maven { url = URI("https://maven.brott.dev/") }
}

dependencies {
    api(kotlin("stdlib-jdk8"))

    api(project(":core"))

    api("com.acmerobotics.dashboard:core:0.4.7")

    testImplementation("org.jetbrains.kotlin:kotlin-test")

    testImplementation(testFixtures(project(":core")))
}

kotlin {
    compilerOptions {
        // Pin to 17: Android Studio's bundled JBR is often JDK 21.
        jvmTarget.set(org.jetbrains.kotlin.gradle.dsl.JvmTarget.JVM_17)
        freeCompilerArgs.add("-Xjvm-default=all")
    }
}

java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
    withSourcesJar()
}

tasks.named<Test>("test") {
    useJUnitPlatform()
}

publishing {
    publications {
        create<MavenPublication>("maven") {
            groupId = "com.acmerobotics.roadrunner"
            artifactId = "actions"
            version = libVersion

            from(components["java"])
        }
    }
}
