val libVersion: String by rootProject.extra

plugins {
    id("org.jetbrains.kotlin.jvm") version "2.1.10"

    `java-library`
    `java-test-fixtures`

    id("org.jetbrains.dokka") version "1.9.20"

    id("org.jlleitschuh.gradle.ktlint") version "12.1.2"

    `maven-publish`
}

repositories {
    mavenCentral()
}

dependencies {
    api(kotlin("stdlib-jdk8"))

    testImplementation("org.jetbrains.kotlin:kotlin-test")

    testImplementation("org.knowm.xchart:xchart:3.8.1")

    dokkaHtmlPlugin("org.jetbrains.dokka:mathjax-plugin:1.9.20")
}

java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
    withSourcesJar()
}

kotlin {
    compilerOptions {
        // Pin to 17: Android Studio's bundled JBR is often JDK 21.
        jvmTarget.set(org.jetbrains.kotlin.gradle.dsl.JvmTarget.JVM_17)
        // compile interface default methods (e.g., the velocity-aware AccelConstraint method) to JVM default
        // methods so that Java implementations inherit them
        freeCompilerArgs.add("-Xjvm-default=all")
    }
}

tasks.named<Test>("test") {
    useJUnitPlatform()
}

publishing {
    publications {
        create<MavenPublication>("maven") {
            groupId = "com.acmerobotics.roadrunner"
            artifactId = "core"
            version = libVersion

            from(components["java"])
        }
    }
}
