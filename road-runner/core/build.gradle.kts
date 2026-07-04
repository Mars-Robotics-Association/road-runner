val libVersion: String by rootProject.extra

plugins {
    id("org.jetbrains.kotlin.jvm") version "1.9.0"

    `java-library`
    `java-test-fixtures`

    id("org.jetbrains.dokka") version "1.9.10"

    id("org.jlleitschuh.gradle.ktlint") version "10.2.1"
    id("org.jlleitschuh.gradle.ktlint-idea") version "10.2.1"

    `maven-publish`
}

repositories {
    mavenCentral()
}

dependencies {
    api(kotlin("stdlib-jdk8"))

    testImplementation("org.jetbrains.kotlin:kotlin-test")

    testImplementation("org.knowm.xchart:xchart:3.8.1")

    dokkaHtmlPlugin("org.jetbrains.dokka:mathjax-plugin:1.9.10")
}

java {
    withSourcesJar()
}

tasks.withType<org.jetbrains.kotlin.gradle.tasks.KotlinCompile> {
    kotlinOptions {
        // compile interface default methods (e.g., the velocity-aware AccelConstraint method) to JVM default
        // methods so that Java implementations inherit them
        freeCompilerArgs += "-Xjvm-default=all"
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
