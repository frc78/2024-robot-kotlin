plugins {
    kotlin("jvm")
    id("edu.wpi.first.GradleRIO")("2024.3.2")
}

group = "frc.team78"
version = "unspecified"

repositories {
    mavenCentral()
}

dependencies {
    testImplementation(kotlin("test"))
}

tasks.test {
    useJUnitPlatform()
}