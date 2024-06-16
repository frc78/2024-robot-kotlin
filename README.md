# FRC 78 2024 Robot - Kotlin

This repo is a rewrite of the [2024 robot code](https://github.com/frc78/FRC78_Robot) completely in
kotlin.

## Setup

WPILib officially supports VSCode for FRC development, but IntelliJ is a much better IDE for Kotlin.

To get started, download [IntelliJ Community Edition](https://www.jetbrains.com/idea/download/) from
JetBrains. Be sure to scroll down and install the Community edition, as it is free.

## Third-Party Dependencies

WPILib supports installing 3rd party dependencies via json files in the `vendordeps` directory.
Using VSCode, you can install these dependencies by running
the `WPILib: Install Third-Party Libraries` command. This command doesn't exist in IntelliJ, so the
preferred method is to open the project in VSCode, install the dependencies, and then open the
project in IntelliJ. Alternatively, you can manually install the dependencies by downloading the
JSON file and placing it in the `vendordeps` directory.

## Structure

The project is structured as follows:

- `src/main/deploy`: Contains files that will be copied to the roboRIO when the code is deployed.
  Mainly used for autonomous path files.
- `src/main/kotlin/frc/team78`: Root package of the robot code. All robot code should be placed in
  this package or a subpackage.
- `src/main/kotlin/frc/team78/y2024`: Contains the robot code for the 2024 robot. This is where the
  majority of the code will be placed. Subsequent years will have their own packages (e.g. `y2025`).
- `src/main/kotlin/frc/team78/lib`: Contains utility classes and other code that is not specific to
  the robot. This code should be reusable across multiple years
- `y20xx/subsystems`: Contains the subsystems for the robot. Each subsystem has its own package that
  contains the subsystem class and any other classes that are specific to that subsystem (such as
  simulation classes)
- `y20xx/lib`: Contains utility classes that are specific to that year (such as field element
  locations). These classes should not be reused across multiple years.
- 