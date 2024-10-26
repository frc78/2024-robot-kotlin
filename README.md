# FRC 78 2024 Robot - Kotlin

This repo is a rewrite of the [2024 robot code](https://github.com/frc78/FRC78_Robot) completely in
kotlin.

## Setup

WPILib officially supports VSCode for FRC development, but IntelliJ is a much better IDE for Kotlin.

To get started, download [IntelliJ Community Edition](https://www.jetbrains.com/idea/download/) from
JetBrains. Be sure to scroll down and install the Community edition, as it is free.

## Third-Party Dependencies

WPILib supports installing 3rd party dependencies via json files in the `vendordeps` directory.

To install a third-party dependency, use the gradle command
`./gradlew vendorDep --url=<url>`

## Structure

The project is structured as follows:

- `src/main/deploy`: Contains files that will be copied to the roboRIO when the code is deployed.
  Mainly used for autonomous path files.
- `src/main/kotlin/frc/team78`: Root package of the robot code. All robot code should be placed in
  this package or a subpackage.
- `src/main/kotlin/frc/team78/lib`: Contains utility classes and other code not specific to a single subsystem.
- `src/main/kotlin/frc/team78/subsystems`: Contains the subsystems for the robot. Each subsystem has its own package
  that contains the subsystem class and any other classes that are specific to that subsystem (such as
  simulation classes)
- `src/main/kotlin/frc/team78/commands`: Contains the commands for the robot that span multiple subsystems
- `Main.kt`: The main entry point for the robot code. This file should usually be left alone, though in the past we have
  modified this class to support running different code on a test chassis
- `Robot.kt`: The main robot class. This class is responsible for initializing the subsystems and commands and
  running the robot code. This class should be kept as small as possible, with most of the code being placed in
  subsystems and commands.

## Subsystem Objects

Because subsystems represent physical components of the robot, of which there exists only one, they should be
represented as kotlin objects, rather than
a class.
This is because kotlin objects are singletons, meaning that there is only one instance of the object in the entire
program.

Constants for a subsystem should be declared inside the object

## Commands

Commands should be created using the factory methods provided by `SubsystemBase` whenever possible. This keeps the
commands inside the subsystem they control, as well as keeping code small.

Remember that a new command must be created any time it is bound to a trigger, so it is recommend to use the `command`
delegate to help ensure this functionality. The delegate is provided by the `CommandDelegate` class in the
`frc.team78.commands` package

```kotlin
val action by command { startEnd({}, {}) }
```

## RobotContainer

In previous years, `RobotContainer` was a class that was responsible for initializing the subsystems and commands. It's
generally considered to be an unnecessary abstraction, especially by WPILib developers, and there's no reason to not
just declare
all of your commands within the `Robot` object.
Similarly, there is no `robotInit` method, since the constructor is just as good for initializing the robot.