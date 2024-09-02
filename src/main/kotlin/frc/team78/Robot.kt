// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team78

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.net.PortForwarder
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team78.commands.VarFeedPrime
import frc.team78.commands.VarShootPrime
import frc.team78.lib.*
import frc.team78.subsystems.chassis.*
import frc.team78.subsystems.elevator.Elevator
import frc.team78.subsystems.feedback.LED
import frc.team78.subsystems.feeder.Feeder
import frc.team78.subsystems.intake.Intake
import frc.team78.subsystems.shooter.Shooter
import frc.team78.subsystems.wrist.Wrist
import java.util.*
import kotlin.jvm.optionals.getOrNull

object Robot : TimedRobot() {
    private var autonomousCommand: Command? = null

    private val driver =
        CommandXboxController(0).apply {
            rightBumper()
                .whileTrue(
                    intakeNote()
                        .deadlineWith(SwerveDrive.autoAlignToNote())
                        .withName("Auto Note Align")
                )

            SwerveDrive.defaultCommand =
                SwerveDrive.applyRequest {
                        SwerveDrive.driveRequest.apply {
                            VelocityX =
                                -leftY *
                                    SwerveDrive.MOTION_LIMITS.maxTranslationVelocity.`in`(
                                        MetersPerSecond
                                    )
                            VelocityY =
                                -leftX *
                                    SwerveDrive.MOTION_LIMITS.maxTranslationVelocity.`in`(
                                        MetersPerSecond
                                    )
                            RotationalRate =
                                -rightX *
                                    SwerveDrive.MOTION_LIMITS.maxAngularVelocity.`in`(
                                        RadiansPerSecond
                                    )
                        }
                    }
                    .withName("Drive Field Centric")

            rightBumper()
                .whileTrue(
                    intakeNote()
                        .deadlineWith(SwerveDrive.autoAlignToNote())
                        .withName("Auto Pickup Note")
                )
            leftBumper()
                .whileTrue(
                    SwerveDrive.applyRequest {
                        SwerveDrive.driveAtAngleRequest.apply {
                            VelocityX =
                                -leftY *
                                    SwerveDrive.MOTION_LIMITS.maxTranslationVelocity.`in`(
                                        MetersPerSecond
                                    )
                            VelocityY =
                                -leftX *
                                    SwerveDrive.MOTION_LIMITS.maxTranslationVelocity.`in`(
                                        MetersPerSecond
                                    )
                            TargetDirection =
                                (SPEAKER_POSE - SwerveDrive.estimatedPose.translation)
                                    .angle
                                    .rotateBy(Rotation2d.fromRadians(Math.PI))
                        }
                    }
                )

            povDown().whileTrue(SwerveDrive.applyRequest { SwerveRequest.SwerveDriveBrake() })
        }

    private val operator =
        CommandXboxController(1).apply {
            x().whileTrue(VarFeedPrime(Constants.SHOOTER_POSITION))

            leftTrigger()
                .whileTrue(Shooter.spinUp().alongWith(VarShootPrime()).withName("Feed"))
                .onFalse(Shooter.stop().alongWith(Wrist.stow()))

            y().whileTrue(Wrist.ampPosition().alongWith(Elevator.goToAmp()).withName("Amp"))
                .onFalse(Wrist.stow())

            a().whileTrue(Elevator.goToClimb())
            rightBumper().whileTrue(intakeNote())

            leftBumper().whileTrue(Feeder.eject())
            rightTrigger().whileTrue(Feeder.shoot())
        }

    // SysId Controller
    init {
        CommandXboxController(5).apply {
            a().whileTrue(Shooter.runSysId())
            b().whileTrue(SwerveDrive.translationSysId())
            x().whileTrue(Elevator.runSysId())
            y().whileTrue(Wrist.runSysId())
            leftBumper().whileTrue(SwerveDrive.rotationSysId())
            rightBumper().whileTrue(SwerveDrive.steerSysId())
        }
    }

    private fun intakeNote() =
        Feeder.intake().deadlineWith(Intake.intake(), Wrist.stow()).withName("Pick Up Note")

    init {
        CommandScheduler.getInstance().onCommandInitialize { command ->
            println("${command.name} initialized")
        }
        CommandScheduler.getInstance().onCommandFinish { command ->
            println("${command.name} finished")
        }

        /** We use extra controllers for running tests, but these aren't always plugged in */
        DriverStation.silenceJoystickConnectionWarning(true)
        SmartDashboard.putData(CommandScheduler.getInstance())

        PortForwarder.add(5801, "photonvision.local", 5801)

        if (isReal()) {
            DataLogManager.start()
            SignalLogger.setPath("/U/ctre-logs")
        }

        Trigger(Feeder::hasNote)
            .whileTrue(LED.indicateNoteInCartridge())
            .and(RobotModeTriggers.teleop())
            .onTrue(driver.shortRumble(RumbleType.kRightRumble))
            .onTrue(operator.shortRumble(RumbleType.kRightRumble))
            .onFalse(driver.shortRumble(RumbleType.kBothRumble))

        Trigger(Shooter::isAtSpeed)
            .whileTrue(LED.indicateShooterWheelsAtSpeed())
            .and(RobotModeTriggers.teleop())
            .onTrue(operator.shortRumble(RumbleType.kBothRumble))

        Trigger(Elevator::isAtGoal)
            .and(RobotModeTriggers.teleop())
            .onTrue(operator.shortRumble(RumbleType.kBothRumble))

        RobotModeTriggers.disabled().negate().and { !Elevator.zeroed }.onTrue(Elevator.zero())

        RobotModeTriggers.disabled()
            .and { DriverStation.isDSAttached() }
            .onTrue(LED.indicateDisabled())

        RobotModeTriggers.teleop().onTrue(Elevator.brake().alongWith(Wrist.brake()))

        RobotModeTriggers.disabled()
            .and { !DriverStation.isFMSAttached() }
            .onTrue(Elevator.coast().alongWith(Wrist.coast()))

        addPeriodic(PoseEstimator::update, 0.02, 0.015)

        SmartDashboard.putData(PowerDistribution(1, PowerDistribution.ModuleType.kRev))
    }

    // Autonomous Definitions
    private val autoChooser: SendableChooser<Command>

    init {
        PPHolonomicDriveController.setRotationTargetOverride {
            if (!Feeder.hasNote) {
                Optional.empty()
            } else {
                val angle =
                    (SPEAKER_POSE - SwerveDrive.estimatedPose.translation).angle +
                        Rotation2d.fromDegrees(180.0)
                Optional.of(angle)
            }
        }
        NamedCommands.registerCommands(
            mapOf(
                "Intake" to intakeNote(),
                "StopShooter" to Shooter.stop(),
                "StartShooter" to Shooter.spinUp(),
                "Score" to Commands.waitUntil { Shooter.isAtSpeed }.andThen(Feeder.shoot()),
                "stow" to Wrist.stow(),
                "Target" to
                    SwerveDrive.applyRequest {
                        SwerveDrive.driveAtAngleRequest.apply {
                            VelocityX = 0.0
                            VelocityY = 0.0
                            TargetDirection =
                                (SPEAKER_POSE - SwerveDrive.estimatedPose.translation).angle +
                                    Rotation2d.fromRadians(Math.PI)
                        }
                    },
                "DriveToNote" to
                    intakeNote()
                        .deadlineWith(SwerveDrive.autoAlignToNote())
                        .until {
                            when (DriverStation.getAlliance().getOrNull()) {
                                DriverStation.Alliance.Blue ->
                                    SwerveDrive.estimatedPose.translation.x > 8.25
                                else -> SwerveDrive.estimatedPose.translation.x < 8.25
                            }
                        }
                        .withName("Drive to Note"),
                "VariableShoot" to VarShootPrime(),
            )
        )
        autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Mode", autoChooser)
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun autonomousInit() {
        autonomousCommand = autoChooser.selected
        autonomousCommand?.schedule()
    }

    override fun teleopInit() {
        autonomousCommand?.cancel()
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    object Constants {

        val SHOOTER_POSITION = Translation2d(0.0, 0.56)
    }

    private fun CommandXboxController.shortRumble(type: RumbleType) =
        Commands.startEnd({ hid.setRumble(type, 1.0) }, { hid.setRumble(type, 0.0) })
            .withTimeout(0.3)
}
