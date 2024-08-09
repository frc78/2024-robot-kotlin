// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team78

import com.ctre.phoenix6.SignalLogger
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.net.PortForwarder
import edu.wpi.first.units.Units.Degrees
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
import frc.team78.lib.MotionLimits
import frc.team78.lib.SPEAKER_POSE
import frc.team78.lib.metersPerSecond
import frc.team78.lib.metersPerSecondPerSecond
import frc.team78.lib.radiansPerSecond
import frc.team78.lib.radiansPerSecondPerSecond
import frc.team78.subsystems.chassis.BaseSwerveDrive
import frc.team78.subsystems.chassis.Chassis
import frc.team78.subsystems.chassis.FieldOrientedDrive
import frc.team78.subsystems.chassis.FieldOrientedWithCardinal
import frc.team78.subsystems.chassis.PoseEstimator
import frc.team78.subsystems.elevator.Elevator
import frc.team78.subsystems.feedback.LED
import frc.team78.subsystems.feeder.Feeder
import frc.team78.subsystems.intake.Intake
import frc.team78.subsystems.shooter.Shooter
import frc.team78.subsystems.wrist.Wrist
import org.littletonrobotics.urcl.URCL
import java.util.*
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI

object Robot : TimedRobot() {
    private var autonomousCommand: Command? = null

    private val driver =
        CommandXboxController(0).apply {
            rightBumper()
                .whileTrue(
                    intakeNote().deadlineWith(Chassis.autoAlignToNote()).withName("Auto Note Align")
                )

            val baseDrive = BaseSwerveDrive(hid, Constants.MOTION_LIMITS)
            Chassis.defaultCommand = FieldOrientedDrive(baseDrive::chassisSpeeds)
            rightBumper()
                .whileTrue(
                    intakeNote()
                        .deadlineWith(Chassis.autoAlignToNote())
                        .withName("Auto Pickup Note")
                )
            leftBumper()
                .whileTrue(
                    FieldOrientedWithCardinal(
                        { (SPEAKER_POSE - PoseEstimator.pose.translation).angle.radians + PI },
                        baseDrive::chassisSpeeds,
                        Constants.ROTATION_PID,
                        Constants.ROTATION_CONSTRAINTS,
                        Degrees.zero()
                    )
                )

            povDown().whileTrue(Chassis.lockWheels())
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
            b().whileTrue(Chassis.runSysId())
            x().whileTrue(Elevator.runSysId())
            y().whileTrue(Wrist.runSysId())
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
            PowerDistribution(1, PowerDistribution.ModuleType.kRev)
            URCL.start(
                mapOf(
                    1 to "Front Left Drive",
                    2 to "Front Left Steer",
                    3 to "Front Right Drive",
                    4 to "Front Right Steer",
                    5 to "Back Left Drive",
                    6 to "Back Left Steer",
                    7 to "Back Right Drive",
                    8 to "Back Right Steer",
                    9 to "Intake Bottom",
                    10 to "Intake Top",
                    11 to "Elevator Left",
                    12 to "Elevator Right",
                    13 to "Wrist"
                )
            )

            SignalLogger.setPath("/U/ctre-logs")
            SignalLogger.start()
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

        RobotModeTriggers.teleop()
            .onTrue(Elevator.brake().alongWith(Wrist.brake(), Chassis.brake()))

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
                    (SPEAKER_POSE - PoseEstimator.pose.translation).angle +
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
                    FieldOrientedWithCardinal(
                        { (SPEAKER_POSE - PoseEstimator.pose.translation).angle.radians + PI },
                        { ChassisSpeeds(0.0, 0.0, 0.0) },
                        Constants.ROTATION_PID,
                        Constants.ROTATION_CONSTRAINTS,
                        Degrees.of(5.0)
                    ),
                "DriveToNote" to
                    intakeNote()
                        .deadlineWith(Chassis.autoAlignToNote())
                        .until {
                            when (DriverStation.getAlliance().getOrNull()) {
                                DriverStation.Alliance.Blue ->
                                    PoseEstimator.pose.translation.x > 8.25
                                else -> PoseEstimator.pose.translation.x < 8.25
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
        val MOTION_LIMITS =
            MotionLimits(
                5.6.metersPerSecond,
                12.radiansPerSecond,
                3.metersPerSecondPerSecond,
                18.radiansPerSecondPerSecond
            )
        val TRANSLATION_PID = PIDConstants(2.0, 0.0, 0.0)
        val ROTATION_PID = PIDConstants(4.0, 0.0, 0.085)
        val ROTATION_CONSTRAINTS =
            TrapezoidProfile.Constraints(
                MOTION_LIMITS.maxAngularVelocity,
                MOTION_LIMITS.maxAngularAcceleration
            )

        val SHOOTER_POSITION = Translation2d(0.0, 0.56)
    }

    private fun CommandXboxController.shortRumble(type: RumbleType) =
        Commands.startEnd(
                { hid.setRumble(type, 1.0) },
                { hid.setRumble(type, 0.0) },
            )
            .withTimeout(0.3)
}
