// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team78

import com.ctre.phoenix6.SignalLogger
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.hal.FRCNetComm.tInstances.kLanguage_Kotlin
import edu.wpi.first.hal.FRCNetComm.tResourceType.kResourceType_Language
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.net.PortForwarder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team78.commands.CommandFactory
import frc.team78.commands.VarShootPrime
import frc.team78.lib.*
import frc.team78.subsystems.chassis.*
import frc.team78.subsystems.elevator.Elevator
import frc.team78.subsystems.feedback.LED
import frc.team78.subsystems.feeder.Feeder
import frc.team78.subsystems.shooter.Shooter
import frc.team78.subsystems.wrist.Wrist
import java.util.*

object Robot : TimedRobot() {
    init {
        HAL.report(kResourceType_Language, kLanguage_Kotlin, 0, WPILibVersion.Version)
    }

    private var autonomousCommand: Command? = null

    private val driver = CommandXboxController(0).apply { configureDriverBindings() }

    private val operator = CommandXboxController(1).apply { configureOperatorBindings() }

    // SysId Controller
    init {
        CommandXboxController(5).apply { configureSysIdBindings() }
    }

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

        /** Doesn't start logging unless FMS is attached */
        SignalLogger.setPath("/U/ctre-logs")

        Trigger(Feeder::hasNote)
            .whileTrue(LED.indicateNoteInCartridge)
            .and(RobotModeTriggers.teleop())
            .onTrue(driver.shortRumble(RumbleType.kRightRumble))
            .onTrue(operator.shortRumble(RumbleType.kRightRumble))
            .onFalse(driver.shortRumble(RumbleType.kBothRumble))

        Trigger(Shooter::isAtSpeed)
            .whileTrue(LED.indicateShooterWheelsAtSpeed)
            .and(RobotModeTriggers.teleop())
            .onTrue(operator.shortRumble(RumbleType.kBothRumble))

        Trigger(Elevator::isAtGoal)
            .and(RobotModeTriggers.teleop())
            .onTrue(operator.shortRumble(RumbleType.kBothRumble))


        RobotModeTriggers.disabled()
            .and { DriverStation.isDSAttached() }
            .onTrue(LED.indicateDisabled())

        RobotModeTriggers.teleop().onTrue(Elevator.brake.alongWith(Wrist.brake))

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
                "Intake" to CommandFactory.intakeNote,
                "StopShooter" to Shooter.stop,
                "StartShooter" to Shooter.spinUp,
                "Score" to CommandFactory.shootNote,
                "stow" to Wrist.stow,
                "Target" to SwerveDrive.targetSpeaker,
                "DriveToNote" to CommandFactory.autoPickupNoteWithMidlineStop,
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

    private fun CommandXboxController.shortRumble(type: RumbleType) =
            Commands.startEnd({ hid.setRumble(type, 1.0) }, { hid.setRumble(type, 0.0) })
                .withTimeout(0.3)
}
