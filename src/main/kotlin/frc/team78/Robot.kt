// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team78

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team78.subsystems.feedback.LED
import frc.team78.subsystems.feeder.Feeder

object Robot : TimedRobot() {

    private val driver = CommandXboxController(0).apply { configureDriverBindings() }

    init {

        /** We use extra controllers for running tests, but these aren't always plugged in */
        DriverStation.silenceJoystickConnectionWarning(true)
        SmartDashboard.putData(CommandScheduler.getInstance())

        /** Doesn't start logging unless FMS is attached */
        SignalLogger.setPath("/U/ctre-logs")

        Trigger(Feeder::hasNote)
            .whileTrue(LED.indicateNoteInCartridge)
            .and(RobotModeTriggers.teleop())
            .onTrue(driver.shortRumble(RumbleType.kRightRumble))
            .onFalse(driver.shortRumble(RumbleType.kBothRumble))

        SmartDashboard.putData(PowerDistribution(1, PowerDistribution.ModuleType.kRev))
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
    }

    private fun CommandXboxController.shortRumble(type: RumbleType) =
        Commands.startEnd({ hid.setRumble(type, 1.0) }, { hid.setRumble(type, 0.0) })
            .withTimeout(0.3)
}
