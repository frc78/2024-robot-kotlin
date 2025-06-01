package frc.team78.commands

import edu.wpi.first.wpilibj2.command.Commands
import frc.team78.subsystems.chassis.SwerveDrive
import frc.team78.subsystems.elevator.Elevator
import frc.team78.subsystems.feeder.Feeder
import frc.team78.subsystems.intake.Intake
import frc.team78.subsystems.shooter.Shooter
import frc.team78.subsystems.wrist.Wrist

object CommandFactory {
    val intakeNote by command {
        Feeder.intake.deadlineFor(Intake.runIntake).withName("Pick Up Note")
    }

    val autoPickupNote by command {
        intakeNote.deadlineFor(SwerveDrive.autoDriveToNote).withName("Auto Pickup Note")
    }

    val setUpAmp by command { Wrist.ampPosition.alongWith(Elevator.goToAmp).withName("Amp") }

    val shootNote by command { Commands.waitUntil { Shooter.isAtSpeed }.andThen(Feeder.shoot) }
}
