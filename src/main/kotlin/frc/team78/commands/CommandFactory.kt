package frc.team78.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Commands
import frc.team78.subsystems.chassis.SwerveDrive
import frc.team78.subsystems.elevator.Elevator
import frc.team78.subsystems.feeder.Feeder
import frc.team78.subsystems.intake.Intake
import frc.team78.subsystems.shooter.Shooter
import frc.team78.subsystems.wrist.Wrist
import kotlin.jvm.optionals.getOrNull

object CommandFactory {
    val prepareToShoot by command { Shooter.spinUp.alongWith(VarShootPrime()) }

    val intakeNote by command {
        Feeder.intake.deadlineFor(Intake.runIntake).withName("Pick Up Note")
    }

    val autoPickupNote by command {
        intakeNote.deadlineFor(SwerveDrive.autoDriveToNote).withName("Auto Pickup Note")
    }

    val setUpAmp by command { Wrist.ampPosition.alongWith(Elevator.goToAmp).withName("Amp") }

    val shootNote by command { Commands.waitUntil { Shooter.isAtSpeed }.andThen(Feeder.shoot) }

    /** Auto pickup note, but don't cross the midline. Used during auto */
    val autoPickupNoteWithMidlineStop by command {
        autoPickupNote
            .until {
                when (DriverStation.getAlliance().getOrNull()) {
                    DriverStation.Alliance.Blue -> SwerveDrive.estimatedPose.translation.x > 8.25

                    else -> SwerveDrive.estimatedPose.translation.x < 8.25
                }
            }
            .withName("Drive to Note")
    }
}
