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
    val prepareToShoot
        get() = Shooter.spinUp.alongWith(VarShootPrime())

    val intakeNote
        get() = Feeder.intake.deadlineWith(Intake.runIntake).withName("Pick Up Note")

    val autoPickupNote
        get() = intakeNote.deadlineWith(SwerveDrive.autoDriveToNote).withName("Auto Pickup Note")

    val setUpAmp
        get() = Wrist.ampPosition.alongWith(Elevator.goToAmp).withName("Amp")

    val shootNote
        get() = Commands.waitUntil { Shooter.isAtSpeed }.andThen(Feeder.shoot)

    /** Auto pickup note, but don't cross the midline. Used during auto */
    val autoPickupNoteWithMidlineStop
        get() =
            autoPickupNote
                .until {
                    when (DriverStation.getAlliance().getOrNull()) {
                        DriverStation.Alliance.Blue ->
                            SwerveDrive.estimatedPose.translation.x > 8.25
                        else -> SwerveDrive.estimatedPose.translation.x < 8.25
                    }
                }
                .withName("Drive to Note")
}
