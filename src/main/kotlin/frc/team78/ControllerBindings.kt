package frc.team78

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team78.commands.CommandFactory
import frc.team78.subsystems.chassis.SwerveDrive
import frc.team78.subsystems.chassis.TunerConstants
import frc.team78.subsystems.elevator.Elevator
import frc.team78.subsystems.feeder.Feeder
import frc.team78.subsystems.shooter.Shooter

fun CommandXboxController.configureDriverBindings() {
    SwerveDrive.defaultCommand =
        SwerveDrive.applyRequest {
                SwerveDrive.driveRequest.apply {
                    VelocityX = -leftX * TunerConstants.SPEED_AT_12_VOLTS_MPS
                    VelocityY = -leftY * TunerConstants.SPEED_AT_12_VOLTS_MPS
                    RotationalRate = -rightX
                }
            }
            .withName("Drive Field Centric")

    // whileTrue ends command on button release
    // onTrue requires command to end itself
    povDown().whileTrue(SwerveDrive.brake)
    a().whileTrue(CommandFactory.setUpAmp)
    y().whileTrue(Elevator.goToClimb)

    rightBumper().whileTrue(CommandFactory.intakeNote)
    leftBumper().whileTrue(Feeder.eject)

    leftTrigger().whileTrue(Shooter.spinUp)
    // Could have made this onTrue, and shooter looks for note is gone
    rightTrigger().whileTrue(Feeder.shoot)
}
