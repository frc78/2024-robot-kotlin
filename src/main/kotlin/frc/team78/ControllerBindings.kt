package frc.team78

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team78.commands.CommandFactory
import frc.team78.commands.VarFeedPrime
import frc.team78.lib.SPEAKER_POSE
import frc.team78.subsystems.chassis.BaseSwerveDrive
import frc.team78.subsystems.chassis.SwerveDrive
import frc.team78.subsystems.elevator.Elevator
import frc.team78.subsystems.feeder.Feeder
import frc.team78.subsystems.shooter.Shooter

fun CommandXboxController.configureDriverBindings() {
    val baseSwerveDrive = BaseSwerveDrive(this.hid, SwerveDrive.MOTION_LIMITS)
    SwerveDrive.defaultCommand =
        SwerveDrive.applyRequest {
                val speeds = baseSwerveDrive.chassisSpeeds
                SwerveDrive.driveRequest.apply {
                    VelocityX = speeds.vxMetersPerSecond
                    VelocityY = speeds.vyMetersPerSecond
                    RotationalRate = speeds.omegaRadiansPerSecond
                }
            }
            .withName("Drive Field Centric")

    rightBumper().whileTrue(CommandFactory.autoPickupNote)
    leftBumper()
        .whileTrue(
            SwerveDrive.applyRequest {
                SwerveDrive.driveAtAngleBlueOriginRequest.apply {
                    VelocityX =
                        -leftY *
                            SwerveDrive.MOTION_LIMITS.maxTranslationVelocity.`in`(MetersPerSecond)
                    VelocityY =
                        -leftX *
                            SwerveDrive.MOTION_LIMITS.maxTranslationVelocity.`in`(MetersPerSecond)
                    TargetDirection =
                        (SPEAKER_POSE - SwerveDrive.estimatedPose.translation)
                            .angle
                            .rotateBy(Rotation2d.fromRadians(Math.PI))
                }
            }
        )

    povDown().whileTrue(SwerveDrive.brake)
}

fun CommandXboxController.configureOperatorBindings() {

    a().whileTrue(CommandFactory.setUpAmp)
    x().whileTrue(VarFeedPrime())
    y().whileTrue(Elevator.goToClimb).onFalse(Elevator.climb)

    rightBumper().whileTrue(CommandFactory.intakeNote)
    leftBumper().whileTrue(Feeder.eject)

    leftTrigger().whileTrue(CommandFactory.prepareToShoot)
    rightTrigger().whileTrue(Feeder.shoot)
}

fun CommandXboxController.configureSysIdBindings() {
    a().whileTrue(Shooter.runSysId)
    b().whileTrue(SwerveDrive.translationSysId)
    x().whileTrue(Elevator.runSysId)
    leftBumper().whileTrue(SwerveDrive.rotationSysId)
    rightBumper().whileTrue(SwerveDrive.steerSysId)
}
