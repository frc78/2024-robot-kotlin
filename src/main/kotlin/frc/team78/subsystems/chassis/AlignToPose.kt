package frc.team78.subsystems.chassis

import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import frc.team78.lib.MotionLimits

class AlignToPose(
    private val goalPoseSupplier: () -> Pose2d,
    translationPID: PIDConstants,
    thetaPID: PIDConstants,
    motionLimits: MotionLimits,
) : Command() {

    private val xController =
        ProfiledPIDController(
            translationPID.kP,
            translationPID.kI,
            translationPID.kD,
            TrapezoidProfile.Constraints(
                motionLimits.maxTranslationVelocity,
                motionLimits.maxTranslationAcceleration,
            ),
        )
    private val yController =
        ProfiledPIDController(
            translationPID.kP,
            translationPID.kI,
            translationPID.kD,
            TrapezoidProfile.Constraints(
                motionLimits.maxTranslationVelocity,
                motionLimits.maxTranslationAcceleration,
            ),
        )
    private val thetaController =
        ProfiledPIDController(
                thetaPID.kP,
                thetaPID.kI,
                thetaPID.kD,
                TrapezoidProfile.Constraints(
                    motionLimits.maxAngularVelocity,
                    motionLimits.maxAngularAcceleration,
                ),
            )
            .apply { setTolerance(3.0) }

    private var goalPose = goalPoseSupplier()

    init {
        addRequirements(SwerveDrive)
    }

    override fun initialize() {
        goalPose = goalPoseSupplier()
        val pose = SwerveDrive.estimatedPose
        val vel = SwerveDrive.currentRobotChassisSpeeds
        xController.reset(pose.translation.x, vel.vxMetersPerSecond)
        yController.reset(pose.translation.y, vel.vyMetersPerSecond)
        thetaController.reset(pose.rotation.radians, vel.omegaRadiansPerSecond)
    }

    override fun execute() {
        val currentPose = SwerveDrive.estimatedPose

        val xOutput = xController.calculate(currentPose.translation.x, goalPose.translation.x)
        val yOutput = yController.calculate(currentPose.translation.y, goalPose.translation.y)
        val thetaOutput =
            thetaController.calculate(currentPose.rotation.radians, goalPose.rotation.radians)

        SwerveDrive.setControl(
            SwerveDrive.driveRequest.apply {
                VelocityX = xOutput
                VelocityY = yOutput
                RotationalRate = thetaOutput
            }
        )
    }

    override fun isFinished(): Boolean {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal()
    }
}
