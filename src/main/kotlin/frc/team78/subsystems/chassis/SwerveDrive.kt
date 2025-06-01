package frc.team78.subsystems.chassis

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.team78.commands.command

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
object SwerveDrive : TunerConstants.MunchkinSwerveDrivetrain(), Subsystem {

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private val blueAlliancePerspectiveRotation: Rotation2d = Rotation2d.fromDegrees(0.0)

    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private val redAlliancePerspectiveRotation: Rotation2d = Rotation2d.fromDegrees(180.0)

    /* Keep track if we've ever applied the operator perspective before or not */
    private var hasAppliedOperatorPerspective = false

    val driveRequest =
        SwerveRequest.FieldCentric().apply {
            DriveRequestType = SwerveModule.DriveRequestType.Velocity
            SteerRequestType = SwerveModule.SteerRequestType.MotionMagicExpo
        }

    val brakeRequest = SwerveRequest.SwerveDriveBrake()

    val brake by command { applyRequest { brakeRequest } }

    fun applyRequest(requestSupplier: () -> SwerveRequest) = run {
        this.setControl(requestSupplier())
    }

    /** Network table subscriber for the note detection position */
    private val noteDetectionYaw =
        NetworkTableInstance.getDefault()
            .getTable("photonvision/Fisheye")
            .getDoubleTopic("targetYaw")
            .subscribe(0.0)
    private val rotationController = PIDController(0.15, 0.0, 0.0015)

    /** Command to drive into the note detected by the camera */
    val autoDriveToNote by command {
        applyRequest {
            driveRequest
                .withVelocityX(2.0)
                .withRotationalRate(rotationController.calculate(noteDetectionYaw.get()))
        }
    }

    override fun periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent { allianceColor: Alliance ->
                this.setOperatorPerspectiveForward(
                    if (allianceColor == Alliance.Red) redAlliancePerspectiveRotation
                    else blueAlliancePerspectiveRotation
                )
                hasAppliedOperatorPerspective = true
            }
        }
    }
}
