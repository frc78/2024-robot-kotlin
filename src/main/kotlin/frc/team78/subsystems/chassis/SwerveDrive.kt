package frc.team78.subsystems.chassis

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference.RedAlliance
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PIDCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team78.commands.command
import frc.team78.lib.*
import frc.team78.subsystems.chassis.TunerConstants.BackLeft
import frc.team78.subsystems.chassis.TunerConstants.BackRight
import frc.team78.subsystems.chassis.TunerConstants.DrivetrainConstants
import frc.team78.subsystems.chassis.TunerConstants.FrontLeft
import frc.team78.subsystems.chassis.TunerConstants.FrontRight
import kotlin.math.PI

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
object SwerveDrive :
    SwerveDrivetrain(DrivetrainConstants, 250.0, FrontLeft, FrontRight, BackLeft, BackRight),
    Subsystem {
    private lateinit var m_simNotifier: Notifier
    private var m_lastSimTime = 0.0

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

    val driveAtAngleBlueOriginRequest =
        SwerveRequest.FieldCentricFacingAngle().apply {
            ForwardReference = RedAlliance
            DriveRequestType = SwerveModule.DriveRequestType.Velocity
            SteerRequestType = SwerveModule.SteerRequestType.MotionMagicExpo
            HeadingController =
                PhoenixPIDController(4.0, 0.0, 0.085).apply {
                    enableContinuousInput(0.0, 2 * PI)
                    setTolerance(2.degrees.radians)
                }
        }

    val brakeRequest = SwerveRequest.SwerveDriveBrake()

    val brake by command { applyRequest { brakeRequest } }

    val targetSpeaker by command {
        applyRequest {
            driveAtAngleBlueOriginRequest.apply {
                VelocityX = 0.0
                VelocityY = 0.0
                TargetDirection =
                    (SPEAKER_POSE - estimatedPose.translation).angle +
                        Rotation2d.fromRadians(Math.PI)
            }
        }
    }

    val translationSysIdRequest = SysIdSwerveTranslationTorqueCurrentFOC()

    val rotationSysIdRequest = SwerveRequest.SysIdSwerveRotation()

    val steerSysIdRequest = SwerveRequest.SysIdSwerveSteerGains()

    init {
        if (Utils.isSimulation()) {
            startSimThread()
        }
        configurePathPlanner()
    }

    /* Use one of these sysidroutines for your particular test */
    private val sysIdRoutineTranslation =
        SysIdRoutine(
            SysIdRoutine.Config(null, 4.volts, null) {
                SignalLogger.writeString("state", it.toString())
            },
            SysIdRoutine.Mechanism(
                { setControl(translationSysIdRequest.withCurrent(Amps.of(it.magnitude()))) },
                null,
                this,
            ),
        )

    private val sysIdRoutineRotation =
        SysIdRoutine(
            SysIdRoutine.Config(null, 4.volts, null) {
                SignalLogger.writeString("state", it.toString())
            },
            SysIdRoutine.Mechanism({ setControl(rotationSysIdRequest.withVolts(it)) }, null, this),
        )

    private val sysIdRoutineSteer =
        SysIdRoutine(
            SysIdRoutine.Config(null, 7.volts, null) {
                SignalLogger.writeString("state", it.toString())
            },
            SysIdRoutine.Mechanism({ setControl(steerSysIdRequest.withVolts(it)) }, null, this),
        )

    val translationSysId =
        Commands.sequence(
            runOnce(SignalLogger::start),
            sysIdRoutineTranslation.quasistatic(SysIdRoutine.Direction.kForward),
            sysIdRoutineTranslation.quasistatic(SysIdRoutine.Direction.kReverse),
            sysIdRoutineTranslation.dynamic(SysIdRoutine.Direction.kForward),
            sysIdRoutineTranslation.dynamic(SysIdRoutine.Direction.kReverse),
            runOnce(SignalLogger::stop),
        )

    val rotationSysId =
        Commands.sequence(
            runOnce(SignalLogger::start),
            sysIdRoutineRotation.quasistatic(SysIdRoutine.Direction.kForward),
            sysIdRoutineRotation.quasistatic(SysIdRoutine.Direction.kReverse),
            sysIdRoutineRotation.dynamic(SysIdRoutine.Direction.kForward),
            sysIdRoutineRotation.dynamic(SysIdRoutine.Direction.kReverse),
            runOnce(SignalLogger::stop),
        )

    val steerSysId =
        Commands.sequence(
            runOnce(SignalLogger::start),
            sysIdRoutineSteer.quasistatic(SysIdRoutine.Direction.kForward),
            sysIdRoutineSteer.quasistatic(SysIdRoutine.Direction.kReverse),
            sysIdRoutineSteer.dynamic(SysIdRoutine.Direction.kForward),
            sysIdRoutineSteer.dynamic(SysIdRoutine.Direction.kReverse),
            runOnce(SignalLogger::stop),
        )

    private val autoRequest = SwerveRequest.ApplyChassisSpeeds()

    private fun configurePathPlanner() {
        val driveBaseRadius = m_moduleLocations.maxOf { it.norm }

        AutoBuilder.configureHolonomic(
            { this.state.Pose }, // Supplier of current robot pose
            this::seedFieldRelative, // Consumer for seeding pose against auto
            { state.speeds },
            { speeds: ChassisSpeeds ->
                this.setControl(autoRequest.withSpeeds(speeds))
            }, // Consumer of ChassisSpeeds to drive the robot
            HolonomicPathFollowerConfig(
                TRANSLATION_PID,
                ROTATION_PID,
                TunerConstants.SPEED_AT_12_VOLTS_MPS,
                driveBaseRadius,
                ReplanningConfig(),
            ),
            {
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            }, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
            this,
        )
    }

    fun applyRequest(requestSupplier: () -> SwerveRequest) = run { this.setControl(requestSupplier()) }

    private fun startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds()

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = Notifier {
            val currentTime = Utils.getCurrentTimeSeconds()
            val deltaTime = currentTime - m_lastSimTime
            m_lastSimTime = currentTime

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage())
        }
        m_simNotifier.startPeriodic(kSimLoopPeriod)
    }

    /** Network table subscriber for the note detection position */
    private val noteDetectionYaw =
        NetworkTableInstance.getDefault()
            .getTable("photonvision/Fisheye")
            .getDoubleTopic("targetYaw")
            .subscribe(0.0)
    private val rotationController = PIDController(0.15, 0.0, 0.0015)

    /** Command to drive into the note detected by the camera */
    val autoDriveToNote =
        PIDCommand(
            rotationController,
            noteDetectionYaw,
            0.0,
            {
                setControl(
                    driveRequest.apply {
                        VelocityX = 2.0
                        VelocityY = 0.0
                        RotationalRate = it
                    }
                )
            },
            this,
        )

    val estimatedPose = m_odometry.estimatedPosition

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

    private const val kSimLoopPeriod = 0.005 // 5 ms

    val MOTION_LIMITS =
        MotionLimits(
            5.6.metersPerSecond,
            12.radiansPerSecond,
            3.metersPerSecondPerSecond,
            18.radiansPerSecondPerSecond,
        )
    val TRANSLATION_PID = PIDConstants(2.0, 0.0, 0.0)
    val ROTATION_PID = PIDConstants(4.0, 0.0, 0.085)
}
