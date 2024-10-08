package frc.team78.subsystems.chassis

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory
import edu.wpi.first.math.util.Units

// Generated by the Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
object TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.
    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private val steerGains =
        Slot0Configs().apply {
            kP = 100.0
            kI = 0.0
            kD = 0.2
            kS = 0.0
            kV = 1.5
            kA = 0.0
        }
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private val driveGains =
        Slot0Configs().apply {
            kP = 3.0
            kI = 0.0
            kD = 0.0
            kS = 0.0
            kV = 0.0
            kA = 0.0
        }

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private val steerClosedLoopOutput = ClosedLoopOutputType.Voltage
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private val driveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private const val kSlipCurrentA = 150.0

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private val driveInitialConfigs = TalonFXConfiguration()
    private val steerInitialConfigs =
        TalonFXConfiguration().apply {
            /* Swerve azimuth does not require much torque output, so we can set a relatively low
            stator current limit to help avoid brownouts without impacting performance.*/
            CurrentLimits.StatorCurrentLimit = 60.0
            CurrentLimits.StatorCurrentLimitEnable = true
        }

    private val cancoderInitialConfigs = CANcoderConfiguration()
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private val pigeonConfigs = null

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    const val kSpeedAt12VoltsMps: Double = 11.92

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private const val kCoupleRatio = 3.125

    private const val kDriveGearRatio = 5.357142857142857
    private const val kSteerGearRatio = 21.428571428571427
    private const val kWheelRadiusInches = 4.0

    private const val kInvertLeftSide = false
    private const val kInvertRightSide = true

    private const val kCANbusName = "Demo Bot CANivore"
    private const val kPigeonId = 0

    // These are only used for simulation
    private const val kSteerInertia = 0.00001
    private const val kDriveInertia = 0.001
    // Simulated voltage necessary to overcome friction
    private const val kSteerFrictionVoltage = 0.25
    private const val kDriveFrictionVoltage = 0.25

    val DrivetrainConstants =
        SwerveDrivetrainConstants().apply {
            CANbusName = kCANbusName
            Pigeon2Id = kPigeonId
            Pigeon2Configs = pigeonConfigs
        }

    private val ConstantCreator =
        SwerveModuleConstantsFactory().apply {
            DriveMotorGearRatio = kDriveGearRatio
            SteerMotorGearRatio = kSteerGearRatio
            WheelRadius = kWheelRadiusInches
            SlipCurrent = kSlipCurrentA
            SteerMotorGains = steerGains
            DriveMotorGains = driveGains
            SteerMotorClosedLoopOutput = steerClosedLoopOutput
            DriveMotorClosedLoopOutput = driveClosedLoopOutput
            SpeedAt12VoltsMps = kSpeedAt12VoltsMps
            SteerInertia = kSteerInertia
            DriveInertia = kDriveInertia
            SteerFrictionVoltage = kSteerFrictionVoltage
            DriveFrictionVoltage = kDriveFrictionVoltage
            FeedbackSource = SteerFeedbackType.FusedCANcoder
            CouplingGearRatio = kCoupleRatio
            DriveMotorInitialConfigs = driveInitialConfigs
            SteerMotorInitialConfigs = steerInitialConfigs
            CANcoderInitialConfigs = cancoderInitialConfigs
        }

    // Front Left
    private const val kFrontLeftDriveMotorId = 1
    private const val kFrontLeftSteerMotorId = 2
    private const val kFrontLeftEncoderId = 1
    private const val kFrontLeftEncoderOffset = 0.152099609375
    private const val kFrontLeftSteerInvert = true

    private const val kFrontLeftXPosInches = 8.5
    private const val kFrontLeftYPosInches = 8.5

    // Front Right
    private const val kFrontRightDriveMotorId = 3
    private const val kFrontRightSteerMotorId = 4
    private const val kFrontRightEncoderId = 2
    private const val kFrontRightEncoderOffset = -0.387939453125
    private const val kFrontRightSteerInvert = true

    private const val kFrontRightXPosInches = 8.5
    private const val kFrontRightYPosInches = -8.5

    // Back Left
    private const val kBackLeftDriveMotorId = 5
    private const val kBackLeftSteerMotorId = 6
    private const val kBackLeftEncoderId = 3
    private const val kBackLeftEncoderOffset = -0.455322265625
    private const val kBackLeftSteerInvert = true

    private const val kBackLeftXPosInches = -8.5
    private const val kBackLeftYPosInches = 8.5

    // Back Right
    private const val kBackRightDriveMotorId = 7
    private const val kBackRightSteerMotorId = 8
    private const val kBackRightEncoderId = 4
    private const val kBackRightEncoderOffset = -0.465576171875
    private const val kBackRightSteerInvert = true

    private const val kBackRightXPosInches = -8.5
    private const val kBackRightYPosInches = -8.5

    val FrontLeft =
        ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId,
                kFrontLeftDriveMotorId,
                kFrontLeftEncoderId,
                kFrontLeftEncoderOffset,
                Units.inchesToMeters(kFrontLeftXPosInches),
                Units.inchesToMeters(kFrontLeftYPosInches),
                kInvertLeftSide,
            )
            .apply { SteerMotorInverted = kFrontLeftSteerInvert }
    val FrontRight =
        ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId,
                kFrontRightDriveMotorId,
                kFrontRightEncoderId,
                kFrontRightEncoderOffset,
                Units.inchesToMeters(kFrontRightXPosInches),
                Units.inchesToMeters(kFrontRightYPosInches),
                kInvertRightSide,
            )
            .apply { SteerMotorInverted = kFrontRightSteerInvert }
    val BackLeft =
        ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId,
                kBackLeftDriveMotorId,
                kBackLeftEncoderId,
                kBackLeftEncoderOffset,
                Units.inchesToMeters(kBackLeftXPosInches),
                Units.inchesToMeters(kBackLeftYPosInches),
                kInvertLeftSide,
            )
            .apply { SteerMotorInverted = kBackLeftSteerInvert }
    val BackRight =
        ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId,
                kBackRightDriveMotorId,
                kBackRightEncoderId,
                kBackRightEncoderOffset,
                Units.inchesToMeters(kBackRightXPosInches),
                Units.inchesToMeters(kBackRightYPosInches),
                kInvertRightSide,
            )
            .apply { SteerMotorInverted = kBackRightSteerInvert }
}
