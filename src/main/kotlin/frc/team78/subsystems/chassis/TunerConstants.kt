package frc.team78.subsystems.chassis

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory
import edu.wpi.first.math.util.Units.inchesToMeters

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
    private val STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private val DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.TorqueCurrentFOC

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private const val SLIP_CURRENT_A = 150.0

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
    const val SPEED_AT_12_VOLTS_MPS = 5.21

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot

    private const val COUPLE_RATIO = 25.0 / 7

    private const val DRIVE_GEAR_RATIO = (50.0 * 16 * 45) / (14 * 28 * 15)
    private const val STEER_GEAR_RATIO = 150.0 / 7
    private const val WHEEL_RADIUS_INCHES = 2.0

    private const val INVERT_LEFT_SIDE = false
    private const val INVERT_RIGHT_SIDE = true

    private const val CAN_BUS_NAME = "*"
    private const val PIGEON_ID = 0

    // These are only used for simulation
    private const val STEER_INERTIA = 0.00001
    private const val DRIVE_INERTIA = 0.001
    // Simulated voltage necessary to overcome friction
    private const val STEER_FRICTION_VOLTAGE = 0.25
    private const val DRIVE_FRICTION_VOLTAGE = 0.25

    val DrivetrainConstants =
        SwerveDrivetrainConstants().apply {
            CANBusName = CAN_BUS_NAME
            Pigeon2Id = PIGEON_ID
            Pigeon2Configs = pigeonConfigs
        }

    private val ConstantCreator =
        SwerveModuleConstantsFactory().apply {
            DriveMotorGearRatio = DRIVE_GEAR_RATIO
            SteerMotorGearRatio = STEER_GEAR_RATIO
            WheelRadius = WHEEL_RADIUS_INCHES
            SlipCurrent = SLIP_CURRENT_A
            SteerMotorGains = steerGains
            DriveMotorGains = driveGains
            SteerMotorClosedLoopOutput = STEER_CLOSED_LOOP_OUTPUT
            DriveMotorClosedLoopOutput = DRIVE_CLOSED_LOOP_OUTPUT
            SpeedAt12Volts = SPEED_AT_12_VOLTS_MPS
            SteerInertia = STEER_INERTIA
            DriveInertia = DRIVE_INERTIA
            SteerFrictionVoltage = STEER_FRICTION_VOLTAGE
            DriveFrictionVoltage = DRIVE_FRICTION_VOLTAGE
            FeedbackSource = SteerFeedbackType.FusedCANcoder
            CouplingGearRatio = COUPLE_RATIO
            DriveMotorInitialConfigs = driveInitialConfigs
            SteerMotorInitialConfigs = steerInitialConfigs
            CANcoderInitialConfigs = cancoderInitialConfigs
        }

    // Front Left
    private const val FRONT_LEFT_DRIVE_MOTOR_ID = 1
    private const val FRONT_LEFT_STEER_MOTOR_ID = 2
    private const val FRONT_LEFT_ENCODER_ID = 1
    private const val FRONT_LEFT_ENCODER_OFFSET = 0.152099609375
    private const val FRONT_LEFT_STEER_INVERT = true

    private const val FRONT_LEFT_X_POS_INCHES = 8.5
    private const val FRONT_LEFT_Y_POS_INCHES = 8.5

    // Front Right
    private const val FRONT_RIGHT_DRIVE_MOTOR_ID = 3
    private const val FRONT_RIGHT_STEER_MOTOR_ID = 4
    private const val FRONT_RIGHT_ENCODER_ID = 2
    private const val FRONT_RIGHT_ENCODER_OFFSET = -0.387939453125
    private const val FRONT_RIGHT_STEER_INVERT = true

    private const val FRONT_RIGHT_X_POS_INCHES = 8.5
    private const val FRONT_RIGHT_Y_POS_INCHES = -8.5

    // Back Left
    private const val BACK_LEFT_DRIVE_MOTOR_ID = 5
    private const val BACK_LEFT_STEER_MOTOR_ID = 6
    private const val BACK_LEFT_ENCODER_ID = 3
    private const val BACK_LEFT_ENCODER_OFFSET = -0.455322265625
    private const val BACK_LEFT_STEER_INVERT = true

    private const val BACK_LEFT_X_POS_INCHES = -8.5
    private const val BACK_LEFT_Y_POS_INCHES = 8.5

    // Back Right
    private const val BACK_RIGHT_DRIVE_MOTOR_ID = 7
    private const val BACK_RIGHT_STEER_MOTOR_ID = 8
    private const val BACK_RIGHT_ENCODER_ID = 4
    private const val BACK_RIGHT_ENCODER_OFFSET = -0.465576171875
    private const val BACK_RIGHT_STEER_INVERT = true

    private const val BACK_RIGHT_X_POS_INCHES = -8.5
    private const val BACK_RIGHT_Y_POS_INCHES = -8.5

    val FrontLeft =
        ConstantCreator.createModuleConstants(
            FRONT_LEFT_STEER_MOTOR_ID,
            FRONT_LEFT_DRIVE_MOTOR_ID,
            FRONT_LEFT_ENCODER_ID,
            FRONT_LEFT_ENCODER_OFFSET,
            inchesToMeters(FRONT_LEFT_X_POS_INCHES),
            inchesToMeters(FRONT_LEFT_Y_POS_INCHES),
            INVERT_LEFT_SIDE,
            FRONT_LEFT_STEER_INVERT,
        )
    val FrontRight =
        ConstantCreator.createModuleConstants(
            FRONT_RIGHT_STEER_MOTOR_ID,
            FRONT_RIGHT_DRIVE_MOTOR_ID,
            FRONT_RIGHT_ENCODER_ID,
            FRONT_RIGHT_ENCODER_OFFSET,
            inchesToMeters(FRONT_RIGHT_X_POS_INCHES),
            inchesToMeters(FRONT_RIGHT_Y_POS_INCHES),
            INVERT_RIGHT_SIDE,
            FRONT_RIGHT_STEER_INVERT,
        )
    val BackLeft =
        ConstantCreator.createModuleConstants(
            BACK_LEFT_STEER_MOTOR_ID,
            BACK_LEFT_DRIVE_MOTOR_ID,
            BACK_LEFT_ENCODER_ID,
            BACK_LEFT_ENCODER_OFFSET,
            inchesToMeters(BACK_LEFT_X_POS_INCHES),
            inchesToMeters(BACK_LEFT_Y_POS_INCHES),
            INVERT_LEFT_SIDE,
            BACK_LEFT_STEER_INVERT,
        )
    val BackRight =
        ConstantCreator.createModuleConstants(
            BACK_RIGHT_STEER_MOTOR_ID,
            BACK_RIGHT_DRIVE_MOTOR_ID,
            BACK_RIGHT_ENCODER_ID,
            BACK_RIGHT_ENCODER_OFFSET,
            inchesToMeters(BACK_RIGHT_X_POS_INCHES),
            inchesToMeters(BACK_RIGHT_Y_POS_INCHES),
            INVERT_RIGHT_SIDE,
            BACK_RIGHT_STEER_INVERT,
        )
}
