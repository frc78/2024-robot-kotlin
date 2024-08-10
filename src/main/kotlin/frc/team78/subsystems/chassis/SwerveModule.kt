package frc.team78.subsystems.chassis

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.pathplanner.lib.commands.PathfindingCommand
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Inches
import frc.team78.lib.div
import kotlin.math.PI
import kotlin.math.abs

data class OpenLoopParameters(val kS: Double, val kV: Double, val kA: Double)

data class ClosedLoopParameters(val kP: Double, val kI: Double, val kD: Double)

class SwerveModule(driveCanId: Int, steerCanId: Int, driveOpenLoopParameters: OpenLoopParameters) {

    private val driveConfiguration =
        TalonFXConfiguration().apply {
            Slot0.kS = driveOpenLoopParameters.kS
            Slot0.kV = driveOpenLoopParameters.kV
            Slot0.kA = driveOpenLoopParameters.kA
            Slot0.kP = DRIVE_CLOSED_LOOP_PARAMETERS.kP
            Slot0.kI = DRIVE_CLOSED_LOOP_PARAMETERS.kI
            Slot0.kD = DRIVE_CLOSED_LOOP_PARAMETERS.kD
            Feedback.SensorToMechanismRatio = DRIVE_ENCODER_CONVERSION_FACTOR.magnitude()

            CurrentLimits.StatorCurrentLimit = DRIVE_CURRENT_LIMIT
            CurrentLimits.StatorCurrentLimitEnable = true
        }
    private val driveMotor =
        TalonFX(driveCanId).apply {
            setNeutralMode(NeutralModeValue.Brake)
            inverted = DRIVE_INVERTED
            configurator.apply(driveConfiguration)
            position.setUpdateFrequency(100.0)
            velocity.setUpdateFrequency(100.0)
            statorCurrent.setUpdateFrequency(50.0)
            optimizeBusUtilization()
        }

    private val driveControl =
        VelocityVoltage(
            0.0,
            0.0,
            true,
            0.0,
            0,
            false,
            false,
            false,
        )

    private val encoderConfigs =
        CANcoderConfiguration().apply {
            MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive
        }

    init {
        CANcoder(steerCanId).apply { this.configurator.apply(encoderConfigs) }
    }

    private val steerConfiguration =
        TalonFXConfiguration().apply {
            Slot0.kP = STEER_CLOSED_LOOP_PARAMETERS.kP
            Slot0.kI = STEER_CLOSED_LOOP_PARAMETERS.kI
            Slot0.kD = STEER_CLOSED_LOOP_PARAMETERS.kD
            Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
            Feedback.RotorToSensorRatio = 150.0 / 7.0
            Feedback.FeedbackRemoteSensorID = steerCanId
            ClosedLoopGeneral.ContinuousWrap = true

            CurrentLimits.StatorCurrentLimit = STEER_CURRENT_LIMIT
            CurrentLimits.StatorCurrentLimitEnable = true
        }
    private val steerMotor =
        TalonFX(steerCanId).apply {
            setNeutralMode(NeutralModeValue.Brake)
            inverted = STEER_INVERTED
            configurator.apply(steerConfiguration)
            position.setUpdateFrequency(100.0)
            velocity.setUpdateFrequency(100.0)
            statorCurrent.setUpdateFrequency(50.0)
            optimizeBusUtilization()
        }

    private val steerControl =
        PositionVoltage(
            0.0,
            0.0,
            true,
            0.0,
            0,
            false,
            false,
            false,
        )

    fun enableBrakeMode() {
        driveMotor.setNeutralMode(NeutralModeValue.Brake)
        steerMotor.setNeutralMode(NeutralModeValue.Brake)
    }

    fun enableCoastMode() {
        driveMotor.setNeutralMode(NeutralModeValue.Coast)
        steerMotor.setNeutralMode(NeutralModeValue.Coast)
    }

    private val driveVelocity
        get() = driveMotor.velocity.value

    private val drivePosition
        get() = driveMotor.position.value

    val steerPosition
        get() = Rotation2d.fromRotations(steerMotor.position.value)

    val swerveModuleState
        get() = SwerveModuleState(driveVelocity, steerPosition)

    val position
        get() = SwerveModulePosition(drivePosition, steerPosition)

    fun setState(state: SwerveModuleState) {
        val optimizedState = SwerveModuleState.optimize(state, steerPosition)

        val angleDelta = optimizedState.angle - steerPosition
        val speedScalar = abs(angleDelta.cos)
        optimizedState.speedMetersPerSecond *= speedScalar

        PathfindingCommand.warmupCommand().schedule()

        driveMotor.setControl(driveControl.withVelocity(state.speedMetersPerSecond))

        steerMotor.setControl(steerControl.withPosition(optimizedState.angle.rotations))
    }

    fun openLoopDiffDrive(voltage: Double) {
        steerMotor.setControl(steerControl.withPosition(0.0))
        driveMotor.setVoltage(voltage)
    }

    companion object {
        private const val DRIVE_INVERTED = true
        private const val STEER_INVERTED = true
        private const val DRIVE_CURRENT_LIMIT = 50.0
        private const val STEER_CURRENT_LIMIT = 20.0
        private val WHEEL_DIAMETER = Inches.of(3.85)
        private const val DRIVE_GEAR_RATIO = 5.3571
        private val DRIVE_ENCODER_CONVERSION_FACTOR: Measure<Distance> =
            WHEEL_DIAMETER * PI / DRIVE_GEAR_RATIO

        private val STEER_CLOSED_LOOP_PARAMETERS = ClosedLoopParameters(20.0, 0.0, 1.0)
        private val DRIVE_CLOSED_LOOP_PARAMETERS = ClosedLoopParameters(0.1, 0.0, 0.0)
    }
}
