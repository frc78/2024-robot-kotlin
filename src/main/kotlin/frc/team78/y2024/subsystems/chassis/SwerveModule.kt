package frc.team78.y2024.subsystems.chassis

import com.pathplanner.lib.commands.PathfindingCommand
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Inches
import frc.team78.lib.div
import frc.team78.lib.setStatusRates
import kotlin.math.PI
import kotlin.math.abs

data class OpenLoopParameters(val kS: Double, val kV: Double, val kA: Double)

data class ClosedLoopParameters(val kP: Double, val kI: Double, val kD: Double)

data class SwerveModuleConfig(
    val driveCanId: Int,
    val steerCanId: Int,
    val driveOpenLoopParameters: OpenLoopParameters,
)

class SwerveModule(config: SwerveModuleConfig) {

    private val driveMotor =
        CANSparkMax(config.driveCanId, CANSparkLowLevel.MotorType.kBrushless).apply {
            restoreFactoryDefaults()
            idleMode = CANSparkBase.IdleMode.kBrake
            setSmartCurrentLimit(DRIVE_CURRENT_LIMIT)
            enableVoltageCompensation(NOMINAL_VOLTAGE)
            inverted = DRIVE_INVERTED
            setStatusRates(10, 20, 20)
        }

    private val steerMotor =
        CANSparkMax(config.steerCanId, CANSparkLowLevel.MotorType.kBrushless).apply {
            restoreFactoryDefaults()
            idleMode = CANSparkBase.IdleMode.kBrake
            setSmartCurrentLimit(STEER_CURRENT_LIMIT)
            enableVoltageCompensation(NOMINAL_VOLTAGE)
            inverted = STEER_INVERTED
            setStatusRates(10, 20, period5 = 100)
        }

    private val driveEncoder =
        driveMotor.encoder.apply {
            positionConversionFactor = DRIVE_ENCODER_CONVERSION_FACTOR.magnitude()
            velocityConversionFactor = DRIVE_ENCODER_CONVERSION_FACTOR.magnitude() / 60.0
            averageDepth = 2
            measurementPeriod = 16
        }

    private val steerEncoder =
        steerMotor.absoluteEncoder.apply {
            velocityConversionFactor = 1.0 / 60.0
            inverted = STEER_ENCODER_INVERTED
        }

    private val driveClosedLoopController =
        driveMotor.pidController.apply {
            p = DRIVE_CLOSED_LOOP_PARAMETERS.kP
            i = DRIVE_CLOSED_LOOP_PARAMETERS.kI
            d = DRIVE_CLOSED_LOOP_PARAMETERS.kD
        }
    private val steerClosedLoopController =
        steerMotor.pidController.apply {
            setFeedbackDevice(steerEncoder)
            positionPIDWrappingEnabled = true
            positionPIDWrappingMinInput = STEER_ENCODER_PID_MIN
            positionPIDWrappingMaxInput = STEER_ENCODER_PID_MAX
            p = STEER_CLOSED_LOOP_PARAMETERS.kP
            i = STEER_CLOSED_LOOP_PARAMETERS.kI
            d = STEER_CLOSED_LOOP_PARAMETERS.kD
        }

    private val driveFeedforward =
        SimpleMotorFeedforward(
            config.driveOpenLoopParameters.kS,
            config.driveOpenLoopParameters.kV,
            config.driveOpenLoopParameters.kA
        )

    fun enableBrakeMode() {
        driveMotor.idleMode = CANSparkBase.IdleMode.kBrake
        steerMotor.idleMode = CANSparkBase.IdleMode.kBrake
    }

    fun enableCoastMode() {
        driveMotor.idleMode = CANSparkBase.IdleMode.kCoast
        steerMotor.idleMode = CANSparkBase.IdleMode.kCoast
    }

    val driveVelocity
        get() = driveEncoder.velocity

    val drivePosition
        get() = driveEncoder.position

    val steerPosition
        get() = Rotation2d.fromRotations(steerEncoder.position)

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

        driveClosedLoopController.setReference(
            state.speedMetersPerSecond,
            CANSparkBase.ControlType.kVelocity,
            0,
            driveFeedforward.calculate(driveVelocity, optimizedState.speedMetersPerSecond, 0.02),
            SparkPIDController.ArbFFUnits.kVoltage
        )

        steerClosedLoopController.setReference(
            optimizedState.angle.rotations,
            CANSparkBase.ControlType.kPosition
        )
    }

    fun openLoopDiffDrive(voltage: Double) {
        steerClosedLoopController.setReference(0.0, CANSparkBase.ControlType.kPosition)
        driveMotor.setVoltage(voltage)
    }

    companion object {
        private const val STEER_ENCODER_PID_MIN = 0.0
        private const val STEER_ENCODER_PID_MAX = 1.0
        private const val DRIVE_INVERTED = true
        private const val STEER_INVERTED = true
        private const val STEER_ENCODER_INVERTED = true
        private const val DRIVE_CURRENT_LIMIT = 50
        private const val STEER_CURRENT_LIMIT = 20
        private const val NOMINAL_VOLTAGE = 10.0
        private val WHEEL_DIAMETER = Inches.of(3.85)
        private const val DRIVE_GEAR_RATIO = 5.3571
        private val DRIVE_ENCODER_CONVERSION_FACTOR: Measure<Distance> =
            WHEEL_DIAMETER * PI / DRIVE_GEAR_RATIO

        private val STEER_CLOSED_LOOP_PARAMETERS = ClosedLoopParameters(20.0, 0.0, 1.0)
        private val DRIVE_CLOSED_LOOP_PARAMETERS = ClosedLoopParameters(0.1, 0.0, 0.0)
    }
}
