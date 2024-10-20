package frc.team78.subsystems.wrist

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.reduxrobotics.sensors.canandmag.Canandmag
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team78.lib.rotations
import frc.team78.lib.volts

object Wrist : SubsystemBase("Wrist") {

    init {
        SmartDashboard.putData(this)
        SmartDashboard.putData(coast)
        SmartDashboard.putData(brake)
    }

    private const val MOTOR_CAN_ID = 13
    private const val K_P = 500.0
    private val FORWARD_SOFT_LIMIT = Degrees.of(55.0)
    private val REVERSE_SOFT_LIMIT = Degrees.of(5.0)
    private val STOW_ANGLE = Degrees.of(55.0)
    private val AMP_ANGLE = Degrees.of(20.0)

    private val positionNtPub =
        NetworkTableInstance.getDefault().getTable("wrist").getDoubleTopic("position").publish()
    private val absEncoderNtPub =
        NetworkTableInstance.getDefault().getTable("wrist").getDoubleTopic("absEncoder").publish()
    private val config =
        TalonFXConfiguration().apply {
            this.Slot0.kP = K_P
            this.Slot0.GravityType = GravityTypeValue.Arm_Cosine

            this.ClosedLoopGeneral.ContinuousWrap = true
            // Times 360 to convert to degrees
            this.Feedback.SensorToMechanismRatio = 108.0

            this.MotorOutput.NeutralMode = NeutralModeValue.Coast
            this.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

            this.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT.rotations
            this.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
            this.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT.rotations
            this.SoftwareLimitSwitch.ReverseSoftLimitEnable = true
        }
    private val absEncoder = Canandmag(1)

    private val motor =
        TalonFX(MOTOR_CAN_ID, "*").apply {
            this.configurator.apply(config)
            this.position.setUpdateFrequency(100.0)
            this.velocity.setUpdateFrequency(100.0)
        }

    private val control = PositionVoltage(STOW_ANGLE.rotations)

    fun setTarget(target: Measure<Angle>): StatusCode =
        if (zeroed) {
            motor.setControl(control.withPosition(target.rotations))
        } else {
            motor.setControl(NeutralOut())
        }

    private fun setTargetCommand(target: Measure<Angle>) =
        runOnce { setTarget(target) }.withName("setTarget($target)")

    val position: Double
        get() = motor.position.value

    val stow
        get() = setTargetCommand(STOW_ANGLE)

    val ampPosition
        get() = setTargetCommand(AMP_ANGLE)

    val coast
        get() =
            runOnce { motor.setNeutralMode(NeutralModeValue.Coast) }
                .ignoringDisable(true)
                .withName("Coast Wrist")

    val brake
        get() =
            runOnce { motor.setNeutralMode(NeutralModeValue.Brake) }
                .ignoringDisable(true)
                .withName("Brake Wrist")

    private var zeroed = false

    override fun periodic() {
        positionNtPub.set(position)

        if (!zeroed && absEncoder.isConnected) {
            motor.setPosition(absEncoder.position)
            zeroed = true
        }
        absEncoderNtPub.set(absEncoder.absPosition)
    }

    private val sysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(null, null, null) {
                SignalLogger.writeString("state", it.toString())
            },
            SysIdRoutine.Mechanism(
                { voltage -> motor.setVoltage(voltage.volts) },
                null,
                this,
                "wrist",
            ),
        )

    val runSysId
        get() =
            Commands.sequence(
                runOnce(SignalLogger::start),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until {
                    motor.fault_ForwardSoftLimit.value
                },
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until {
                    motor.fault_ReverseSoftLimit.value
                },
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until {
                    motor.fault_ForwardSoftLimit.value
                },
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until {
                    motor.fault_ReverseSoftLimit.value
                },
                runOnce(SignalLogger::stop),
            )
}
