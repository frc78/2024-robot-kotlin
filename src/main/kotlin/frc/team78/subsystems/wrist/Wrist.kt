package frc.team78.subsystems.wrist

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
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team78.commands.command
import frc.team78.lib.degrees
import frc.team78.lib.rotations

object Wrist : SubsystemBase("Wrist") {

    init {
        SmartDashboard.putData(this)
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
            Slot0.kP = K_P
            Slot0.GravityType = GravityTypeValue.Arm_Cosine

            ClosedLoopGeneral.ContinuousWrap = true
            // Times 360 to convert to degrees
            Feedback.SensorToMechanismRatio = 108.0

            MotorOutput.NeutralMode = NeutralModeValue.Coast
            MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

            SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT.rotations
            SoftwareLimitSwitch.ForwardSoftLimitEnable = true
            SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT.rotations
            SoftwareLimitSwitch.ReverseSoftLimitEnable = true
        }

    private val absEncoder: Canandmag = Canandmag(1)

    private val motor =
        TalonFX(MOTOR_CAN_ID, "*").apply {
            this.configurator.apply(config)
            this.position.setUpdateFrequency(100.0)
            this.velocity.setUpdateFrequency(100.0)
        }

    private val control = PositionVoltage(STOW_ANGLE.rotations)

    fun setTarget(target: Angle): StatusCode =
        if (zeroed) {
            motor.setControl(control.withPosition(target.rotations))
        } else {
            motor.setControl(NeutralOut())
        }

    private fun setTargetCommand(target: Angle) =
        runOnce { setTarget(target) }.withName("setTarget($target)")

    val position: Angle
        get() = motor.position.value

    val stow by command { setTargetCommand(STOW_ANGLE) }

    init {
        defaultCommand = stow
    }

    val ampPosition by command { setTargetCommand(AMP_ANGLE) }

    private var zeroed = false

    override fun periodic() {
        positionNtPub.set(position.degrees)

        if (!zeroed && absEncoder.isConnected) {
            motor.setPosition(absEncoder.position)
            zeroed = true
        }
        absEncoderNtPub.set(absEncoder.absPosition)
    }
}
