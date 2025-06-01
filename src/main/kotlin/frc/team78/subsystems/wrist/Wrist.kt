package frc.team78.subsystems.wrist

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.networktables.NetworkTableInstance
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
    private const val K_P = 100.0
    private val FORWARD_SOFT_LIMIT = 55.degrees
    private val REVERSE_SOFT_LIMIT = 5.degrees
    private val STOW_ANGLE = 55.degrees
    private val AMP_ANGLE = 20.degrees

    private val positionNtPub =
        NetworkTableInstance.getDefault().getTable("wrist").getDoubleTopic("position").publish()
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

    private val motor = TalonFX(MOTOR_CAN_ID).apply { this.configurator.apply(config) }

    private var zeroed = false

    private val control = PositionVoltage(STOW_ANGLE)

    fun setTarget(target: Angle): StatusCode =
        motor.setControl(
            control
                .withPosition(target)
                .withLimitForwardMotion(!zeroed)
                .withLimitReverseMotion(!zeroed)
        )

    val position: Angle
        get() = motor.position.value

    val stow by command { runOnce { setTarget(STOW_ANGLE) }.withName("stow") }

    init {
        defaultCommand = stow
    }

    val ampPosition by command { runOnce { setTarget(AMP_ANGLE) }.withName("amp") }

    override fun periodic() {
        positionNtPub.set(position.degrees)
        if (!zeroed && motor.setPosition(55.degrees) == StatusCode.OK) {
            zeroed = true
        }
    }
}
