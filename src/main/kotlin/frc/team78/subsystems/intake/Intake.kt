package frc.team78.subsystems.intake

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Intake : SubsystemBase() {

    private const val LEADER_CAN_ID = 9
    private const val FOLLOWER_CAN_ID = 10
    private const val INTAKE_SPEED = 0.75
    private val stoppedControl = DutyCycleOut(0.0)
    private val intakeControl = DutyCycleOut(INTAKE_SPEED)

    private val leaderMotor =
        TalonFX(LEADER_CAN_ID, "*").apply {
            configurator.apply(
                MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
            )
            setNeutralMode(NeutralModeValue.Coast)
        }

    init {
        TalonFX(FOLLOWER_CAN_ID, "*").apply {
            setNeutralMode(NeutralModeValue.Coast)
            // Motors are inverted, but rollers should also be inverted so we do not oppose master
            // direction
            setControl(Follower(LEADER_CAN_ID, false))
        }
    }

    val intake
        get() =
            startEnd(
                { leaderMotor.setControl(intakeControl) },
                { leaderMotor.setControl(stoppedControl) },
            )
}
