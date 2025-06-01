package frc.team78.subsystems.intake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team78.commands.command

object Intake : SubsystemBase() {

    private const val LEADER_CAN_ID = 9
    private const val FOLLOWER_CAN_ID = 10
    private const val INTAKE_SPEED = 0.25
    private val stoppedControl = DutyCycleOut(0.0)
    private val intakeControl = DutyCycleOut(INTAKE_SPEED)

    private val config =
        TalonFXConfiguration().apply {
            MotorOutput.NeutralMode = NeutralModeValue.Coast
            MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
            OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1.0
        }
    private val leaderMotor = TalonFX(LEADER_CAN_ID).apply { configurator.apply(config) }

    init {
        TalonFX(FOLLOWER_CAN_ID).apply {
            setNeutralMode(NeutralModeValue.Coast)
            // Motors are inverted, but rollers should also be inverted so we do not oppose master
            // direction
            setControl(Follower(LEADER_CAN_ID, false))
        }
    }

    val runIntake by command {
        startEnd(
            // Don't run intake if the feeder has a note already
            { leaderMotor.setControl(intakeControl) },
            { leaderMotor.setControl(stoppedControl) },
        )
    }
}
