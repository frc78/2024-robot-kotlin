package frc.team78.subsystems.intake

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Intake : SubsystemBase() {

    private const val LEADER_CAN_ID = 9
    private const val FOLLOWER_CAN_ID = 10
    private const val INTAKE_SPEED = 0.75
    private val stoppedControl = DutyCycleOut(0.0)
    private val intakeControl = DutyCycleOut(INTAKE_SPEED)

    private val leaderMotor =
        TalonFX(LEADER_CAN_ID, "*").apply {
            configurator.apply(MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
            setNeutralMode(NeutralModeValue.Coast)
            optimizeBusUtilization()
        }

    init {
        TalonFX(FOLLOWER_CAN_ID, "*").apply {
            setNeutralMode(NeutralModeValue.Coast)
            // Motors are inverted, but rollers should also be inverted so we do not oppose master
            // direction
            setControl(Follower(LEADER_CAN_ID, false))
            optimizeBusUtilization()
        }

        // When there is no active command, stop the intake
        defaultCommand = runOnce { leaderMotor.setControl(stoppedControl) }.andThen(Commands.idle())
    }

    fun intake(): Command =
        runOnce { leaderMotor.setControl(intakeControl) }.andThen(Commands.idle())
}
