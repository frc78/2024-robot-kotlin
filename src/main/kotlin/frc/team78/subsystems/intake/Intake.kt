package frc.team78.subsystems.intake

import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Intake : SubsystemBase() {

    private const val LEADER_CAN_ID = 9
    private const val FOLLOWER_CAN_ID = 10
    private const val INTAKE_SPEED = 0.75

    private val leaderMotor =
        TalonFX(LEADER_CAN_ID).apply {
            setNeutralMode(NeutralModeValue.Coast)
            optimizeBusUtilization()
        }

    init {
        TalonFX(FOLLOWER_CAN_ID).apply {
            setNeutralMode(NeutralModeValue.Coast)
            optimizeBusUtilization()
        }
    }

    fun intake() = startEnd({ leaderMotor.set(INTAKE_SPEED) }, { leaderMotor.set(0.0) })
}
