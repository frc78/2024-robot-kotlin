package frc.team78.y2024.subsystems.intake

import com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team78.lib.setStatusRates

object Intake : SubsystemBase() {

    private const val LEADER_CAN_ID = 9
    private const val FOLLOWER_CAN_ID = 10
    private const val INTAKE_SPEED = 0.75

    private val leaderMotor =
        CANSparkMax(LEADER_CAN_ID, kBrushless).apply {
            restoreFactoryDefaults()
            setStatusRates(20, 20)
        }

    private val followerMotor =
        CANSparkMax(FOLLOWER_CAN_ID, kBrushless).apply {
            restoreFactoryDefaults()
            follow(leaderMotor)
            setStatusRates(500, 20)
        }

    val intake
        get() = startEnd({ leaderMotor.set(INTAKE_SPEED) }, { leaderMotor.set(0.0) })
}
