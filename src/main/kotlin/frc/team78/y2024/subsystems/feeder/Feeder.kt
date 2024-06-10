package frc.team78.y2024.subsystems.feeder

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.ForwardLimitValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase

object Feeder : SubsystemBase() {

    private const val MOTOR_CAN_ID = 16
    private val feedMotor =
        TalonFX(MOTOR_CAN_ID).apply {
            forwardLimit.setUpdateFrequency(50.0)
            optimizeBusUtilization()
        }

    private val INTAKE_CONFIG = HardwareLimitSwitchConfigs().withForwardLimitEnable(true)
    private val SHOOT_CONFIG = HardwareLimitSwitchConfigs().withForwardLimitEnable(false)

    private val ntTable = NetworkTableInstance.getDefault().getTable("feeder")
    private val hasNotePub = ntTable.getBooleanTopic("hasNote").publish()

    init {
        SmartDashboard.putData(this)
    }

    fun intake() =
        FunctionalCommand(
                /* onInit = */ {
                    feedMotor.configurator.apply(INTAKE_CONFIG, 0.01)
                    feedMotor.set(0.85)
                },
                /* onExecute = */ {},
                /* onEnd = */ {
                    feedMotor.set(0.0)
                    feedMotor.configurator.apply(SHOOT_CONFIG, 0.01)
                },
                /* isFinished = */ {
                    feedMotor.forwardLimit.value == ForwardLimitValue.ClosedToGround
                },
                /* ...requirements = */ this
            )
            .withName("Intake")

    fun eject() = startEnd({ feedMotor.set(-0.85) }, { feedMotor.set(0.0) }).withName("Outtake")

    fun shoot() =
        FunctionalCommand(
                {
                    feedMotor.configurator.apply(SHOOT_CONFIG, 0.01)
                    feedMotor.set(1.0)
                },
                {},
                {
                    feedMotor.set(0.0)
                    feedMotor.configurator.apply(INTAKE_CONFIG, 0.01)
                },
                { feedMotor.forwardLimit.value == ForwardLimitValue.Open },
                this,
            )
            .withName("Shoot")

    val hasNote
        get() = feedMotor.forwardLimit.value == ForwardLimitValue.ClosedToGround

    override fun periodic() {
        hasNotePub.set(hasNote)
    }
}
