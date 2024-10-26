package frc.team78.subsystems.feeder

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.ForwardLimitValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands.idle
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team78.commands.command

object Feeder : SubsystemBase() {

    private const val MOTOR_CAN_ID = 16
    private val feedMotor =
        TalonFX(MOTOR_CAN_ID, "*").apply { forwardLimit.setUpdateFrequency(50.0) }

    private val ejectControl = DutyCycleOut(-.85)
    private val intakeControl = DutyCycleOut(0.85)
    private val shootControl = DutyCycleOut(1.0)
    private val stopControl = DutyCycleOut(0.0)

    private val INTAKE_CONFIG = HardwareLimitSwitchConfigs().withForwardLimitEnable(true)
    private val SHOOT_CONFIG = HardwareLimitSwitchConfigs().withForwardLimitEnable(false)

    private val ntTable = NetworkTableInstance.getDefault().getTable("feeder")
    private val hasNotePub = ntTable.getBooleanTopic("hasNote").publish()

    init {
        SmartDashboard.putData(this)

        defaultCommand =
            runOnce { feedMotor.setControl(stopControl) }.andThen(idle()).withName("stop")
    }

    val intake by command {
        FunctionalCommand(
                {
                    feedMotor.configurator.apply(INTAKE_CONFIG, 0.1)
                    feedMotor.setControl(intakeControl)
                },
                {},
                { feedMotor.configurator.apply(SHOOT_CONFIG, .1) },
                { feedMotor.forwardLimit.value == ForwardLimitValue.ClosedToGround },
                this,
            )
            .withName("Intake")
    }

    val eject by command {
        runOnce { feedMotor.setControl(ejectControl) }.andThen(idle()).withName("Eject")
    }

    val shoot by command {
        FunctionalCommand(
                {
                    feedMotor.configurator.apply(SHOOT_CONFIG, 0.01)
                    feedMotor.setControl(shootControl)
                },
                {},
                { feedMotor.configurator.apply(INTAKE_CONFIG, 0.01) },
                { feedMotor.forwardLimit.value == ForwardLimitValue.Open },
                this,
            )
            .withName("Shoot")
    }

    val hasNote
        get() = feedMotor.forwardLimit.value == ForwardLimitValue.ClosedToGround

    override fun periodic() {
        hasNotePub.set(hasNote)
    }
}
