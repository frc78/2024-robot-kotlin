package frc.team78.subsystems.feeder

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.ForwardLimitTypeValue
import com.ctre.phoenix6.signals.ForwardLimitValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands.idle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team78.commands.command

object Feeder : SubsystemBase() {

    private const val MOTOR_CAN_ID = 16
    private val feedMotor =
        TalonFX(MOTOR_CAN_ID).apply {
            val config =
                TalonFXConfiguration().apply {
                    HardwareLimitSwitch.ForwardLimitEnable = true
                    HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen
                }
            configurator.apply(config)
        }

    private val ejectControl = DutyCycleOut(-.25)
    private val intakeControl = DutyCycleOut(0.25)
    private val shootControl = DutyCycleOut(1.0).withIgnoreHardwareLimits(true)
    private val stopControl = DutyCycleOut(0.0)

    private val ntTable = NetworkTableInstance.getDefault().getTable("feeder")
    private val hasNotePub = ntTable.getBooleanTopic("hasNote").publish()

    init {
        SmartDashboard.putData(this)

        defaultCommand =
            runOnce { feedMotor.setControl(stopControl) }.andThen(idle()).withName("stop")
    }

    val intake by command {
        startEnd({ feedMotor.setControl(intakeControl) }, { feedMotor.setControl(stopControl) })
            .withName("Intake")
    }

    val eject by command {
        runOnce { feedMotor.setControl(ejectControl) }.andThen(idle()).withName("Eject")
    }

    val shoot by command {
        startEnd({ feedMotor.setControl(shootControl) }, { feedMotor.setControl(stopControl) })
            .withTimeout(0.5)
            .withName("Shoot")
    }

    val hasNote
        get() = feedMotor.forwardLimit.value == ForwardLimitValue.ClosedToGround

    override fun periodic() {
        hasNotePub.set(hasNote)
    }
}
