package frc.team78.subsystems.shooter

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Commands.idle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team78.commands.command
import frc.team78.lib.rpm

object Shooter : SubsystemBase() {

    private const val TOP_CAN_ID = 14
    private const val BOTTOM_CAN_ID = 15

    private val SHOT_RPM = 4000.rpm
    private val SLOW_SHOT_RPM = 500.rpm

    private val velocityControl = VelocityVoltage(0.0)
    private val topConfig =
        TalonFXConfiguration().apply {
            Slot0.kS = 0.11599
            Slot0.kV = 0.11275
            Slot0.kA = 0.018537
            Slot0.kP = 0.15045
        }
    private val bottomConfig =
        TalonFXConfiguration().apply {
            Slot0.kS = 0.12435
            Slot0.kV = 0.1115
            Slot0.kA = 0.018978
            Slot0.kP = 0.14965
        }

    private val ntTable = NetworkTableInstance.getDefault().getTable("Shooter")
    private val topVelocityPub = ntTable.getDoubleTopic("Top Velocity").publish()
    private val bottomVelocityPub = ntTable.getDoubleTopic("Bottom Velocity").publish()
    private val slowShot = ntTable.getBooleanTopic("Slow Shot").subscribe(false)

    private val topMotor =
        TalonFX(TOP_CAN_ID).apply {
            configurator.apply(topConfig)
            velocity.setUpdateFrequency(100.0)
            torqueCurrent.setUpdateFrequency(50.0)
            supplyCurrent.setUpdateFrequency(50.0)
        }

    private val bottomMotor =
        TalonFX(BOTTOM_CAN_ID).apply {
            configurator.apply(bottomConfig)
            velocity.setUpdateFrequency(100.0)
            closedLoopError.setUpdateFrequency(50.0)
            closedLoopReference.setUpdateFrequency(50.0)
            torqueCurrent.setUpdateFrequency(50.0)
            supplyCurrent.setUpdateFrequency(50.0)
        }

    fun setSpeed(speed: AngularVelocity) {
        velocityControl.withVelocity(if (slowShot.get()) SLOW_SHOT_RPM else speed)
        topMotor.setControl(velocityControl)
        bottomMotor.setControl(velocityControl)
    }

    val spinUp by command { runOnce { setSpeed(SHOT_RPM) }.andThen(idle()).withName("Spin Up") }

    val isAtSpeed: Boolean
        get() {
            val reference = topMotor.closedLoopReference.value
            val velocity = topMotor.velocity.value
            return velocity.rpm > (reference * .9)
        }

    val velocity: AngularVelocity
        get() = (topMotor.velocity.value + bottomMotor.velocity.value) / 2.0

    override fun periodic() {
        topVelocityPub.set(topMotor.velocity.value.rpm)
        bottomVelocityPub.set(bottomMotor.velocity.value.rpm)
    }
}
