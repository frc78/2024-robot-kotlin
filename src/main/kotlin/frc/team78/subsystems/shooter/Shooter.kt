package frc.team78.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

object Shooter : SubsystemBase() {

    private const val TOP_CAN_ID = 14
    private const val BOTTOM_CAN_ID = 15

    private val SHOT_RPM = Units.RPM.of(4000.0)
    private val SLOW_SHOT_RPM = Units.RPM.of(500.0)

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
        TalonFX(TOP_CAN_ID, "*").apply {
            configurator.apply(topConfig)
            velocity.setUpdateFrequency(50.0)
            statorCurrent.setUpdateFrequency(50.0)
            supplyCurrent.setUpdateFrequency(50.0)
            optimizeBusUtilization()
        }

    private val topVelocity = topMotor.velocity

    private val bottomMotor =
        TalonFX(BOTTOM_CAN_ID, "*").apply {
            configurator.apply(bottomConfig)
            velocity.setUpdateFrequency(50.0)
            closedLoopError.setUpdateFrequency(50.0)
            closedLoopReference.setUpdateFrequency(50.0)
            statorCurrent.setUpdateFrequency(50.0)
            supplyCurrent.setUpdateFrequency(50.0)
            optimizeBusUtilization()
        }
    private val bottomVelocity = bottomMotor.velocity

    fun setSpeed(speed: Measure<Velocity<Angle>>) {
        if (slowShot.get()) {
            velocityControl.Velocity = SLOW_SHOT_RPM.`in`(Units.RotationsPerSecond)
        } else {
            velocityControl.Velocity = speed.`in`(Units.RotationsPerSecond)
        }
        topMotor.setControl(velocityControl)
        bottomMotor.setControl(velocityControl)
    }

    fun spinUp(): Command = runOnce { setSpeed(SHOT_RPM) }.withName("Spin Up")

    fun stop(): Command = runOnce { setSpeed(Units.RotationsPerSecond.of(0.0)) }.withName("Stop")

    val isAtSpeed: Boolean
        get() {
            val reference = topMotor.closedLoopReference.value
            val error = topMotor.closedLoopError.value
            return reference > 0 && error < reference * 0.1
        }

    val velocity
        get() = (topVelocity.value * 60 + bottomVelocity.value * 60) / 2

    override fun periodic() {
        BaseStatusSignal.waitForAll(0.005, topVelocity, bottomVelocity)
        topVelocityPub.set(topVelocity.value * 60)
        bottomVelocityPub.set(bottomVelocity.value)
    }

    private val sysIdVoltage = VoltageOut(0.0, true, true, false, false)
    private val sysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(null, null, null) {
                SignalLogger.writeString("state", it.toString())
            },
            SysIdRoutine.Mechanism(
                { voltage ->
                    sysIdVoltage.Output = voltage.`in`(Volts)
                    topMotor.setControl(sysIdVoltage)
                    bottomMotor.setControl(sysIdVoltage)
                },
                null,
                this,
                "shooter",
            ),
        )

    fun runSysId(): Command =
        Commands.sequence(
                runOnce {
                    SignalLogger.start()
                    topMotor.motorVoltage.setUpdateFrequency(100.0)
                    topMotor.position.setUpdateFrequency(100.0)
                    bottomMotor.motorVoltage.setUpdateFrequency(100.0)
                    bottomMotor.position.setUpdateFrequency(100.0)
                },
                Commands.waitSeconds(1.0),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1.0),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(1.0),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1.0),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                runOnce {
                    SignalLogger.stop()
                    topMotor.motorVoltage.setUpdateFrequency(0.0)
                    topMotor.position.setUpdateFrequency(0.0)
                    bottomMotor.motorVoltage.setUpdateFrequency(0.0)
                    bottomMotor.position.setUpdateFrequency(0.0)
                },
            )
            .withName("Shooter SysId")
}
