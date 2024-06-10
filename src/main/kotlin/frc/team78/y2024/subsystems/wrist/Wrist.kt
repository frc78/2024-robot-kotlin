package frc.team78.y2024.subsystems.wrist

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkLimitSwitch
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team78.lib.setStatusRates

object Wrist : SubsystemBase("Wrist") {
    private const val MOTOR_CAN_ID = 13
    private const val K_P = 0.03
    private const val FORWARD_SOFT_LIMIT = 60f
    private const val REVERSE_SOFT_LIMIT = 5f
    private const val STOW_ANGLE = 55.0
    private const val AMP_ANGLE = 20.0

    private val positionNtPub =
        NetworkTableInstance.getDefault().getTable("wrist").getDoubleTopic("position").publish()
    private val motor =
        CANSparkMax(MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless).apply {
            restoreFactoryDefaults()
            idleMode = CANSparkBase.IdleMode.kCoast
            pidController.setFeedbackDevice(absoluteEncoder)
            pidController.positionPIDWrappingEnabled = true
            pidController.positionPIDWrappingMinInput = 0.0
            pidController.positionPIDWrappingMaxInput = 360.0
            pidController.p = K_P

            setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, FORWARD_SOFT_LIMIT)
            setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, REVERSE_SOFT_LIMIT)

            setStatusRates(period0 = 20, period4 = 20)
        }

    private val hardLimitFwd = motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)

    private val encoder =
        motor.absoluteEncoder.apply {
            positionConversionFactor = 360.0
            inverted = true
        }

    fun setTarget(target: Double) =
        motor.pidController.setReference(target, CANSparkBase.ControlType.kPosition)

    fun setTargetCommand(target: Double) =
        runOnce { setTarget(target) }.withName("setTarget($target)")

    val position
        get() = encoder.position

    fun stow() = setTargetCommand(STOW_ANGLE)

    fun ampPosition() = setTargetCommand(AMP_ANGLE)

    fun coast() =
        runOnce { motor.idleMode = CANSparkBase.IdleMode.kCoast }
            .ignoringDisable(true)
            .withName("Coast Wrist")

    fun brake() =
        runOnce { motor.idleMode = CANSparkBase.IdleMode.kBrake }
            .ignoringDisable(true)
            .withName("Brake Wrist")

    init {
        SmartDashboard.putData(this)
        SmartDashboard.putData(coast())
        SmartDashboard.putData(brake())
    }

    override fun periodic() {
        positionNtPub.set(encoder.position)
    }

    private val sysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                { voltage -> motor.setVoltage(voltage.`in`(Volts)) },
                null,
                this,
                "wrist",
            ),
        )

    fun sysId() =
        Commands.sequence(
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until { hardLimitFwd.isPressed },
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until {
                motor.getFault(CANSparkBase.FaultID.kSoftLimitRev)
            },
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until {
                hardLimitFwd.isPressed
            },
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until {
                motor.getFault(CANSparkBase.FaultID.kSoftLimitRev)
            },
        )
}
