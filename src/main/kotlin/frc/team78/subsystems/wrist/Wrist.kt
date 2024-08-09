package frc.team78.subsystems.wrist

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.reduxrobotics.sensors.canandmag.Canandmag
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine

object Wrist : SubsystemBase("Wrist") {

    init {
        SmartDashboard.putData(this)
        SmartDashboard.putData(coast())
        SmartDashboard.putData(brake())
    }

    private const val MOTOR_CAN_ID = 13
    private const val K_P = 0.03
    private const val FORWARD_SOFT_LIMIT = 60.0
    private const val REVERSE_SOFT_LIMIT = 5.0
    private const val STOW_ANGLE = 55.0
    private const val AMP_ANGLE = 20.0

    private val positionNtPub =
        NetworkTableInstance.getDefault().getTable("wrist").getDoubleTopic("position").publish()
    private val config =
        TalonFXConfiguration().apply {
            this.Slot0.kP = K_P
            this.Slot0.GravityType = GravityTypeValue.Arm_Cosine

            this.ClosedLoopGeneral.ContinuousWrap = true
            // Times 360 to convert to degrees
            this.Feedback.SensorToMechanismRatio = 360 * 108.0

            this.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT
            this.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
            this.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT
            this.SoftwareLimitSwitch.ReverseSoftLimitEnable = true
        }
    private val absEncoder = Canandmag(0)

    private val motor =
        TalonFX(MOTOR_CAN_ID).apply {
            this.configurator.apply(config)
            // Start the wrist in coast mode when the robot turns on
            this.setNeutralMode(NeutralModeValue.Coast)
            // Seed the position with the absolute encoder
            this.setPosition(absEncoder.position, 1.0)
            this.position.setUpdateFrequency(100.0)
            this.velocity.setUpdateFrequency(100.0)
            this.optimizeBusUtilization()
        }

    private val control =
        PositionVoltage(
            STOW_ANGLE,
            0.0,
            true,
            0.0,
            0,
            false,
            false,
            false,
        )

    fun setTarget(target: Double) = motor.setControl(control.withPosition(target))

    private fun setTargetCommand(target: Double) =
        runOnce { setTarget(target) }.withName("setTarget($target)")

    val position
        get() = motor.position.value

    fun stow() = setTargetCommand(STOW_ANGLE)

    fun ampPosition() = setTargetCommand(AMP_ANGLE)

    fun coast() =
        runOnce { motor.setNeutralMode(NeutralModeValue.Coast) }
            .ignoringDisable(true)
            .withName("Coast Wrist")

    fun brake() =
        runOnce { motor.setNeutralMode(NeutralModeValue.Brake) }
            .ignoringDisable(true)
            .withName("Brake Wrist")

    override fun periodic() {
        positionNtPub.set(position)
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

    fun runSysId() =
        Commands.sequence(
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until {
                motor.fault_ForwardSoftLimit.value
            },
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until {
                motor.fault_ReverseSoftLimit.value
            },
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until {
                motor.fault_ForwardSoftLimit.value
            },
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until {
                motor.fault_ReverseSoftLimit.value
            },
        )
}
