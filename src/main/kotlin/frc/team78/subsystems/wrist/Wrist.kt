package frc.team78.subsystems.wrist

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.reduxrobotics.sensors.canandmag.Canandmag
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import kotlin.math.abs

object Wrist : SubsystemBase("Wrist") {

    init {
        SmartDashboard.putData(this)
        SmartDashboard.putData(coast())
        SmartDashboard.putData(brake())
    }

    private const val MOTOR_CAN_ID = 13
    private const val K_P = 500.0
    private const val FORWARD_SOFT_LIMIT = 55.0
    private const val REVERSE_SOFT_LIMIT = 5.0
    private const val STOW_ANGLE = 55.0
    private const val AMP_ANGLE = 20.0

    private val positionNtPub =
        NetworkTableInstance.getDefault().getTable("wrist").getDoubleTopic("position").publish()
    private val absEncoderNtPub =
        NetworkTableInstance.getDefault().getTable("wrist").getDoubleTopic("absEncoder").publish()
    private val config =
        TalonFXConfiguration().apply {
            this.Slot0.kP = K_P
            this.Slot0.GravityType = GravityTypeValue.Arm_Cosine

            this.ClosedLoopGeneral.ContinuousWrap = true
            // Times 360 to convert to degrees
            this.Feedback.SensorToMechanismRatio = 375.0

            this.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT
            this.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
            this.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT
            this.SoftwareLimitSwitch.ReverseSoftLimitEnable = true
        }
    private val absEncoder = Canandmag(1)

    private val motor =
        TalonFX(MOTOR_CAN_ID, "*").apply {
            this.configurator.apply(config)
            // Start the wrist in coast mode when the robot turns on
            this.setNeutralMode(NeutralModeValue.Coast)
            // Seed the position with the absolute encoder
            this.setPosition(absEncoder.position, 1.0)
            this.position.setUpdateFrequency(100.0)
            this.velocity.setUpdateFrequency(100.0)
            this.optimizeBusUtilization()
        }

    private val control = PositionVoltage(STOW_ANGLE)

    fun setTarget(target: Double): StatusCode = motor.setControl(control.withPosition(target))

    private fun setTargetCommand(target: Double) =
        runOnce { setTarget(target) }.withName("setTarget($target)")

    val position: Double
        get() = motor.position.value

    fun stow(): Command = setTargetCommand(STOW_ANGLE)

    fun ampPosition(): Command = setTargetCommand(AMP_ANGLE)

    fun coast(): Command =
        runOnce { motor.setNeutralMode(NeutralModeValue.Coast) }
            .ignoringDisable(true)
            .withName("Coast Wrist")

    fun brake(): Command =
        runOnce { motor.setNeutralMode(NeutralModeValue.Brake) }
            .ignoringDisable(true)
            .withName("Brake Wrist")

    private var encoderSyncCount = 0
    private var lastEncoderReading = absEncoder.absPosition

    override fun periodic() {
        positionNtPub.set(position)

        absEncoder.absPosition.let {
            absEncoderNtPub.set(it)
            // If the encoder reading is stable for 5 continuous cycles, sync the motor position
            if (abs(it - lastEncoderReading) < 0.01) {
                encoderSyncCount++
                if (encoderSyncCount >= 5) {
                    motor.setPosition(it, 0.0)
                    encoderSyncCount = 0
                }
            } else {
                encoderSyncCount = 0
            }
            lastEncoderReading = it
        }
    }

    private val sysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(null, null, null) {
                SignalLogger.writeString("state", it.toString())
            },
            SysIdRoutine.Mechanism(
                { voltage -> motor.setVoltage(voltage.`in`(Volts)) },
                null,
                this,
                "wrist",
            ),
        )

    fun runSysId(): Command =
        Commands.sequence(
            runOnce(SignalLogger::start),
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
            runOnce(SignalLogger::stop),
        )
}
