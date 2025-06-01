package frc.team78.subsystems.elevator

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DifferentialFollower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.ReverseLimitSourceValue
import com.ctre.phoenix6.signals.ReverseLimitTypeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.idle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team78.commands.command
import frc.team78.lib.inches
import frc.team78.lib.radians
import frc.team78.lib.rotations

/**
 * Elevator subsystem
 *
 * The elevator on the 2024 robot is controlled by 2 Falcons
 *
 * When the elevator is at the bottom, the reverse limit switch is pressed and the elevator is
 * zeroed.
 */
object Elevator : SubsystemBase("Elevator") {

    // Constants for the feedforward calculation
    private const val K_S = 0.070936
    private const val K_V = 0.79005
    private const val K_A = 0.086892
    private const val K_G = 0.088056

    // PID gains
    private const val K_P = 4.572
    private const val K_I = 0.0
    private const val K_D = 0.0
    private const val K_TOLERANCE = 0.1

    private const val LEADER_MOTOR_ID = 11
    private const val FOLLOWER_MOTOR_ID = 12

    private val STOW_HEIGHT = 0.inches
    private val AMP_HEIGHT = 16.3.inches
    private val CLIMB_HEIGHT = 16.9.inches

    private const val GEAR_RATIO = 5.0 * 5.0

    /**
     * Pitch Diameter is 1.29 inches. This measurement is taken from the official drawing of the
     * part https://www.revrobotics.com/content/docs/REV-21-2016-DR.pdf
     */
    val Distance.drumRotations: Angle
        get() = (this.inches / (Math.PI * 1.29)).rotations

    val Angle.elevatorInches: Distance
        get() = (this.rotations * (Math.PI * 1.29)).inches

    // Initialize the SmartDashboard with the elevator commands, and details about this subsystem
    init {
        SmartDashboard.putData(this)
    }

    private val leader =
        TalonFX(LEADER_MOTOR_ID).apply {
            val leaderMotorConfiguration =
                TalonFXConfiguration().apply {
                    Feedback.SensorToMechanismRatio = GEAR_RATIO
                    SoftwareLimitSwitch.ForwardSoftLimitEnable = true
                    // Do not allow the motor to move upwards until after zeroing
                    SoftwareLimitSwitch.withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(17.0.inches.drumRotations)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(0.radians)
                    // Allow the motor to move downwards until the limit switch is pressed
                    SoftwareLimitSwitch.ReverseSoftLimitEnable = false
                    SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0

                    // Enable hardware reverse limit switch
                    HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin
                    HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen
                    HardwareLimitSwitch.ReverseLimitEnable = true

                    MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

                    Slot0.kS = K_S
                    Slot0.kV = K_V
                    Slot0.kA = K_A
                    Slot0.kG = K_G
                    Slot0.kP = K_P
                    Slot0.kI = K_I
                    Slot0.kD = K_D
                    Slot0.GravityType = GravityTypeValue.Elevator_Static
                    Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign

                    MotionMagic.MotionMagicAcceleration = 80.0
                    MotionMagic.MotionMagicCruiseVelocity = 15.0
                }
            configurator.apply(leaderMotorConfiguration)
            position.setUpdateFrequency(100.0)
            velocity.setUpdateFrequency(100.0)
            motorVoltage.setUpdateFrequency(100.0)
            closedLoopError.setUpdateFrequency(100.0)
        }

    init {
        TalonFX(FOLLOWER_MOTOR_ID).apply { setControl(DifferentialFollower(LEADER_MOTOR_ID, true)) }
    }

    private val motionMagicRequest = MotionMagicVoltage(0.0)

    /** Position of the elevator in inches */
    val position: Distance
        get() = leader.position.value.elevatorInches

    val isAtGoal
        get() = leader.closedLoopError.value < K_TOLERANCE

    val goToStow by command { goToPosition(STOW_HEIGHT).withName("goToStow") }

    init {
        defaultCommand = goToStow
    }

    /** Command to move the elevator to the AMP height */
    val goToAmp by command { goToPosition(AMP_HEIGHT).withName("go_to_amp") }

    /** Command to move the elevator to the CLIMB height */
    val goToClimb by command { goToPosition(CLIMB_HEIGHT).withName("go_to_climb") }

    private fun goToPosition(position: Distance): Command =
        runOnce { leader.setControl(motionMagicRequest.withPosition(position.drumRotations)) }
            .andThen(idle())

    override fun periodic() {
        SmartDashboard.putNumber("Elevator Position", position.inches)
    }
}
