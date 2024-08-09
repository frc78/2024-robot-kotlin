package frc.team78.subsystems.elevator

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.ReverseLimitSourceValue
import com.ctre.phoenix6.signals.ReverseLimitTypeValue
import com.ctre.phoenix6.signals.ReverseLimitValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.idle
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import kotlin.math.abs

/**
 * Elevator subsystem
 *
 * The elevator on the 2024 robot is controlled by 2 Falcons
 *
 * When the elevator is at the bottom, the reverse limit switch is pressed and the elevator is
 * zeroed.
 */
object Elevator : SubsystemBase("Elevator") {

    // Initialize the SmartDashboard with the elevator commands, and details about this subsystem
    init {
        SmartDashboard.putData(this)
        SmartDashboard.putData(coast())
        SmartDashboard.putData(brake())

        // When there is no active command, move the elevator to the bottom
        defaultCommand = runOnce { motionMagicRequest.Position = 0.0 }.andThen(idle())
    }

    // Constants for the feedforward calculation
    private const val K_S = 0.070936
    private const val K_V = 0.79005
    private const val K_A = 0.086892
    private const val K_G = 0.088056

    // Command loop runs at 50Hz, 20ms period
    private const val K_DT = 0.02

    // PID gains
    private const val K_P = 4.572
    private const val K_I = 0.0
    private const val K_D = 0.0
    private val kTolerance = Inches.of(0.1).magnitude()

    private const val LEADER_MOTOR_ID = 11
    private const val FOLLOWER_MOTOR_ID = 12

    // The elevator can move at a max velocity of 15 in/s and a max acceleration of 80 in/s^2
    private val constraints: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(15.0, 80.0)

    private const val AMP_HEIGHT = 16.3
    private const val CLIMB_HEIGHT = 16.9

    private const val GEAR_RATIO = 5.0 * 5.0

    /**
     * Pitch Diameter is 1.29 inches. This measurement is taken from the official drawing of the
     * part https://www.revrobotics.com/content/docs/REV-21-2016-DR.pdf
     */
    private val DRUM_RADIUS = Inches.of(1.29 / 2.0)

    // Converts 1 rotation of the motor to inches of travel of the elevator
    private val POSITION_CONVERSION_FACTOR = DRUM_RADIUS.magnitude() * 2 * Math.PI / GEAR_RATIO

    private val leaderMotorConfiguration =
        TalonFXConfiguration().apply {
            Feedback.SensorToMechanismRatio = POSITION_CONVERSION_FACTOR
            // Do not allow the motor to move upwards until after zeroing
            SoftwareLimitSwitch.ForwardSoftLimitEnable = true
            SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0
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

    private val leader =
        TalonFX(LEADER_MOTOR_ID).apply {
            position.setUpdateFrequency(50.0)
            optimizeBusUtilization()
            configurator.apply(leaderMotorConfiguration)
        }

    private val follower =
        TalonFX(FOLLOWER_MOTOR_ID).apply { setControl(Follower(LEADER_MOTOR_ID, true)) }

    private val motionMagicRequest =
        MotionMagicVoltage(
            0.0,
            true,
            0.0,
            0,
            false,
            false,
            false,
        )

    /** Whether the elevator has been zeroed */
    var zeroed = false
        private set

    /** Position of the elevator in inches */
    val position
        get() = leader.position.value

    val isAtGoal
        get() = abs(position - motionMagicRequest.Position) < kTolerance

    /**
     * Command to set the motors to coast mode.
     *
     * This allows for more easily moving the elevator by hand when the robot is disabled.
     */
    fun coast() =
        Commands.runOnce({
                leader.setNeutralMode(NeutralModeValue.Coast)
                follower.setNeutralMode(NeutralModeValue.Coast)
            })
            .withName("Coast Elevator")
            .ignoringDisable(true)

    /**
     * Command to set the motors to brake mode.
     *
     * This is necessary to hold the robot on the stage when the robot is disabled.
     */
    fun brake() =
        Commands.runOnce({
                leader.setNeutralMode(NeutralModeValue.Brake)
                follower.setNeutralMode(NeutralModeValue.Brake)
            })
            .withName("Brake Elevator")
            .ignoringDisable(true)

    /**
     * Command to run the zeroing routine.
     *
     * The routine will slowly move the elevator down until the limit switch is pressed, then set
     * the soft limits and allow for positional control.
     */
    fun zero() =
        FunctionalCommand(
                {
                    // When the zero routine starts, the elevator is not zeroed
                    zeroed = false
                },
                {
                    // Drive the elevator down slowly
                    leader.set(-0.1)
                },
                {
                    leader.setPosition(0.0)
                    leaderMotorConfiguration.apply {
                        SoftwareLimitSwitch.ReverseSoftLimitEnable = true
                        SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0
                        SoftwareLimitSwitch.ForwardSoftLimitEnable = true
                        SoftwareLimitSwitch.ForwardSoftLimitThreshold = CLIMB_HEIGHT
                    }
                    zeroed = true
                },
                { leader.reverseLimit.value == ReverseLimitValue.ClosedToGround },
                this,
            )
            // Don't allow interrupting this routine. It must complete to zero the elevator
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
            .withName("Zero Elevator")

    /** Command to move the elevator to the AMP height */
    fun goToAmp() = runOnce { motionMagicRequest.Position = AMP_HEIGHT }.andThen(idle())

    /** Command to move the elevator to the CLIMB height */
    fun goToClimb() = runOnce { motionMagicRequest.Position = CLIMB_HEIGHT }.andThen(idle())

    private val sysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                { voltage -> leader.setVoltage(voltage.`in`(Volts)) },
                null,
                this,
                "elevator",
            ),
        )

    fun runSysId() =
        Commands.sequence(
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until {
                leader.fault_ForwardSoftLimit.value
            },
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until {
                leader.reverseLimit.value == ReverseLimitValue.ClosedToGround
            },
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until {
                leader.fault_ForwardSoftLimit.value
            },
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until {
                leader.reverseLimit.value == ReverseLimitValue.ClosedToGround
            },
        )
}
