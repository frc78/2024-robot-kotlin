package frc.team78.subsystems.elevator

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DifferentialFollower
import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.*
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.math.util.Units.metersToInches
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team78.commands.command
import frc.team78.lib.inches
import frc.team78.lib.volts

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

    // Command loop runs at 50Hz, 20ms period
    private const val K_DT = 0.02

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
    private val DRUM_DIAMETER = Inches.of(1.29)

    // Converts 1 rotation of the motor to inches of travel of the elevator
    private val POSITION_CONVERSION_FACTOR = GEAR_RATIO / (DRUM_DIAMETER.inches * Math.PI)

    // Initialize the SmartDashboard with the elevator commands, and details about this subsystem
    init {
        SmartDashboard.putData(this)

        // When there is no active command, move the elevator to the bottom
        defaultCommand = zero
    }

    private val elevatorSim =
        ElevatorSim(
            DCMotor.getFalcon500(2),
            25.0,
            10.0,
            inchesToMeters(1.29 / 2),
            0.0,
            inchesToMeters(17.0),
            false,
            inchesToMeters(5.0),
        )

    private val leader =
        TalonFX(LEADER_MOTOR_ID, "*").apply {
            val leaderMotorConfiguration =
                TalonFXConfiguration().apply {
                    Feedback.SensorToMechanismRatio = POSITION_CONVERSION_FACTOR
                    SoftwareLimitSwitch.ForwardSoftLimitEnable = true
                    // Do not allow the motor to move upwards until after zeroing
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
            configurator.apply(leaderMotorConfiguration)
            position.setUpdateFrequency(100.0)
            velocity.setUpdateFrequency(100.0)
            motorVoltage.setUpdateFrequency(100.0)
            closedLoopError.setUpdateFrequency(100.0)
        }

    val follower =
        TalonFX(FOLLOWER_MOTOR_ID, "*").apply {
            setControl(DifferentialFollower(LEADER_MOTOR_ID, true))
        }

    private val motionMagicRequest = DifferentialMotionMagicVoltage(0.0, 0.0)

    /** Whether the elevator has been zeroed */
    private var zeroed = false

    /** Position of the elevator in inches */
    val position: Double
        get() = leader.position.value

    val isAtGoal
        get() = leader.closedLoopError.value < K_TOLERANCE

    /**
     * Command to run the zeroing routine.
     *
     * The routine will slowly move the elevator down until the limit switch is pressed, then set
     * the soft limits and allow for positional control.
     */
    val zero
        get() =
            FunctionalCommand(
                    {
                        // When the zero routine starts, the elevator is not zeroed
                        zeroed = false
                    },
                    {
                        // Drive the elevator down slowly
                        leader.set(-0.1)
                    },
                    { interrupted ->
                        if (interrupted) return@FunctionalCommand
                        leader.setPosition(0.0)
                        val softLimitSwitchConfigs =
                            SoftwareLimitSwitchConfigs().apply {
                                ReverseSoftLimitEnable = true
                                ReverseSoftLimitThreshold = 0.0
                                ForwardSoftLimitEnable = true
                                ForwardSoftLimitThreshold = CLIMB_HEIGHT.inches
                            }
                        leader.configurator.apply(softLimitSwitchConfigs)
                        zeroed = true
                        defaultCommand = goToStow
                    },
                    { leader.reverseLimit.value == ReverseLimitValue.ClosedToGround },
                    this,
                )
                // Don't allow interrupting this routine. It must complete to zero the elevator
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                .withName("Zero Elevator")

    val goToStow by command { goToPosition(STOW_HEIGHT).withName("goToStow") }

    /** Command to move the elevator to the AMP height */
    val goToAmp by command { goToPosition(AMP_HEIGHT).withName("go_to_amp") }

    /** Command to move the elevator to the CLIMB height */
    val goToClimb by command { goToPosition(CLIMB_HEIGHT).withName("go_to_climb") }

    val climb by command { goToPosition(STOW_HEIGHT, true).withName("climb") }

    private fun goToPosition(
        position: Measure<Distance>,
        brakeOnNeutral: Boolean = false,
    ): Command =
        runOnce {
                leader.setControl(
                    motionMagicRequest
                        .withTargetPosition(position.inches)
                        .withOverrideBrakeDurNeutral(brakeOnNeutral)
                )
            }
            .andThen(idle())

    private val sysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(null, Volts.of(4.0), null) {
                SignalLogger.writeString("state", it.toString())
            },
            SysIdRoutine.Mechanism(
                { voltage ->
                    leader.setVoltage(voltage.volts)
                    follower.setVoltage(voltage.volts)
                },
                null,
                this,
                "elevator",
            ),
        )

    val runSysId by command {
        sequence(
            runOnce(SignalLogger::start),
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
            runOnce(SignalLogger::stop),
        )
    }

    override fun periodic() {
        SmartDashboard.putNumber("Elevator Position", position)
    }

    override fun simulationPeriodic() {
        val leaderSim = leader.simState
        leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        val motorVoltage = leaderSim.motorVoltage

        elevatorSim.setInput(motorVoltage)
        elevatorSim.update(K_DT)

        leaderSim.setReverseLimit(elevatorSim.hasHitLowerLimit())

        val elevatorPositionInches = metersToInches(elevatorSim.positionMeters)
        val elevatorVelocityInchesPerSecond = metersToInches(elevatorSim.velocityMetersPerSecond)
        val rotations = elevatorPositionInches * POSITION_CONVERSION_FACTOR
        val rotationsPerSecond = elevatorVelocityInchesPerSecond * POSITION_CONVERSION_FACTOR
        leaderSim.setRawRotorPosition(rotations)
        leaderSim.setRotorVelocity(rotationsPerSecond)

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.currentDrawAmps)
        )
    }
}
