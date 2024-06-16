package frc.team78.y2024.subsystems.elevator

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkLimitSwitch
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team78.lib.setStatusRates

object Elevator : SubsystemBase("Elevator") {

    init {
        SmartDashboard.putData(this)
        SmartDashboard.putData(coast())
        SmartDashboard.putData(brake())

        defaultCommand = moveToTarget(0.0)
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

    private val constraints: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(15.0, 80.0)

    private const val AMP_HEIGHT = 16.3
    private const val CLIMB_HEIGHT = 16.9

    private const val GEAR_RATIO = 5.0 * 5.0

    /** Diameter is 1.29 inches */
    private val DRUM_RADIUS = Inches.of(1.29 / 2.0)

    private val POSITION_CONVERSION_FACTOR = DRUM_RADIUS.magnitude() * 2 * Math.PI / GEAR_RATIO

    private const val LEADER_MOTOR_INVERTED = false
    private const val FOLLOWER_MOTOR_INVERTED = true

    // Taken from CAD
    private val CARRIAGE_MASS = Units.Pounds.of(18.427)

    private val MIN_HEIGHT = Inches.of(0.0)
    private val MAX_HEIGHT = Inches.of(16.9)

    // Start above 0 so that the mechanism zeroes itself
    private val SIM_STARTING_HEIGHT = Inches.of(5.0)

    private val elevatorGearbox = DCMotor.getNEO(2)
    private val elevatorSim =
        ElevatorSim(
            elevatorGearbox,
            GEAR_RATIO,
            CARRIAGE_MASS.`in`(Units.Kilograms),
            DRUM_RADIUS.`in`(Units.Meters),
            MIN_HEIGHT.`in`(Units.Meters),
            MAX_HEIGHT.`in`(Units.Meters),
            true,
            SIM_STARTING_HEIGHT.`in`(Units.Meters),
        )

    private val mech2d = Mechanism2d(20.0, 50.0)
    private val root2d = mech2d.getRoot("Elevator Root", 10.0, 0.0)
    private val elevatorMech =
        root2d.append(MechanismLigament2d("Elevator", elevatorSim.positionMeters, 90.0))

    private val feedForward = ElevatorFeedforward(K_S, K_G, K_V, K_A)

    private val pidController =
        ProfiledPIDController(K_P, K_I, K_D, constraints, K_DT).apply { setTolerance(kTolerance) }

    private val leaderMotor =
        CANSparkMax(LEADER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless).apply {
            restoreFactoryDefaults()
            inverted = LEADER_MOTOR_INVERTED
            enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false)
            enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false)
            setStatusRates(10, 20, 20)
        }

    private val reverseLimitSwitch =
        leaderMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen)

    private val follower =
        CANSparkMax(FOLLOWER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless).apply {
            restoreFactoryDefaults()
            follow(leaderMotor, FOLLOWER_MOTOR_INVERTED)
            setStatusRates(500)
        }
    private val encoder =
        leaderMotor.encoder.apply { positionConversionFactor = POSITION_CONVERSION_FACTOR }

    var zeroed = false
        private set

    val position
        get() = encoder.position

    val isAtGoal
        get() = pidController.atGoal()

    fun coast() =
        Commands.runOnce({
                leaderMotor.idleMode = CANSparkBase.IdleMode.kCoast
                follower.idleMode = CANSparkBase.IdleMode.kCoast
            })
            .withName("Coast Elevator")
            .ignoringDisable(true)

    fun brake() =
        Commands.runOnce({
                leaderMotor.idleMode = CANSparkBase.IdleMode.kBrake
                follower.idleMode = CANSparkBase.IdleMode.kBrake
            })
            .withName("Brake Elevator")
            .ignoringDisable(true)

    fun zero() =
        FunctionalCommand(
                { zeroed = false },
                { leaderMotor.set(-0.1) },
                {
                    encoder.position = 0.0
                    pidController.reset(0.0)
                    leaderMotor.apply {
                        enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true)
                        setSoftLimit(
                            CANSparkBase.SoftLimitDirection.kForward,
                            MAX_HEIGHT.magnitude().toFloat(),
                        )
                        enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true)
                        setSoftLimit(
                            CANSparkBase.SoftLimitDirection.kReverse,
                            MIN_HEIGHT.magnitude().toFloat(),
                        )
                    }
                    zeroed = true
                },
                { reverseLimitSwitch.isPressed },
                this,
            )
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
            .withName("Zero Elevator")

    fun goToAmp() = moveToTarget(AMP_HEIGHT)

    fun goToClimb() = moveToTarget(CLIMB_HEIGHT)

    private fun moveToTarget(target: Double) =
        FunctionalCommand(
                { pidController.setGoal(target) },
                {
                    if (zeroed) {
                        val currentVelocity = pidController.setpoint.velocity
                        leaderMotor.setVoltage(
                            pidController.calculate(encoder.position) +
                                feedForward.calculate(
                                    currentVelocity,
                                    pidController.setpoint.velocity,
                                    K_DT
                                )
                        )
                    }
                },
                {},
                { false },
                this,
            )
            .withName("setTo[$target]")

    override fun simulationPeriodic() {
        elevatorSim.setInput(leaderMotor.appliedOutput * RobotController.getBatteryVoltage())
        elevatorSim.update(K_DT)
        // TODO the encoder doesn't get updated in sim, so the mechanism just falls once it reaches
        // the top?
        elevatorMech.length = elevatorSim.positionMeters
    }

    private val sysIdRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                { voltage -> leaderMotor.setVoltage(voltage.`in`(Volts)) },
                null,
                this,
                "elevator",
            ),
        )

    fun runSysId() =
        Commands.sequence(
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until {
                leaderMotor.getFault(CANSparkBase.FaultID.kSoftLimitFwd)
            },
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until {
                reverseLimitSwitch.isPressed
            },
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until {
                leaderMotor.getFault(CANSparkBase.FaultID.kSoftLimitFwd)
            },
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until {
                reverseLimitSwitch.isPressed
            },
        )
}
