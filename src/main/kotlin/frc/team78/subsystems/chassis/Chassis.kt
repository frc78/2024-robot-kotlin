package frc.team78.subsystems.chassis

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units.FeetPerSecond
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config
import frc.team78.Robot
import frc.team78.lib.div
import frc.team78.lib.unaryMinus
import kotlin.math.hypot

object Chassis : SubsystemBase() {

    private const val FRONT_LEFT_DRIVE_ID = 1
    private const val FRONT_LEFT_STEER_ID = 2
    private const val FRONT_RIGHT_DRIVE_ID = 3
    private const val FRONT_RIGHT_STEER_ID = 4
    private const val BACK_LEFT_DRIVE_ID = 5
    private const val BACK_LEFT_STEER_ID = 6
    private const val BACK_RIGHT_DRIVE_ID = 7
    private const val BACK_RIGHT_STEER_ID = 8

    private val FRONT_LEFT_OPEN_LOOP_PARAMETERS = OpenLoopParameters(0.078918, 2.1152, 0.73299)
    private val FRONT_RIGHT_OPEN_LOOP_PARAMETERS = OpenLoopParameters(0.26707, 2.0848, 0.36198)
    private val BACK_LEFT_OPEN_LOOP_PARAMETERS = OpenLoopParameters(0.25259, 2.0883, 0.35247)
    private val BACK_RIGHT_OPEN_LOOP_PARAMETERS = OpenLoopParameters(0.055245, 2.1739, 0.73292)

    private val WHEELBASE = Inches.of(16.75)
    private val TRACK = Inches.of(16.75)

    init {
        AutoBuilder.configureHolonomic(
            PoseEstimator::pose,
            PoseEstimator::resetPose,
            { kinematics.toChassisSpeeds(*states) },
            { drive(it) },
            HolonomicPathFollowerConfig(
                Robot.Constants.TRANSLATION_PID,
                Robot.Constants.ROTATION_PID,
                Robot.Constants.MOTION_LIMITS.maxTranslationVelocity.`in`(MetersPerSecond),
                hypot(WHEELBASE.`in`(Meters), TRACK.`in`(Meters)),
                ReplanningConfig(true, true),
            ),
            { DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red },
            this,
        )
    }

    private val MODULE_TRANSLATIONS =
        arrayOf(
            Translation2d(WHEELBASE / 2, TRACK / 2),
            Translation2d(WHEELBASE / 2, -TRACK / 2),
            Translation2d(-WHEELBASE / 2, TRACK / 2),
            Translation2d(-WHEELBASE / 2, -TRACK / 2),
        )

    private val swerveModules =
        arrayOf(
            SwerveModule(FRONT_LEFT_DRIVE_ID, FRONT_LEFT_STEER_ID, FRONT_LEFT_OPEN_LOOP_PARAMETERS),
            SwerveModule(
                FRONT_RIGHT_DRIVE_ID,
                FRONT_RIGHT_STEER_ID,
                FRONT_RIGHT_OPEN_LOOP_PARAMETERS
            ),
            SwerveModule(BACK_LEFT_DRIVE_ID, BACK_LEFT_STEER_ID, BACK_LEFT_OPEN_LOOP_PARAMETERS),
            SwerveModule(BACK_RIGHT_DRIVE_ID, BACK_RIGHT_STEER_ID, BACK_RIGHT_OPEN_LOOP_PARAMETERS),
        )

    val kinematics = SwerveDriveKinematics(*MODULE_TRANSLATIONS)

    private val table = NetworkTableInstance.getDefault().getTable("chassis")
    private val statePub = table.getStructArrayTopic("states", SwerveModuleState.struct).publish()

    val positions: Array<SwerveModulePosition>
        get() = swerveModules.map { it.position }.toTypedArray()

    val states: Array<SwerveModuleState>
        get() = swerveModules.map { it.swerveModuleState }.toTypedArray()

    fun drive(speeds: ChassisSpeeds, centerOfRotation: Translation2d = Translation2d()) {
        val discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02)
        val states = kinematics.toSwerveModuleStates(discretizedSpeeds, centerOfRotation)
        SwerveDriveKinematics.desaturateWheelSpeeds(states, FeetPerSecond.of(16.0))
        swerveModules.forEachIndexed { index, module -> module.setState(states[index]) }
        statePub.set(swerveModules.map { it.swerveModuleState }.toTypedArray())
    }

    fun lockWheels() =
        startEnd(
            {
                swerveModules[0].setState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
                swerveModules[1].setState(SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)))
                swerveModules[2].setState(SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)))
                swerveModules[3].setState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
            },
            {},
        )

    private val sysIdRoutine =
        SysIdRoutine(
            Config(),
            SysIdRoutine.Mechanism(
                { voltage -> swerveModules.forEach { it.openLoopDiffDrive(voltage.`in`(Volts)) } },
                null,
                this,
                "chassis",
            ),
        )

    fun runSysId() =
        Commands.sequence(
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(1.0),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(1.0),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(1.0),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        )

    fun brake() =
        runOnce { swerveModules.forEach { it.enableBrakeMode() } }
            .ignoringDisable(true)
            .withName("Brake Chassis")

    fun coast() =
        runOnce { swerveModules.forEach { it.enableCoastMode() } }
            .ignoringDisable(true)
            .withName("Coast Chassis")

    private val noteDetectionYaw =
        NetworkTableInstance.getDefault()
            .getTable("photonvision/Fisheye")
            .getDoubleTopic("targetYaw")
            .subscribe(0.0)
    private val rotationController = PIDController(0.15, 0.0, 0.0015)

    fun autoAlignToNote() =
        FunctionalCommand(
            {
                rotationController.reset()
                rotationController.setpoint = 0.0
            },
            {
                drive(
                    ChassisSpeeds(
                        2.0,
                        0.0,
                        rotationController.calculate(noteDetectionYaw.get()),
                    )
                )
            },
            {},
            { false },
            this,
        )

    init {
        SmartDashboard.putData(this)
        SmartDashboard.putData(brake())
        SmartDashboard.putData(coast())
    }
}
