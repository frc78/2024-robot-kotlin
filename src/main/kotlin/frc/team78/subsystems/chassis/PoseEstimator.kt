package frc.team78.subsystems.chassis

import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.networktables.NetworkTableInstance
import frc.team78.lib.inches
import frc.team78.lib.unaryMinus
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget

object PoseEstimator {

    private val APRIL_TAG_FIELD_LAYOUT =
        try {
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
        } catch (e: Exception) {
            AprilTagFieldLayout(emptyList(), 0.0, 0.0)
        }
    private val imu = Pigeon2(0)

    private val ODOMETRY_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1)
    private val VISION_STD_DEVS = VecBuilder.fill(1.0, 1.0, 1.5)
    private val SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0)
    private val MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0)

    private val STERN_CAM = PhotonCamera("SternCam")
    private val STERN_CAM_POSE =
        Transform3d(
            Translation3d(-4.5.inches, 0.inches, 17.902.inches),
            Rotation3d(PI, Math.toRadians(-25.0), Math.toRadians(-30.0)),
        )

    private val PORT_CAM = PhotonCamera("PortCam")
    private val PORT_CAM_POSE =
        Transform3d(
            Translation3d(4.465.inches, 10.205.inches, 21.274.inches),
            Rotation3d(0.0, Math.toRadians(-25.0), Math.toRadians(30.0)),
        )

    private val STARBOARD_CAM = PhotonCamera("StarboardCam")
    private val STARBOARD_CAM_POSE =
        Transform3d(
            Translation3d(4.465.inches, -10.205.inches, 21.274.inches),
            Rotation3d(PI, Math.toRadians(-25.0), Math.toRadians(-30.0)),
        )

    private val sternCamPoseEstimator =
        PhotonPoseEstimator(
            APRIL_TAG_FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            STERN_CAM,
            STERN_CAM_POSE,
        )

    private val portCamPoseEstimator =
        PhotonPoseEstimator(
            APRIL_TAG_FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PORT_CAM,
            PORT_CAM_POSE,
        )

    private val starboardCamPoseEstimator =
        PhotonPoseEstimator(
            APRIL_TAG_FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            STARBOARD_CAM,
            STARBOARD_CAM_POSE,
        )
    private val visionPoseEstimators =
        listOf(sternCamPoseEstimator, portCamPoseEstimator, starboardCamPoseEstimator)

    private val ntTable = NetworkTableInstance.getDefault().getTable("pose_estimator")
    private val ntPublishers =
        arrayOf(
            ntTable.getStructTopic("SternPose", Pose2d.struct).publish(),
            ntTable.getStructTopic("PortPose", Pose2d.struct).publish(),
            ntTable.getStructTopic("StarboardPose", Pose2d.struct).publish(),
        )

    private val gyroPub = ntTable.getStructTopic("Gyro", Rotation2d.struct).publish()
    private val poseEstimatePub = ntTable.getStructTopic("Chassis Pose", Pose2d.struct).publish()

    private val swerveDrivePoseEstimator =
        SwerveDrivePoseEstimator(
            Chassis.kinematics,
            imu.rotation2d,
            Chassis.positions,
            Pose2d(),
            ODOMETRY_STD_DEVS,
            VISION_STD_DEVS,
        )

    fun update() {
        visionPoseEstimators.forEachIndexed { i, it ->
            it.update().getOrNull()?.let {
                val pose = it.estimatedPose.toPose2d()

                val standardDeviations = getEstimatedStandardDeviations(pose, it.targetsUsed)
                swerveDrivePoseEstimator.addVisionMeasurement(
                    pose,
                    it.timestampSeconds,
                    standardDeviations,
                )

                ntPublishers[i].set(pose)
            }
        }

        swerveDrivePoseEstimator.update(imu.rotation2d, Chassis.positions)
        poseEstimatePub.set(swerveDrivePoseEstimator.estimatedPosition)
        gyroPub.set(imu.rotation2d)
    }

    val pose: Pose2d
        get() = swerveDrivePoseEstimator.estimatedPosition

    fun resetPose(pose: Pose2d) =
        swerveDrivePoseEstimator.resetPosition(imu.rotation2d, Chassis.positions, pose)

    private fun getEstimatedStandardDeviations(
        pose: Pose2d,
        targetsUsed: List<PhotonTrackedTarget>,
    ): Vector<N3> {

        var numTargets: Int
        val averageDistanceToTargets =
            targetsUsed
                .map { APRIL_TAG_FIELD_LAYOUT.getTagPose(it.fiducialId) }
                .filter { it.isPresent }
                // Avoid divide by zero
                .also { numTargets = if (it.isEmpty()) 1 else it.size }
                .sumOf { it.get().translation.getDistance(Pose3d(pose).translation) } / numTargets

        val standardDeviations =
            when (numTargets) {
                1 -> SINGLE_TAG_STD_DEVS
                else -> MULTI_TAG_STD_DEVS
            }
        return standardDeviations * (1 + (averageDistanceToTargets * averageDistanceToTargets / 30))
    }
}
