package frc.team78.subsystems.chassis

import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import frc.team78.lib.inches
import frc.team78.lib.unaryMinus
import java.io.UncheckedIOException
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonTrackedTarget

object PoseEstimator {

    private val APRIL_TAG_FIELD_LAYOUT =
        try {
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
        } catch (e: UncheckedIOException) {
            DriverStation.reportError(e.message, false)
            AprilTagFieldLayout(emptyList(), 0.0, 0.0)
        }
    private val imu = Pigeon2(0)

    private val SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0)
    private val MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0)

    private val STERN_CAM = PhotonCamera("SternCam")
    private val STERN_CAM_POSE =
        Transform3d(
            Translation3d(-4.5.inches, 0.inches, 17.902.inches),
            Rotation3d(PI, Math.toRadians(-25.0), Math.toRadians(-30.0)),
        )

    private val sternCamPoseEstimator =
        PhotonPoseEstimator(
            APRIL_TAG_FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            STERN_CAM,
            STERN_CAM_POSE,
        )

    private val ntTable = NetworkTableInstance.getDefault().getTable("pose_estimator")
    private val posePublisher = ntTable.getStructTopic("Pose", Pose2d.struct).publish()

    private val gyroPub = ntTable.getStructTopic("Gyro", Rotation2d.struct).publish()

    fun update() {
        sternCamPoseEstimator.update().getOrNull()?.let {
            val pose = it.estimatedPose.toPose2d()

            val standardDeviations = getEstimatedStandardDeviations(pose, it.targetsUsed)
            SwerveDrive.addVisionMeasurement(pose, it.timestampSeconds, standardDeviations)

            posePublisher.set(pose)
        }

        gyroPub.set(imu.rotation2d)
    }

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
