// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team78.y2024.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import frc.team78.lib.meters
import frc.team78.y2024.lib.SPEAKER_HEIGHT
import frc.team78.y2024.lib.SPEAKER_POSE
import frc.team78.y2024.subsystems.chassis.PoseEstimator
import frc.team78.y2024.subsystems.elevator.Elevator
import frc.team78.y2024.subsystems.shooter.Shooter
import frc.team78.y2024.subsystems.wrist.Wrist
import kotlin.math.PI
import kotlin.math.atan
import kotlin.math.sqrt

class VarShootPrime : Command() {
    private var speakerTranslation = Translation2d()

    // Translation of where the note exits in the XZ plane (side view)

    /** Creates a new VarShootPrime. */
    init {
        addRequirements(Wrist)
    }

    override fun initialize() {
        speakerTranslation = SPEAKER_POSE
    }

    override fun execute() {
        val pose = PoseEstimator.pose

        // Distance and height to speaker
        val l = pose.translation.getDistance(speakerTranslation)
        val h =
            SPEAKER_HEIGHT.magnitude() -
                SHOOTER_HEIGHT.magnitude() -
                Units.inchesToMeters(Elevator.position)
        // Calculate velocity based on lerping within the velocity range based on the distance range
        // double v = Util.lerp(Util.clamp(h, distRange) / distRange.getRange(), velRange);
        val v: Double = Shooter.velocity * RPM_TO_MPS
        var theta = calcTheta(l, h, v)
        if (theta.isNaN()) theta = DEFAULT_SHOT_ANGLE
        theta = Units.radiansToDegrees(theta)

        Wrist.setTarget(theta)
    }

    // Source? It was revealed to me by a wise tree in a dream
    // JK this https://en.wikipedia.org/wiki/Projectile_motion
    private fun calcTheta(l: Double, h: Double, v: Double): Double {
        val sqrt: Double = v * v * v * v - (GRAVITY * ((GRAVITY * l * l) + (2 * h * v * v)))
        val numerator = (v * v) - sqrt(sqrt)
        val denominator = GRAVITY * l

        return atan(numerator / denominator)
    }

    companion object {
        private const val GRAVITY = 9.81
        private val RPM_TO_MPS = 2 * Units.inchesToMeters(3.85) * PI / 60
        private val SHOOTER_HEIGHT = 0.56.meters
        private const val DEFAULT_SHOT_ANGLE = 55.0
    }
}
