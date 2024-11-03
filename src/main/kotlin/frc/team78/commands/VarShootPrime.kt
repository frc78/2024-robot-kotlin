// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team78.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import frc.team78.lib.SPEAKER_HEIGHT
import frc.team78.lib.SPEAKER_POSE
import frc.team78.lib.inches
import frc.team78.lib.meters
import frc.team78.lib.metersPerSecond
import frc.team78.lib.toLinearVelocity
import frc.team78.subsystems.chassis.SwerveDrive
import frc.team78.subsystems.elevator.Elevator
import frc.team78.subsystems.shooter.Shooter
import frc.team78.subsystems.wrist.Wrist
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
        val pose = SwerveDrive.estimatedPose

        // Distance and height to speaker
        val l = pose.translation.getDistance(speakerTranslation).meters
        val h = SPEAKER_HEIGHT - SHOOTER_HEIGHT - Elevator.position
        // Calculate velocity based on lerping within the velocity range based on the distance range
        // double v = Util.lerp(Util.clamp(h, distRange) / distRange.getRange(), velRange);
        val v = Shooter.velocity.toLinearVelocity(SHOOTER_WHEEL_DIAMETER)
        var theta = calcTheta(l, h, v)
        if (theta.isNaN()) theta = DEFAULT_SHOT_ANGLE

        Wrist.setTarget(Radians.of(theta))
    }

    // Source? It was revealed to me by a wise tree in a dream
    // JK this https://en.wikipedia.org/wiki/Projectile_motion
    private fun calcTheta(l: Distance, h: Distance, v: LinearVelocity): Double {
        val l_mag = l.meters
        val h_mag = h.meters
        val v_mag = v.metersPerSecond
        val sqrt: Double =
            v_mag * v_mag * v_mag * v_mag -
                (GRAVITY * ((GRAVITY * l_mag * l_mag) + (2 * h_mag * v_mag * v_mag)))
        val numerator = (v_mag * v_mag) - sqrt(sqrt)
        val denominator = GRAVITY * l_mag

        return atan(numerator / denominator)
    }

    companion object {
        private const val GRAVITY = 9.81
        private val SHOOTER_WHEEL_DIAMETER = 3.85.inches
        private val SHOOTER_HEIGHT = 0.56.meters
        private const val DEFAULT_SHOT_ANGLE = 55.0
    }
}
