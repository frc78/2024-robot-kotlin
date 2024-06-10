// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team78.y2024.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.Command
import frc.team78.y2024.lib.PLOP_POSE
import frc.team78.y2024.subsystems.chassis.PoseEstimator
import frc.team78.y2024.subsystems.elevator.Elevator
import frc.team78.y2024.subsystems.shooter.Shooter
import frc.team78.y2024.subsystems.wrist.Wrist
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/** @param shooterXZTrans Position of the shooter in the XZ plane */
class VarFeedPrime(
    private val shooterXZTrans: Translation2d,
) : Command() {

    private var plopPose = Translation2d()

    /** Creates a new VarShootPrime. */
    init {
        addRequirements(Shooter)
    }

    override fun initialize() {
        // Reinitialize the plop pose in case the alliance changes
        plopPose = PLOP_POSE
    }

    override fun execute() {
        val pose = PoseEstimator.pose

        // Distance and height to speaker
        var distanceToTarget = pose.translation.getDistance(plopPose) - shooterXZTrans.x
        distanceToTarget *= DIST_FUDGE_FACTOR

        // double distanceToTarget = lInput.getDouble(3);
        var heightToTarget: Double = shooterXZTrans.y - inchesToMeters(Elevator.position)
        // Inverts the heigh as we are shooting from the robot to the ground, but the calculations
        // are
        // always done from (0, 0) so we use this as our offset
        heightToTarget = -heightToTarget

        val theta = Math.toRadians(Wrist.position)

        val calcVel = calcVel(distanceToTarget, heightToTarget, theta)
        // Safety for NaN, probably should put this in the setSpeed() itself though
        val mps = if (calcVel.isNaN()) 0.0 else calcVel

        Shooter.setSpeed(RotationsPerSecond.of(mps * ROTATIONS_PER_METER))
    }

    override fun end(interrupted: Boolean) {
        Shooter.setSpeed(RotationsPerSecond.of(0.0))
    }

    // Source? It was revealed to me by a wise tree in a dream
    // JK this https://en.wikipedia.org/wiki/Projectile_motion
    private fun calcVel(l: Double, h: Double, a: Double): Double {
        val nominator = l * l * GRAVITY
        val denominator = l * sin(2 * a) - 2 * h * cos(a) * cos(a)
        return sqrt(nominator / denominator)
    }

    companion object {
        private const val GRAVITY = 9.81
        private val ROTATIONS_PER_METER = 1 / inchesToMeters(3.85) * PI
        private const val DIST_FUDGE_FACTOR = 1
    }
}
