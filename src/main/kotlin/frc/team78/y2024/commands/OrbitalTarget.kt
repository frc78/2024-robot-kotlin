// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team78.y2024.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import frc.team78.lib.MotionLimits
import frc.team78.y2024.lib.SPEAKER_POSE
import frc.team78.y2024.subsystems.chassis.Chassis
import frc.team78.y2024.subsystems.chassis.PoseEstimator

class OrbitalTarget(
    private val speedsSupplier: () -> ChassisSpeeds,
    motionLimits: MotionLimits,
) : Command() {

    val constraints =
        TrapezoidProfile.Constraints(
            motionLimits.maxTranslationVelocity,
            motionLimits.maxTranslationAcceleration
        )
    private var speakerPose: Translation2d? = null

    /** Set to 180 when on Red */
    private var allianceOffset: Rotation2d = Rotation2d.fromDegrees(0.0)

    init {
        addRequirements(Chassis)
    }

    override fun initialize() {
        speakerPose = SPEAKER_POSE
        allianceOffset =
            when (DriverStation.getAlliance().orElse(Alliance.Blue)) {
                Alliance.Red -> Rotation2d.fromDegrees(180.0)
                else -> Rotation2d.fromDegrees(0.0)
            }
    }

    override fun execute() {
        val robotPose = PoseEstimator.pose

        val goalPosition = robotPose.translation - speakerPose

        val commandedSpeeds = speedsSupplier()

        val speeds: ChassisSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                ChassisSpeeds(0.0, 0.0, commandedSpeeds.vxMetersPerSecond / goalPosition.norm),
                robotPose.rotation.plus(allianceOffset)
            )

        Chassis.driveRobotRelative(speeds, goalPosition)
    }
}
