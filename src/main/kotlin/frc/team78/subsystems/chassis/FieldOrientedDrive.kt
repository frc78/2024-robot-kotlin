// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team78.subsystems.chassis

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import kotlin.jvm.optionals.getOrNull

class FieldOrientedDrive(private val speeds: () -> ChassisSpeeds) : Command() {

    /**
     * Robot rotation offset needed by ChassisSpeeds#fromFieldRelativespeeds.
     *
     * Set to 180 when on Red
     */
    private var allianceOffset = Rotation2d.fromDegrees(0.0)

    /** Creates a new FieldOrientedDrive. */
    init {
        addRequirements(Chassis)
    }

    override fun initialize() {
        allianceOffset =
            when (DriverStation.getAlliance().getOrNull()) {
                Alliance.Red -> Rotation2d.fromDegrees(180.0)
                else -> Rotation2d.fromDegrees(0.0)
            }
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        val chassisSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds(),
                PoseEstimator.pose.rotation.plus(allianceOffset),
            )
        Chassis.drive(chassisSpeeds)
    }
}
