// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team78.subsystems.chassis

import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Radian
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command

class FieldOrientedWithCardinal(
    private val angle: () -> Double,
    private val speedsSupplier: () -> ChassisSpeeds,
    cardinalPidConstants: PIDConstants,
    constraints: TrapezoidProfile.Constraints,
    private val threshold: Measure<Angle>,
) : Command() {

    /** Set to 180 when on Red */
    private var allianceOffset = Rotation2d.fromDegrees(0.0)

    private val thetaPID =
        ProfiledPIDController(
                cardinalPidConstants.kP,
                cardinalPidConstants.kI,
                cardinalPidConstants.kD,
                constraints,
            )
            .apply {
                enableContinuousInput(-Math.PI, Math.PI)
                setTolerance(threshold.`in`(Radian))
            }

    init {
        addRequirements(Chassis)
    }

    override fun initialize() {
        allianceOffset =
            if (
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
                    DriverStation.Alliance.Red
            ) {
                Rotation2d.fromDegrees(180.0)
            } else {
                Rotation2d.fromDegrees(0.0)
            }
        thetaPID.reset(
            PoseEstimator.pose.rotation.radians,
            Chassis.kinematics.toChassisSpeeds(*Chassis.states).omegaRadiansPerSecond,
        )
    }

    override fun execute() {
        val speeds = speedsSupplier()
        thetaPID.setGoal(angle())

        val cardinalRotSpeed: Double = thetaPID.calculate(PoseEstimator.pose.rotation.radians)
        speeds.omegaRadiansPerSecond = cardinalRotSpeed

        Chassis.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                PoseEstimator.pose.rotation.plus(allianceOffset),
            )
        )
    }

    override fun isFinished(): Boolean {
        return threshold != Degrees.zero() && thetaPID.atGoal()
    }

    override fun end(interrupted: Boolean) {
        Chassis.drive(ChassisSpeeds())
    }
}
