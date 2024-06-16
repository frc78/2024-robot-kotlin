package frc.team78.lib

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance

val SPEAKER_POSE
    get() =
        when (DriverStation.getAlliance().orElse(Alliance.Blue)) {
            Alliance.Red -> Translation2d(16.5.meters, 5.5.meters)
            else -> Translation2d(0.meters, 5.5.meters)
        }

val PLOP_POSE
    get() =
        when (DriverStation.getAlliance().orElse(Alliance.Blue)) {
            Alliance.Red -> Translation2d(16.5.meters, 7.meters)
            else -> Translation2d(0.meters, 7.meters)
        }

val SPEAKER_HEIGHT = 2.1.meters
