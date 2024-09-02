package frc.team78.lib

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Velocity

data class MotionLimits(
    val maxTranslationVelocity: Measure<Velocity<Distance>>,
    val maxAngularVelocity: Measure<Velocity<Angle>>,
    val maxTranslationAcceleration: Measure<Velocity<Velocity<Distance>>>,
    val maxAngularAcceleration: Measure<Velocity<Velocity<Angle>>>,
)
