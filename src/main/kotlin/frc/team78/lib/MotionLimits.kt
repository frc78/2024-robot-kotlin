package frc.team78.lib

import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity

data class MotionLimits(
    val maxTranslationVelocity: LinearVelocity,
    val maxAngularVelocity: AngularVelocity,
    val maxTranslationAcceleration: LinearAcceleration,
    val maxAngularAcceleration: AngularAcceleration,
)
