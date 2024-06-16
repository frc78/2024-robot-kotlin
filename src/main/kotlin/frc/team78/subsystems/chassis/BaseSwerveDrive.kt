package frc.team78.subsystems.chassis

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.XboxController
import frc.team78.lib.MotionLimits

class BaseSwerveDrive(
    private val controller: XboxController,
    private val motionLimits: MotionLimits
) {

    private fun triggerAdjust(slow: Double, fast: Double) = 0.5 + fast * 0.5 - slow * 0.25

    val chassisSpeeds: ChassisSpeeds
        get() {
            val leftTriggerDeadbanded = MathUtil.applyDeadband(controller.leftTriggerAxis, 0.1)
            val rightTriggerDeadbanded = MathUtil.applyDeadband(controller.rightTriggerAxis, 0.1)

            val speedAdjustment = triggerAdjust(leftTriggerDeadbanded, rightTriggerDeadbanded)

            val maxSpeed = motionLimits.maxTranslationVelocity * speedAdjustment
            val maxAngularSpeed = motionLimits.maxAngularVelocity * speedAdjustment

            val x = maxSpeed * -controller.leftY
            val y = maxSpeed * -controller.leftX
            val rot = maxAngularSpeed * -controller.rightX
            return ChassisSpeeds(x, y, rot)
        }
}
