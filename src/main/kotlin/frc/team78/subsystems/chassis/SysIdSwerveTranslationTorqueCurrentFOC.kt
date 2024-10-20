package frc.team78.subsystems.chassis

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Current
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure.mutable
import edu.wpi.first.units.Units.Amps

class SysIdSwerveTranslationTorqueCurrentFOC : SwerveRequest {
    /* Voltage to apply to drive wheels. This is final to enforce mutating the value */
    val torqueCurrentToApply = mutable(Amps.of(0.0))

    /* Local reference to a voltage request to drive the motors with */
    private val torqueCurrentRequest = TorqueCurrentFOC(0.0)

    override fun apply(
        parameters: SwerveRequest.SwerveControlRequestParameters,
        vararg modulesToApply: SwerveModule,
    ): StatusCode {
        for (module in modulesToApply) {
            module.applyCharacterization(
                Rotation2d.fromDegrees(0.0),
                torqueCurrentRequest.withOutput(torqueCurrentToApply.`in`(Amps)),
            )
        }
        return StatusCode.OK
    }

    /**
     * Update the current to apply to the drive wheels.
     *
     * @param current Amps to apply
     * @return this request
     */
    fun withCurrent(current: Measure<Current>): SysIdSwerveTranslationTorqueCurrentFOC {
        torqueCurrentToApply.mut_replace(current)
        return this
    }
}
