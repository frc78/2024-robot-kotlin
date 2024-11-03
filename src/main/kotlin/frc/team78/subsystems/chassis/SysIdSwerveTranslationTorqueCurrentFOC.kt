package frc.team78.subsystems.chassis

import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.swerve.SwerveDrivetrain
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Current
import frc.team78.lib.amps

class SysIdSwerveTranslationTorqueCurrentFOC : SwerveRequest {
    /* Voltage to apply to drive wheels. This is final to enforce mutating the value */
    val torqueCurrentToApply = Amps.of(0.0).mutableCopy()

    /* Local reference to a voltage request to drive the motors with */
    private val driveRequest = TorqueCurrentFOC(0.0)
    private val steerRequest = PositionVoltage(0.0)

    override fun apply(
        parameters: SwerveDrivetrain.SwerveControlParameters,
        vararg modulesToApply: SwerveModule,
    ): StatusCode {
        for (module in modulesToApply) {
            module.apply(driveRequest.withOutput(torqueCurrentToApply.amps), steerRequest)
        }
        return StatusCode.OK
    }

    /**
     * Update the current to apply to the drive wheels.
     *
     * @param current Amps to apply
     * @return this request
     */
    fun withCurrent(current: Current): SysIdSwerveTranslationTorqueCurrentFOC {
        torqueCurrentToApply.mut_replace(current)
        return this
    }
}
