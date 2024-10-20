package frc.team78.lib

import edu.wpi.first.units.*
import edu.wpi.first.units.Unit
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts

/**
 * Helper extension function to use the negative operator on a Measure instance
 *
 * Example: -Volts.of(1)
 */
operator fun <U : Unit<U>> Measure<U>.unaryMinus(): Measure<U> = this.negate()

/** Helper function to use the division operator on a Measure instance */
operator fun <U : Unit<U>> Measure<U>.div(divisor: Number): Measure<U> =
    this.divide(divisor.toDouble())

/**
 * Helper functions to create common Measures
 *
 * Usage: 5.inches
 */
val Number.inches: Measure<Distance>
    get() = Inches.of(this.toDouble())
val Number.meters: Measure<Distance>
    get() = Meters.of(this.toDouble())
val Number.seconds: Measure<Time>
    get() = Seconds.of(this.toDouble())
val Number.metersPerSecond: Measure<Velocity<Distance>>
    get() = MetersPerSecond.of(this.toDouble())
val Number.volts: Measure<Voltage>
    get() = Volts.of(this.toDouble())
val Number.amps: Measure<Current>
    get() = Amps.of(this.toDouble())
val Number.metersPerSecondPerSecond: Measure<Velocity<Velocity<Distance>>>
    get() = MetersPerSecond.of(this.toDouble()).per(1.seconds)
val Number.radiansPerSecond: Measure<Velocity<Angle>>
    get() = RadiansPerSecond.of(this.toDouble())
val Number.radiansPerSecondPerSecond: Measure<Velocity<Velocity<Angle>>>
    get() = RadiansPerSecond.of(this.toDouble()).per(1.seconds)
val Number.degrees: Measure<Angle>
    get() = Degrees.of(this.toDouble())

val Measure<Distance>.inches
    get() = this.`in`(Inches)
val Measure<Angle>.rotations
    get() = this.`in`(Rotations)
val Measure<Velocity<Distance>>.metersPerSecond
    get() = this.`in`(MetersPerSecond)
val Measure<Velocity<Angle>>.radiansPerSecond
    get() = this.`in`(RadiansPerSecond)
val Measure<Velocity<Angle>>.rotationsPerSecond
    get() = this.`in`(RotationsPerSecond)
val Measure<Voltage>.volts
    get() = this.`in`(Volts)
val Measure<Angle>.radians
    get() = this.`in`(Radians)
