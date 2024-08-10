package frc.team78.lib

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts

/**
 * Helper extension function to use the negative operator on a Measure instance
 *
 * Example: -Volts.of(1)
 */
operator fun <U : Unit<U>> Measure<U>.unaryMinus(): Measure<U> = this.negate()

/** Helper function to use the division operator on a Measure instance */
operator fun <U : Unit<U>> Measure<U>.div(divisor: Number) = this.divide(divisor.toDouble())

/**
 * Helper functions to create common Measures
 *
 * Usage: 5.inches
 */
val Number.inches
    get() = Inches.of(this.toDouble())
val Number.meters
    get() = Meters.of(this.toDouble())
val Number.seconds
    get() = Seconds.of(this.toDouble())
val Number.metersPerSecond
    get() = MetersPerSecond.of(this.toDouble())
val Number.volts
    get() = Volts.of(this.toDouble())
val Number.metersPerSecondPerSecond
    get() = MetersPerSecond.of(this.toDouble()).per(1.seconds)
val Number.radiansPerSecond
    get() = RadiansPerSecond.of(this.toDouble())
val Number.radiansPerSecondPerSecond
    get() = RadiansPerSecond.of(this.toDouble()).per(1.seconds)
