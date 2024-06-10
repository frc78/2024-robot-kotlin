package frc.team78.lib

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts

fun CANSparkBase.setStatusRates(
    period0: Int = 32767,
    period1: Int = 32767,
    period2: Int = 32767,
    period3: Int = 32767,
    period4: Int = 32767,
    period5: Int = 32767,
    period6: Int = 32767,
    period7: Int = 32767
) {
    setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, period0)
    setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, period1)
    setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, period2)
    setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, period3)
    setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, period4)
    setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, period5)
    setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, period6)
    setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus7, period7)
}

fun CANSparkBase.setSingleStatusRate(frame: CANSparkLowLevel.PeriodicFrame, period: Int) {
    setPeriodicFramePeriod(frame, period)
}

operator fun <U : Unit<U>> Measure<U>.unaryMinus(): Measure<U> = this.negate()

operator fun <U : Unit<U>> Measure<U>.div(divisor: Number) = this.divide(divisor.toDouble())

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

fun <U : Unit<U>> U.contains(measure: Measure<U>): Double {
    return measure.`in`(this)
}
