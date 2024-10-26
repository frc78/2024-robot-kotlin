package frc.team78.lib

import edu.wpi.first.wpilibj.DriverStation
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty

/** Delegate to get a value based on the current alliance */
class Alliance<T>(val blueValue: T, val redValue: T) : ReadOnlyProperty<Any?, T> {
    override fun getValue(thisRef: Any?, property: KProperty<*>): T {
        return if (
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
                DriverStation.Alliance.Red
        ) {
            redValue
        } else {
            blueValue
        }
    }
}
