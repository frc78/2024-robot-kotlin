package frc.team78.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty

class CommandDelegate(private val command: () -> Command) : ReadOnlyProperty<Any?, Command> {
    override fun getValue(thisRef: Any?, property: KProperty<*>): Command {
        val command = command()
        if (command.name == command.javaClass.simpleName.substringAfter(".")) {
            command.name = property.name
        }
        SmartDashboard.putData(command)
        return command
    }
}

fun command(command: () -> Command) = CommandDelegate(command)
