package frc.team78.commands

import edu.wpi.first.wpilibj2.command.Command
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty

class CommandDelegate(private val command: () -> Command) : ReadOnlyProperty<Any?, Command> {
    override fun getValue(thisRef: Any?, property: KProperty<*>): Command {
        return command()
    }
}

fun command(command: () -> Command) = CommandDelegate(command)
