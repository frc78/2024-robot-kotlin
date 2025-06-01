package frc.team78.subsystems.feedback

import com.ctre.phoenix6.configs.CANdleConfiguration
import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.controls.StrobeAnimation
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.RGBWColor
import com.ctre.phoenix6.signals.StripTypeValue
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.idle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team78.commands.command
import frc.team78.lib.hertz

/** Subsystem for controlling the LEDs on the robot. */
object LED : SubsystemBase() {
    private const val CANDLE_CAN_ID = 1

    // The CANdle object for controlling the LEDs.
    private val bracelet =
        CANdle(CANDLE_CAN_ID, "CANivore").apply {
            configurator.apply(
                CANdleConfiguration().apply {
                    LED.BrightnessScalar = 0.5
                    LED.StripType = StripTypeValue.RGB
                }
            )
        }

    val solidColor = SolidColor(0, 8)

    /** Set the color of the entire LED strip */
    private fun setColor(color: Color) {
        bracelet.setControl(solidColor.withColor(RGBWColor(color)))
    }

    init {
        /**
         * When no command is running, set the LED strip to the color of the alliance.
         *
         * RunOnce and idle are used to ensure this command only sets the color once but does not
         * end, since default commands will be restarted when no other command is running.
         */
        defaultCommand =
            Commands.waitUntil { DriverStation.getAlliance().isPresent }
                .andThen(
                    runOnce {
                        when (DriverStation.getAlliance().orElse(Alliance.Blue)) {
                            Alliance.Red -> setColor(Color.kRed)
                            else -> setColor(Color.kBlue)
                        }
                    }
                )
                .andThen(idle())
                .withName("Set Alliance Color")
    }

    /**
     * Command to indicate the note is within the cartridge (shooter).
     *
     * This sets the leds to green
     */
    val indicateNoteInCartridge by command { startEnd({ setColor(Color.kGreen) }, this::off) }

    /** Command to indicate the shooter is up to speed and ready to fire. Strobes the leds green */
    val indicateShooterWheelsAtSpeed by command {
        startEnd(
            {
                bracelet.setControl(
                    StrobeAnimation(0, 8).withColor(RGBWColor(Color.kGreen)).withFrameRate(10.hertz)
                )
            },
            this::off,
        )
    }

    private fun off() {
        setColor(Color.kBlack)
    }
}
