package frc.team78.subsystems.feedback

import com.ctre.phoenix.led.CANdle
import com.ctre.phoenix.led.CANdleConfiguration
import com.ctre.phoenix.led.ColorFlowAnimation
import com.ctre.phoenix.led.StrobeAnimation
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Commands.idle
import edu.wpi.first.wpilibj2.command.SubsystemBase

/** Subsystem for controlling the LEDs on the robot. */
object LED : SubsystemBase() {
    private const val CANDLE_CAN_ID = 1

    // The CANdle object for controlling the LEDs.
    private val bracelet =
        CANdle(CANDLE_CAN_ID).apply {
            configFactoryDefault()
            val config =
                CANdleConfiguration().apply {
                    brightnessScalar = 0.5
                    stripType = CANdle.LEDStripType.RGB
                }
            configAllSettings(config)
        }

    /** Set the color of the entire LED strip */
    private fun setColor(color: Color) {
        bracelet.apply {
            setLEDs(color.red.toInt() * 255, color.green.toInt() * 255, color.blue.toInt() * 255)
        }
    }

    init {
        /**
         * When no command is running, set the LED strip to the color of the alliance.
         *
         * RunOnce and idle are used to ensure this command only sets the color once but does not
         * end, since default commands will be restarted when no other command is running.
         */
        defaultCommand =
            runOnce {
                    when (DriverStation.getAlliance().orElse(Alliance.Blue)) {
                        Alliance.Red -> setColor(Color.kRed)
                        else -> setColor(Color.kBlue)
                    }
                }
                .andThen(idle())
                .withName("Set Alliance Color")
    }

    /**
     * Command to indicate the note is within the cartridge (shooter).
     *
     * This sets the leds to green
     */
    fun indicateNoteInCartridge() =
        startEnd(
            { bracelet.apply { setLEDs(0, 255, 0) } },
            this::off,
        )

    /** Command to run an animation when the robot is disabled */
    fun indicateDisabled() = runOnce {
        bracelet.apply {
            animate(
                ColorFlowAnimation(
                    0,
                    0,
                    255,
                    0,
                    0.1,
                    15,
                    ColorFlowAnimation.Direction.Forward,
                    7,
                )
            )
            animate(
                ColorFlowAnimation(
                    0,
                    255,
                    0,
                    0,
                    0.1,
                    15,
                    ColorFlowAnimation.Direction.Backward,
                    7,
                )
            )
            animate(
                ColorFlowAnimation(
                    0,
                    0,
                    255,
                    0,
                    0.1,
                    7,
                    ColorFlowAnimation.Direction.Forward,
                    0,
                )
            )
            animate(
                ColorFlowAnimation(
                    0,
                    255,
                    0,
                    0,
                    0.1,
                    7,
                    ColorFlowAnimation.Direction.Backward,
                    0,
                )
            )
        }
    }

    /** Command to indicate the shooter is up to speed and ready to fire. Strobes the leds green */
    fun indicateShooterWheelsAtSpeed() =
        startEnd({ bracelet.animate(StrobeAnimation(0, 255, 0, 0, 0.2, 68)) }, this::off)

    private fun off() =
        bracelet.apply {
            clearAnimation(0)
            setLEDs(0, 0, 0)
        }
}
