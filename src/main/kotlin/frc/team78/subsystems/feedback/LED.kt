package frc.team78.subsystems.feedback

import com.ctre.phoenix.led.CANdle
import com.ctre.phoenix.led.CANdleConfiguration
import com.ctre.phoenix.led.ColorFlowAnimation
import com.ctre.phoenix.led.StrobeAnimation
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase

object LED : SubsystemBase() {
    private const val CANDLE_CAN_ID = 1

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

    private fun setColor(color: Color) {
        bracelet.apply {
            clearAnimation(0)
            setLEDs(color.red.toInt() * 255, color.green.toInt() * 255, color.blue.toInt() * 255)
        }
    }

    init {
        defaultCommand =
            startEnd(
                    {
                        when (DriverStation.getAlliance().orElse(Alliance.Blue)) {
                            Alliance.Red -> setColor(Color.kRed)
                            else -> setColor(Color.kBlue)
                        }
                    },
                    {}
                )
                .withName("Set Alliance Color")
    }

    fun indicateNoteInCartridge() =
        startEnd(
            {
                bracelet.apply {
                    clearAnimation(0)
                    setLEDs(0, 255, 0)
                }
            },
            this::off,
        )

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

    fun indicateShooterWheelsAtSpeed() =
        startEnd({ bracelet.animate(StrobeAnimation(0, 255, 0, 0, 0.2, 68)) }, this::off)

    private fun off() =
        bracelet.apply {
            clearAnimation(0)
            setLEDs(0, 0, 0)
        }
}
