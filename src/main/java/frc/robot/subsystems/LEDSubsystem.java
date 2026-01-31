package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

import static edu.wpi.first.units.Units.Seconds;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private LEDPattern pattern;

    public LEDSubsystem() {
        led = new AddressableLED(LEDConstants.PWM);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_BUFFER);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        pattern = LEDPattern.solid(Color.kRed);
        led.start();
    }

    public void isAligned (boolean isAlign) {
        if (isAlign) {
            pattern = LEDPattern.solid(Color.kGreen);
        } else {
            pattern = LEDPattern.solid(Color.kRed);
        }
    }

    private void setBlueYellow() {
        LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kBlue);
        gradient.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    public void isTracking(boolean on) {
        if (on){
            pattern = pattern.blink(Seconds.of(1));
        }else {
            pattern = LEDPattern.solid(Color.kRed);
        }
    }

    public void periodic() {
        if (DriverStation.isDisabled()) {
            setBlueYellow();
        } else {
            pattern.applyTo(ledBuffer);
            led.setData(ledBuffer);
        }
    }
}
