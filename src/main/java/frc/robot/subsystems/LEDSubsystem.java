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
    private boolean tracking;
    private boolean aligned;

    public LEDSubsystem() {
        led = new AddressableLED(LEDConstants.PWM);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_BUFFER);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        aligned = false;
        tracking = false;
        led.start();
    }

    public void setAligned (boolean aligned) {
        this.aligned = aligned;
    }

    private void setBlueYellow() {
        LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kBlue);
        gradient.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    public void setTracking(boolean tracking) {
        this.tracking = tracking;
    }

    private void setColor(LEDPattern pattern) {
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    public void periodic() {
        if (tracking) {
            if (aligned) {
                setColor(LEDPattern.solid(Color.kGreen).blink(Seconds.of(1)));
            } else {
                setColor(LEDPattern.solid(Color.kRed).blink(Seconds.of(1)));
            }
        } else {
            if (aligned) {
                setColor(LEDPattern.solid(Color.kGreen));
            } else {
                setColor(LEDPattern.solid(Color.kRed));
            }
        }
    }
}