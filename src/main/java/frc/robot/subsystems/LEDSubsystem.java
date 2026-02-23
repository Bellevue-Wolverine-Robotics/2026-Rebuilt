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

    private boolean tracking = false;
    private boolean aligned = false;

    public LEDSubsystem() {
        led = new AddressableLED(LEDConstants.PWM_PORT);

        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);

        led.start();
    }

    public void setAligned(boolean aligned) {
        this.aligned = aligned;
    }

    public void setTracking(boolean tracking) {
        this.tracking = tracking;
    }

    private void setBlueYellow() {
        LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kBlue);
        gradient.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    private void setPattern(LEDPattern pattern) {
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setBlueYellow();
            return;
        }

        if (tracking) {
            if (aligned) {
                setPattern(LEDPattern.solid(Color.kGreen).blink(Seconds.of(1)));
            } else {
                setPattern(LEDPattern.solid(Color.kRed).blink(Seconds.of(1)));
            }
        } else {
            if (aligned) {
                setPattern(LEDPattern.solid(Color.kGreen));
            } else {
                setPattern(LEDPattern.solid(Color.kRed));
            }
        }
    }
}