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

    private boolean inRange = false;   // Within range to shoot
    private boolean climbing = false;  // In the process of auto climbing
    private boolean aligning = false;   // Aligned to pose (false = blinking)
    private LEDPattern basePattern;

    public LEDSubsystem() {
        led = new AddressableLED(LEDConstants.PWM_PORT);

        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);

        led.start();
    }

    public void setInRange(boolean inRange) {
        this.inRange = inRange;
    }

    public void setClimbing(boolean climbing) {
        this.climbing = climbing;
    }

    public void setAligning(boolean aligning) {
        this.aligning = aligning;
    }

    private void setPattern(LEDPattern pattern) {
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    private void setBlueYellow() {
        LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kBlue);
        gradient.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setBlueYellow();
            return;
        }

        if (climbing) {
            basePattern = LEDPattern.solid(Color.kBlue);
        } else if (inRange) {
            basePattern = LEDPattern.solid(Color.kGreen);
        } else {
            basePattern = LEDPattern.solid(Color.kRed);
        }

        if (aligning) {
            setPattern(basePattern.blink(Seconds.of(0.5)));
        } else {
            setPattern(basePattern);
        }
    }
}