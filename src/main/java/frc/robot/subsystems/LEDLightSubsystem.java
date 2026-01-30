package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.LEDConstants;

public class LEDLightSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private int animationOffset = 0;
    private int loopCounter = 0;
    private boolean alignmentStatus = false;

    public LEDLightSubsystem() {
        led = new AddressableLED(LEDConstants.LED_PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_STRIP_LENGTH);
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    public void isAligned(boolean isAligned) {
        alignmentStatus = isAligned;

        if (!DriverStation.isDisabled()) {
            if (isAligned) {
                setGreen();
            } else {
                setRed();
            }
        }
    }

    private void updateGradientFlow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            int position = (i + animationOffset) % LEDConstants.PATTERN_LENGTH;
            double progress = (double) position / LEDConstants.PATTERN_LENGTH;

            int red, green, blue;

            if (progress < 0.5) {
                double blend = progress * 2.0;
                red = (int) (LEDConstants.COLOR_YELLOW[0] * blend);
                green = (int) (LEDConstants.COLOR_YELLOW[1] * blend);
                blue = (int) (LEDConstants.COLOR_BLUE[2] * (1 - blend));
            } else {
                double blend = (progress - 0.5) * 2.0;
                red = (int) (LEDConstants.COLOR_YELLOW[0] * (1 - blend));
                green = (int) (LEDConstants.COLOR_YELLOW[1] * (1 - blend));
                blue = (int) (LEDConstants.COLOR_BLUE[2] * blend);
            }

            ledBuffer.setRGB(i, red, green, blue);
        }
    }

    private void setGreen() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 255, 0);
        }
    }

    private void setRed() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 0, 0);
        }
    }

    public void setBlue() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i,
                    LEDConstants.COLOR_BLUE[0],
                    LEDConstants.COLOR_BLUE[1],
                    LEDConstants.COLOR_BLUE[2]);
        }
    }

    public void setYellow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i,
                    LEDConstants.COLOR_YELLOW[0],
                    LEDConstants.COLOR_YELLOW[1],
                    LEDConstants.COLOR_YELLOW[2]);
        }
    }

    public void turnOff() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            loopCounter++;
            if (loopCounter >= LEDConstants.ANIMATION_SPEED) {
                loopCounter = 0;
                animationOffset++;

                if (animationOffset >= LEDConstants.PATTERN_LENGTH * 10) {
                    animationOffset = 0;
                }

                updateGradientFlow();
            }
        } else {
            if (alignmentStatus) {
                setGreen();
            } else {
                setRed();
            }
        }

        led.setData(ledBuffer);
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public boolean getAlignmentStatus() {
        return alignmentStatus;
    }
}