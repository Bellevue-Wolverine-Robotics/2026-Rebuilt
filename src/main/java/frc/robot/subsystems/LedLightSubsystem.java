package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.LedConstants;

public class LedLightSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    // Animation variables
    private int animationOffset = 0;
    private int loopCounter = 0;

    // Alignment state
    private boolean alignmentStatus = false;

    public LedLightSubsystem() {
        // Initialize the LED strip
        led = new AddressableLED(LedConstants.LED_PWM_PORT);

        // Create the buffer for the LED data
        ledBuffer = new AddressableLEDBuffer(LedConstants.LED_STRIP_LENGTH);

        // Set the length of the LED strip
        led.setLength(ledBuffer.getLength());

        // Start the LED output
        led.start();

        System.out.println("LED Subsystem initialized - Flowing when disabled, alignment when enabled");
    }

    /**
     * Set the alignment status and update LED color
     * @param isAligned true = green LEDs, false = red LEDs
     */
    public void isAligned(boolean isAligned) {
        alignmentStatus = isAligned;

        // Immediately update LEDs if robot is enabled
        if (!DriverStation.isDisabled()) {
            if (isAligned) {
                setGreen();
            } else {
                setRed();
            }
        }
    }

    /**
     * Creates a smooth gradient flowing pattern between blue and yellow
     */
    private void updateGradientFlow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate position in pattern with animation offset
            int position = (i + animationOffset) % LedConstants.PATTERN_LENGTH;

            // Calculate how far we are through the pattern (0.0 to 1.0)
            double progress = (double) position / LedConstants.PATTERN_LENGTH;

            int red, green, blue;

            if (progress < 0.5) {
                // First half: transition from blue to yellow
                double blend = progress * 2.0; // 0.0 to 1.0
                red = (int) (LedConstants.COLOR_YELLOW[0] * blend);
                green = (int) (LedConstants.COLOR_YELLOW[1] * blend);
                blue = (int) (LedConstants.COLOR_BLUE[2] * (1 - blend));
            } else {
                // Second half: transition from yellow to blue
                double blend = (progress - 0.5) * 2.0; // 0.0 to 1.0
                red = (int) (LedConstants.COLOR_YELLOW[0] * (1 - blend));
                green = (int) (LedConstants.COLOR_YELLOW[1] * (1 - blend));
                blue = (int) (LedConstants.COLOR_BLUE[2] * blend);
            }

            ledBuffer.setRGB(i, red, green, blue);
        }
    }

    /**
     * Sets all LEDs to solid green (aligned)
     */
    private void setGreen() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 255, 0);
        }
    }

    /**
     * Sets all LEDs to solid red (not aligned)
     */
    private void setRed() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 0, 0);
        }
    }

    /**
     * Sets all LEDs to solid blue
     */
    public void setBlue() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i,
                    LedConstants.COLOR_BLUE[0],
                    LedConstants.COLOR_BLUE[1],
                    LedConstants.COLOR_BLUE[2]);
        }
    }

    /**
     * Sets all LEDs to solid yellow
     */
    public void setYellow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i,
                    LedConstants.COLOR_YELLOW[0],
                    LedConstants.COLOR_YELLOW[1],
                    LedConstants.COLOR_YELLOW[2]);
        }
    }

    /**
     * Turns off all LEDs
     */
    public void turnOff() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    @Override
    public void periodic() {
        // Check robot state and display appropriate pattern
        if (DriverStation.isDisabled()) {
            // Robot is DISABLED - show flowing blue/yellow animation
            loopCounter++;
            if (loopCounter >= LedConstants.ANIMATION_SPEED) {
                loopCounter = 0;

                // Move the pattern forward
                animationOffset++;

                // Reset offset to prevent overflow
                if (animationOffset >= LedConstants.PATTERN_LENGTH * 10) {
                    animationOffset = 0;
                }

                // Update the flowing pattern
                updateGradientFlow();
            }
        } else {
            // Robot is ENABLED (Teleop, Auto, or Test) - show alignment status
            if (alignmentStatus) {
                setGreen();  // Aligned
            } else {
                setRed();    // Not aligned
            }
        }

        // Always send data to keep LEDs lit
        led.setData(ledBuffer);
    }

    @Override
    public void simulationPeriodic() {
        // In simulation, still update the pattern
        periodic();
    }

    /**
     * Gets current alignment status
     * @return true if aligned, false otherwise
     */
    public boolean getAlignmentStatus() {
        return alignmentStatus;
    }
}