package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.LedConstants;

public class LedLightSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    // Animation variables
    private int animationOffset = 0;
    private int loopCounter = 0;
    private AnimationMode currentMode = AnimationMode.GRADIENT_FLOW;

    // Animation modes
    public enum AnimationMode {
        GRADIENT_FLOW,    // Smooth gradient flowing
        BLOCK_FLOW,       // Blocks of colors flowing
        WAVE_FLOW,        // Wave pattern
        SOLID_BLUE,       // Static blue
        SOLID_YELLOW,     // Static yellow
        OFF               // All off
    }

    public LedLightSubsystem() {
        // Initialize the LED strip
        led = new AddressableLED(LedConstants.LED_PWM_PORT);

        // Create the buffer for the LED data
        ledBuffer = new AddressableLEDBuffer(LedConstants.LED_STRIP_LENGTH);

        // Set the length of the LED strip
        led.setLength(ledBuffer.getLength());

        // Start the LED output
        led.start();

        System.out.println("LED Subsystem initialized with flowing blue/yellow pattern");
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
     * Creates alternating blue and yellow blocks that flow
     */
    private void updateBlockFlow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate which block we're in
            int blockIndex = ((i + animationOffset) / LedConstants.BLOCK_SIZE) % 2;

            if (blockIndex == 0) {
                // Blue block
                ledBuffer.setRGB(i,
                        LedConstants.COLOR_BLUE[0],
                        LedConstants.COLOR_BLUE[1],
                        LedConstants.COLOR_BLUE[2]);
            } else {
                // Yellow block
                ledBuffer.setRGB(i,
                        LedConstants.COLOR_YELLOW[0],
                        LedConstants.COLOR_YELLOW[1],
                        LedConstants.COLOR_YELLOW[2]);
            }
        }
    }

    /**
     * Creates a wave pattern with blue and yellow
     */
    private void updateWaveFlow() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            // Create a sine wave pattern
            double angle = ((i + animationOffset) * 2 * Math.PI) / LedConstants.PATTERN_LENGTH;
            double wave = (Math.sin(angle) + 1.0) / 2.0; // Normalize to 0.0-1.0

            // Interpolate between blue and yellow based on wave
            int red = (int) (LedConstants.COLOR_YELLOW[0] * wave);
            int green = (int) (LedConstants.COLOR_YELLOW[1] * wave);
            int blue = (int) (LedConstants.COLOR_BLUE[2] * (1 - wave) + LedConstants.COLOR_YELLOW[2] * wave);

            ledBuffer.setRGB(i, red, green, blue);
        }
    }

    /**
     * Updates the LED pattern based on current mode
     */
    private void updatePattern() {
        switch (currentMode) {
            case GRADIENT_FLOW:
                updateGradientFlow();
                break;
            case BLOCK_FLOW:
                updateBlockFlow();
                break;
            case WAVE_FLOW:
                updateWaveFlow();
                break;
            case SOLID_BLUE:
                setSolidColor(LedConstants.COLOR_BLUE);
                break;
            case SOLID_YELLOW:
                setSolidColor(LedConstants.COLOR_YELLOW);
                break;
            case OFF:
                setSolidColor(new int[]{0, 0, 0});
                break;
        }
    }

    /**
     * Sets all LEDs to a solid color
     */
    private void setSolidColor(int[] color) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, color[0], color[1], color[2]);
        }
    }

    @Override
    public void periodic() {
        // Update animation based on speed setting
        loopCounter++;
        if (loopCounter >= LedConstants.ANIMATION_SPEED) {
            loopCounter = 0;

            // Only update offset for flowing modes
            if (currentMode == AnimationMode.GRADIENT_FLOW ||
                    currentMode == AnimationMode.BLOCK_FLOW ||
                    currentMode == AnimationMode.WAVE_FLOW) {
                animationOffset++;

                // Reset offset to prevent overflow
                if (animationOffset >= LedConstants.PATTERN_LENGTH * 10) {
                    animationOffset = 0;
                }
            }

            // Update the LED pattern
            updatePattern();
        }

        // Always send data to keep LEDs lit
        led.setData(ledBuffer);
    }

    @Override
    public void simulationPeriodic() {
        // In simulation, still update the animation
        periodic();
    }

    // Public methods to control the LEDs

    /**
     * Sets the animation mode
     */
    public void setMode(AnimationMode mode) {
        currentMode = mode;
        animationOffset = 0; // Reset animation
        updatePattern();
        led.setData(ledBuffer);
    }

    /**
     * Sets flowing gradient pattern (default)
     */
    public void setFlowingGradient() {
        setMode(AnimationMode.GRADIENT_FLOW);
    }

    /**
     * Sets flowing block pattern
     */
    public void setFlowingBlocks() {
        setMode(AnimationMode.BLOCK_FLOW);
    }

    /**
     * Sets flowing wave pattern
     */
    public void setFlowingWave() {
        setMode(AnimationMode.WAVE_FLOW);
    }

    /**
     * Sets all LEDs to solid blue
     */
    public void setBlue() {
        setMode(AnimationMode.SOLID_BLUE);
    }

    /**
     * Sets all LEDs to solid yellow
     */
    public void setYellow() {
        setMode(AnimationMode.SOLID_YELLOW);
    }

    /**
     * Turns off all LEDs
     */
    public void turnOff() {
        setMode(AnimationMode.OFF);
    }

    /**
     * Gets current animation mode
     */
    public AnimationMode getCurrentMode() {
        return currentMode;
    }
}