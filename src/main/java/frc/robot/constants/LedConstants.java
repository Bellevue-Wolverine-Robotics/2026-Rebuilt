package frc.robot.constants;

public class LedConstants {
    public static final int LED_PWM_PORT = 9; // Change this to your actual PWM port
    public static final int LED_STRIP_LENGTH = 60; // Change this to your actual LED strip length

    // Animation settings (for disabled mode)
    public static final int ANIMATION_SPEED = 3; // Lower = faster, higher = slower (1-10 recommended)
    public static final int PATTERN_LENGTH = 20; // How many LEDs before pattern repeats

    // Color definitions (RGB)
    public static final int[] COLOR_BLUE = {0, 0, 255};
    public static final int[] COLOR_YELLOW = {255, 255, 0};
    public static final int[] COLOR_GREEN = {0, 255, 0};   // Aligned
    public static final int[] COLOR_RED = {255, 0, 0};     // Not aligned
}