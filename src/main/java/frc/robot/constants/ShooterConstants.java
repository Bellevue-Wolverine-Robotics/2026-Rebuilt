package frc.robot.constants;

public class ShooterConstants {
    public static final int LEFT_MOTOR_ID = 24;
    public static final int RIGHT_MOTOR_ID = 25;

    /** The gear ratio between the motor shaft and shooter shaft. */
    public static final double GEAR_RATIO = 3.0;

    /** Acceptable tolerance between the target and current RPM in order to shoot. */
    public static final double RPM_TOLERANCE = 50;

    /* The voltage required per RPM of the motor shaft. */
    public static final double VOLTAGE_PER_MOTOR_RPM = GEAR_RATIO / 473.0; 

    /* The voltage required to overcome the static friction of the shooter shaft. */
    public static final double STATIC_FRICTION_VOLTAGE = 0.336;

    public static final double PROPORTIONAL_GAIN = 0.001;
    public static final double INTEGRAL_GAIN = 0;
    public static final double DERIVATIVE_GAIN = 0;

    public static final double SHORT_SHOOT_METERS = 10.0; // TODO: Come up with an actual value
    public static final double PASS_SHOOT_METERS = 10.0; // TODO: Come up with an actual value

    public static final double DIST_KX2 = 0;
    public static final double DIST_KX1 = 0;
    public static final double DIST_KX0 = 0;
}
