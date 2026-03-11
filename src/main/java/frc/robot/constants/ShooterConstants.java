package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

// TODO: Tune PID, feedforward, and lookup table
public class ShooterConstants {
    public static final int LEFT_MOTOR_ID = 24;
    public static final int RIGHT_MOTOR_ID = 25;
    public static final boolean LEFT_MOTOR_INVERTED = true;

    /** The gear ratio between the motor shaft and shooter shaft. */
    public static final double GEAR_RATIO = 2.0;

    /** Acceptable tolerance between the target and current RPS on the shooter shaft for the feeder to engage. */
    public static final double RPS_TOLERANCE = 0.5;

    /** The voltage required per RPS of the shooter shaft. */
    // The Kraken X60 motor generates about 504 RPM per volt, so the reciprocal is the volts per RPM
    // We then multiply this value by 60, the number of seconds in a minute, to find the volts per RPS
    public static final double VOLTS_PER_RPS = (GEAR_RATIO * 60) / 504.0;

    /** The voltage required to overcome the static friction of the shooter shaft. */
    public static final double STATIC_FRICTION_OVERCOME_VOLTAGE = 0.336;

    public static final double PROPORTIONAL_GAIN = 0.01;
    public static final double INTEGRAL_GAIN = 0.0;
    public static final double DERIVATIVE_GAIN = 0.0;

    public static final double MANUAL_SHOOT_DISTANCE_METERS = Units.inchesToMeters(75.25);
    public static final double PASS_SHOOT_DISTANCE_METERS = 3.0;

    /** The maximum velocity that the feeder can run at. Set this lower to disable shooting on the go. */
    public static final double MAXIMUM_SHOOT_SPEED_METERS_PER_SECOND = 5.0;

    public static final double MAXIMUM_ANGLE_MAGNITUDE_RADIANS = Units.degreesToRadians(45.0);
    public static final double MINIMUM_DISTANCE_METERS = Units.inchesToMeters(63.25);
    public static final double MAXIMUM_DISTANCE_METERS = Units.inchesToMeters(87.25);

    public static final double ROTATIONAL_PID_KP = 8.0;
    public static final double ROTATIONAL_PID_KI = 0.0;
    public static final double ROTATIONAL_PID_KD = 0.0;
    public static final double ROTATIONAL_TOLERANCE_RADIANS = Units.degreesToRadians(5.0);

    /** 
     * For automatically shooting while moving, this is the number of iterations used to find the future position.
     */
    public static final int MOVEMENT_CALCULATION_ITERATIONS = 3;

    /**
     * A mapping of distances in meters to RPM setpoints.
     * The distances are measured from the center of the robot to the center of the hub.
     * It represents a piecewise function, with linear interpoolation between known points.
     */
    public static final InterpolatingDoubleTreeMap DISTANCE_METERS_TO_RPS = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DISTANCE_METERS_TO_TIME_OF_FLIGHT_SECONDS = new InterpolatingDoubleTreeMap();

    static {
        DISTANCE_METERS_TO_RPS.put(Units.inchesToMeters(63.25), 26.25);
        DISTANCE_METERS_TO_RPS.put(Units.inchesToMeters(75.25), 27.5);
        DISTANCE_METERS_TO_RPS.put(Units.inchesToMeters(87.25), 29.583);

        // These are temporary values so that the time of flight is always zero
        // We'll replace these with actual values during final testing
        DISTANCE_METERS_TO_TIME_OF_FLIGHT_SECONDS.put(63.25, 0.0);
        DISTANCE_METERS_TO_TIME_OF_FLIGHT_SECONDS.put(87.25, 0.0);
    }
}
