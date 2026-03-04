package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final int LEFT_MOTOR_ID = 24;
    public static final int RIGHT_MOTOR_ID = 25;
    public static final boolean LEFT_MOTOR_INVERTED = true;

    /** The gear ratio between the motor shaft and shooter shaft. */
    public static final double GEAR_RATIO = 3.0;

    /**Acceptable tolerance between the target and current RPM on the shooter shaft, before the feeder engages. */
    public static final double RPM_TOLERANCE = 50;

    /** The voltage required per RPM of the shooter shaft. */
    public static final double VOLTAGE_PER_SHAFT_RPM = GEAR_RATIO / 473.0; 

    /** The voltage required to overcome the static friction of the shooter shaft. */
    public static final double STATIC_FRICTION_OVERCOME_VOLTAGE = 0.336;

    public static final double PROPORTIONAL_GAIN = 0.01;
    public static final double INTEGRAL_GAIN = 0;
    public static final double DERIVATIVE_GAIN = 0;

    public static final double MANUAL_SHOOT_DISTANCE_METERS = Units.inchesToMeters(75.25);
    public static final double PASS_SHOOT_DISTANCE_METERS = 3.0;

    /** The maximum velocity that the feeder can run at. Set this lower to disable shooting on the go. */
    public static final double MAXIMUM_SHOOT_SPEED_METERS_PER_SECOND = 5;
    public static final double MAXIMUM_SHOOT_ANGLE_RADIANS = Units.degreesToRadians(45);

    /** 
     * For automatically shooting while moving, this is the number of iterations used to find the future position.
     */
    public static final int MOVEMENT_CALCULATION_ITERATIONS = 3;

    /** A mapping of distances in meters to RPM setpoints.
     * The distances are measured from the center of the robot to the center of the hub.
     * It represents a piecewise function, with linear interpoolation between known points.
     */
    public static final InterpolatingDoubleTreeMap DISTANCE_METERS_TO_RPM = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DISTANCE_METERS_TO_TIME_OF_FLIGHT_SECONDS = new InterpolatingDoubleTreeMap();

    static {
        DISTANCE_METERS_TO_RPM.put(Units.inchesToMeters(63.25), 1575.0);
        DISTANCE_METERS_TO_RPM.put(Units.inchesToMeters(75.25), 1650.0);
        DISTANCE_METERS_TO_RPM.put(Units.inchesToMeters(87.25), 1775.0);

        DISTANCE_METERS_TO_TIME_OF_FLIGHT_SECONDS.put(0.0, 0.0);
    }
}
