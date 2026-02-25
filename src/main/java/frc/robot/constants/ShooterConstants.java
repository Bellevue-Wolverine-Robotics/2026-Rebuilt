package frc.robot.constants;

import java.util.NavigableMap;
import java.util.TreeMap;

public class ShooterConstants {
    public static final int LEFT_MOTOR_ID = 24;
    public static final int RIGHT_MOTOR_ID = 25;

    /** The gear ratio between the motor shaft and shooter shaft. */
    public static final double GEAR_RATIO = 3.0;

    /**Acceptable tolerance between the target and current RPM on the shooter shaft, before the feeder engages. */
    public static final double RPM_TOLERANCE = 25;

    /** The voltage required per RPM of the shooter shaft. */
    public static final double VOLTAGE_PER_MOTOR_RPM = GEAR_RATIO / 473.0; 

    /** The voltage required to overcome the static friction of the shooter shaft. */
    public static final double STATIC_FRICTION_VOLTAGE = 0.336;

    public static final double PROPORTIONAL_GAIN = 0.1;
    public static final double INTEGRAL_GAIN = 0;
    public static final double DERIVATIVE_GAIN = 0;

    public static final double MANUAL_SHOOT_METERS = 10.0; // TODO: Come up with an actual value
    public static final double PASS_SHOOT_METERS = 10.0; // TODO: Come up with an actual value

    /** A mapping of distances in meters to RPM setpoints.
     * The distances are measured from the center of the robot to the center of the hub.
     * It represents a piecewise function, with linear interpoolation between known points.
     */
    public static final NavigableMap<Double, Double> SETPOINTS_METERS_TO_RPM = new TreeMap<>();


    static {
        SETPOINTS_METERS_TO_RPM.put(1.00965, 1425.0);
        SETPOINTS_METERS_TO_RPM.put(1.31445, 1450.0);
        SETPOINTS_METERS_TO_RPM.put(1.61925, 1550.0);
        SETPOINTS_METERS_TO_RPM.put(1.92405, 1650.0);
        SETPOINTS_METERS_TO_RPM.put(2.22885, 1750.0);
        SETPOINTS_METERS_TO_RPM.put(2.53365, 1825.0);
    }

}
