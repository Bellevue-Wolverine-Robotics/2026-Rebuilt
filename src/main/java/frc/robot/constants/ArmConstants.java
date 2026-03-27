package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    public static final int MOTOR_ID = 22;
    public static final int ENCODER_PORT = 0;

    // TODO: Tune PID constants\
    public static final double PROPORTIONAL_GAIN = 0.45;
    public static final double INTEGRAL_GAIN = 0.0;
    public static final double DERIVATIVE_GAIN = 0.1;

    // TODO: Find actual values using SysId
    public static final double STATIC_FRICTION_OVERCOME_VOLTAGE = 0.0;
    public static final double VOLTS_PER_RADIAN_PER_SECOND = 0.0;
    public static final double INTERTIA_OVERCOME_VOLTAGE = 0.0;
    public static final double GRAVITY_OVERCOME_VOLTAGE = 0.10;

    // TODO: Find actual values based on final robot
    public static final boolean MOTOR_INVERTED = false;
    public static final boolean ABSOLUTE_ENCODER_INVERTED = true;
    
    /** The gear ratio between the motor and arm shaft. */
    public static final double GEAR_RATIO = 12;

    /** The reading from the absolute encoder when the arm is horizontal. */
    public static final double ABSOLUTE_ENCODER_OFFSET_DUTY_CYCLE = 0.058;

    /** The angle between the arm and the ground when extended. */
    public static final double EXTENDED_ANGLE_RADIANS = Units.degreesToRadians(0);

    /** The angle betwen the arm and ground when retracted. */
    public static final double RETRACTED_ANGLE_RADIANS = (0.215 - 0.058) * Math.PI * 2.0;

    /** The required accuracy of the arm, in order to finish moving it. */
    public static final double ERROR_TOLERANCE_RADIANS = Units.degreesToRadians(1.0);

    /** The distance the arm must be from the original setpoint to provide a warning on the controller. */
    public static final double WARN_THRESHOLD_RADIANS = Units.degreesToRadians(10.0);

    public static final double MANUAL_CONTROL_COFFICIENT = 0.25;
    public static final double EXTENSION_DURATION_SECONDS = 0.75;
}
