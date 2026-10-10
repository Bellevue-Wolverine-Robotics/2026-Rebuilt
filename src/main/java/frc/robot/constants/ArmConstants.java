package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    public static final int MOTOR_ID = 22;
    public static final int ENCODER_PORT = 0;

    public static final double PROPORTIONAL_GAIN = 0.25;
    public static final double INTEGRAL_GAIN = 0.0;
    public static final double DERIVATIVE_GAIN = 0.0;

    // For simplicity, we only use the gravity constant, since the other three are less important in an arm
    public static final double STATIC_FRICTION_OVERCOME_VOLTAGE = 0.0;
    public static final double VOLTS_PER_RADIAN_PER_SECOND = 0.0;
    public static final double INTERTIA_OVERCOME_VOLTAGE = 0.0;

    // Calculated with holding test and sanity checked with theoretical torque exerted on motor
    public static final double GRAVITY_OVERCOME_VOLTAGE = 0.288;

    public static final double CRUISE_VELOCITY_RADIANS_PER_SECOND = Math.PI * 3.0;
    public static final double MAXIMUM_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI * 3.0;
    public static final double ALLOWED_PROFILE_ERROR_RADIANS = Units.degreesToRadians(10.0);

    public static final boolean MOTOR_INVERTED = false;
    public static final boolean ABSOLUTE_ENCODER_INVERTED = true;

    /** The gear ratio between the motor and arm shaft. */
    public static final double GEAR_RATIO = 36;

    /** The reading from the absolute encoder when the arm is horizontal. */
    // Encoder reads 0.835 at extended state, and the angle of the side plate is 13 degrees relative to horizontal, while the center of mass line is 6.30873581464 degrees relative to horizontal.
    public static final double ABSOLUTE_ENCODER_OFFSET_DUTY_CYCLE = 0.81085759948511111111;
    
    /** The angle between the arm and the ground when extended. */
    public static final double EXTENDED_ANGLE_RADIANS = 0.586;

    /** The angle betwen the arm and ground when retracted. */
    public static final double RETRACTED_ANGLE_RADIANS = 1.937;

    /** The required accuracy of the arm, in order to finish moving it. */
    public static final double ERROR_TOLERANCE_RADIANS = Units.degreesToRadians(3.6);

    // TODO: Find more optimal manual control speed
    public static final double MANUAL_CONTROL_COFFICIENT = 0.5;
    public static final double EXTENSION_DURATION_SECONDS = 0.75;
}
