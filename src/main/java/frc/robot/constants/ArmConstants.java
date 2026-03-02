package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    public static final int MOTOR_ID = 23;
    public static final int ENCODER_PORT = 0;

    // TODO: Tune PID constants
    public static final double PROPORTIONAL_GAIN = 3.0;
    public static final double INTEGRAL_GAIN = 0.0;
    public static final double DERIVATIVE_GAIN = 0.05;

    // TODO: Find actual values using SysId
    public static final double STATIC_FRICTION_OVERCOME_VOLTAGE = 0;
    public static final double VOLTS_PER_RADIAN_PER_SECOND = 0;
    public static final double INTERTIA_OVERCOME_VOLTAGE = 0;
    public static final double GRAVITY_OVERCOME_VOLTAGE = 0;
    public static final double FEEDFORWARD_KG = 0;

    // TODO: Find actual values based on final robot
    public static final boolean MOTOR_INVERTED = false;
    public static final boolean ABSOLUTE_ENCODER_INVERTED = false;
    
    /** The gear ratio between the motor and arm shaft. */
    public static final double GEAR_RATIO =  12;

    /** The reading from the absolute encoder when the arm is horizontal */
    public static final double ABSOLUTE_ENCODER_OFFSET_DUTY_CYCLE = 0.0;

    /** The angle between the arm and the ground when extended. */
    public static final double EXTENDED_ANGLE_RADIANS = Units.degreesToRadians(56.6712636);

    /** The angle betwen the arm and ground when retracted. */
    public static final double RETRACTED_ANGLE_RADIANS = Units.degreesToRadians(79.0541668);

    /** The required accuracy of the arm, in order to finish moving it. */
    public static final double ERROR_TOLERANCE_RADIANS = Units.degreesToRadians(1); 

    // TODO: Update for more accurate simulation
    public static final double LENGTH_METERS = Units.inchesToMeters(14.722102);
    public static final double MASS_KILOGRAMS = 2.0675323998;
}
