package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    public static final int MOTOR_CAN_ID = 23;
    public static final int ENCODER_PWM_PORT = 0;

    public static final double PID_KP = 3.0; // TODO: Tune PID constants
    public static final double PID_KI = 0.0; // TODO: Tune PID constants
    public static final double PID_KD = 0.05; // TODO: Tune PID constants

    public static final double FEEDFORWARD_KS = -1; // TODO: Run SysId to find constants
    public static final double FEEDFORWARD_KV = -1; // TOOD: Run SysId to find constants
    public static final double FEEDFORWARD_KA = -1; // TODO: Run SysId to find constants
    public static final double FEEDFORWARD_KG = -1; // TODO: Run SysId to find constants

    public static final double RETRACTED_SETPOINT = 0.25; // TODO: Replace with actual offset
    public static final double EXTENDED_SETPOINT = 0; // TODO: Replace with actual offset
    public static final double SETPOINT_ERROR_TOLERANCE = 0.025; // TODO: Replace with actual tolerance 
    public static final double VELOCITY_ERROR_TOLERANCE = 0.025; // TODO: Replace with actual velocity error tolerance

    public static final double GEAR_RATIO = 12;
    public static final double LENGTH_METERS = Units.inchesToMeters(14.722102);
    public static final double MASS_KILOGRAMS = 2.0675323998;
    public static final double MINIMUM_ANGLE_RADIANS = Units.degreesToRadians(56.6712636);
    public static final double MAXIMUM_ANGLE_RADIANS = Units.degreesToRadians(79.0541668);
}
