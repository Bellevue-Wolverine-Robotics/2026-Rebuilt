package frc.robot.constants;

public class IntakeConstants {
    public static final int LEFT_INTAKE_MOTOR_CAN_ID = 21;
    public static final int RIGHT_INTAKE_MOTOR_CAN_ID = 22;
    public static final int ARM_MOTOR_CAN_ID = 23;
    public static final int ARM_ENCODER_PWM_PORT = 0;
    public static final double ARM_PID_KP = 3.0; // TODO: Tune PID constants
    public static final double ARM_PID_KI = 0.0; // TODO: Tune PID constants
    public static final double ARM_PID_KD = 0.05; // TODO: Tune PID constants
    public static final double ARM_RETRACTED_SETPOINT = 0.25; // TODO: Replace with actual offset
    public static final double ARM_EXTENDED_SETPOINT = 0; // TODO: Replace with actual offset
    public static final double ARM_GEAR_RATIO = 20; // TODO: Replace with actual gear ratio
    public static final double ARM_LENGTH_METERS = 0.5; // TODO Replace with actual intake width
    public static final double ARM_MASS_KILOGRAMS = 2; // TODO: Replace with actual intake weight
    public static final double MINIMUM_ANGLE_RADIANS = Math.toRadians(-10); // TODO: replace with actual extended position
    public static final double MAXIMUM_ANGLE_RADIANS = Math.toRadians(100); // TODO: Replace with actual retracted position
    public static final double INTAKE_SPEED = 1.0; // TODO: Replace with actual intake speed
}
