package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class AlignmentConstants {
    public static final double TRANSLATIONAL_PID_KP = 2.5;
    public static final double TRANSLATIONAL_PID_KI = 0.0;
    public static final double TRANSLATIONAL_PID_KD = 0.25;
    public static final double ROTATIONAL_PID_KP = 10.0;
    public static final double ROTATIONAL_PID_KI = 0.0;
    public static final double ROTATIONAL_PID_KD = 0.0;
    public static final double MAXIMUM_SPEED_RADIANS = Units.degreesToRadians(360);
    public static final double MAXIMUM_ACCELERATION_RADIANS = Units.degreesToRadians(180);
}
