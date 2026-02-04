package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class AlignmentConstants {
    public static final double TRANSLATIONAL_PID_KP = 2.5;
    public static final double TRANSLATIONAL_PID_KI = 0.0;
    public static final double TRANSLATIONAL_PID_KD = 0.25;
    public static final double ROTATIONAL_PID_KP = 5.0;
    public static final double ROTATIONAL_PID_KI = 0.0;
    public static final double ROTATIONAL_PID_KD = 0.0;
    public static final double MAXIMUM_SPEED_RADIANS = Units.degreesToRadians(360);
    public static final double MAXIMUM_ACCELERATION_RADIANS = Units.degreesToRadians(720);
    public static final Pose2d TOLERANCE = new Pose2d(new Translation2d(0.0254, 0.0254), Rotation2d.fromDegrees(5));
    public static final double ROTATIONAL_TOLERANCE_DISTANCE = Units.degreesToRadians(5);
    public static final double ROTATIONAL_TOLERANCE_VELOCITY = Units.degreesToRadians(2.5);
}
