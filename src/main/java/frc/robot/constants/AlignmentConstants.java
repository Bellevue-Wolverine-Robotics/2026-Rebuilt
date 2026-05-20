package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class AlignmentConstants {
    public static final double TRANSLATIONAL_PID_KP = 3.0;
    public static final double TRANSLATIONAL_PID_KI = 0.0;
    public static final double TRANSLATIONAL_PID_KD = 0.15;

    public static final double ROTATIONAL_PID_KP = 3.0;
    public static final double ROTATIONAL_PID_KI = 0.0;
    public static final double ROTATIONAL_PID_KD = 0.15;

    public static final double MAXIMUM_SPEED_RADIANS = Units.degreesToRadians(360.0);
    public static final double MAXIMUM_ACCELERATION_RADIANS_PER_SECOND = Units.degreesToRadians(720.0);

    public static final Pose2d TOLERANCE = new Pose2d(new Translation2d(Units.inchesToMeters(1.0), Units.inchesToMeters(1.0)), Rotation2d.fromDegrees(5.0));
}
