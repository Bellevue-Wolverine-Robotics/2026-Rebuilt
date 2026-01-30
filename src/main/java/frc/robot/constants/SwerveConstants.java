package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    public static final double MAXIMUM_SPEED_METERS = 5.0;
    public static final Pose2d STARTING_POSE = new Pose2d(new Translation2d(5, 5),  new Rotation2d());

    public static final double PATHPLANNER_TRANSLATIONAL_PID_KP = 5.0;
    public static final double PATHPLANNER_TRANSLATIONAL_PID_KI = 0.0;
    public static final double PATHPLANNER_TRANSLATIONAL_PID_KD = 0.0;
    public static final double PATHPLANNER_ROTATIONAL_PID_KP = 5.0;
    public static final double PATHPLANNER_ROTATIONAL_PID_KI = 0.0;
    public static final double PATHPLANNER_ROTATIONAL_PID_KD = 0.0;
    public static final double PATHPLANNER_MAXIMUM_SPEED_METERS = 5.0;
    public static final double PATHPLANNER_MAXIMUM_ACCELERATION_METERS = 2.5;
    public static final double PATHPLANNER_MAXIMUM_SPEED_RADIANS = Units.degreesToRadians(540);
    public static final double PATHPLANNER_MAXIMUM_ACCELERATION_RADIANS = Units.degreesToRadians(720);

    public static final double AUTOALIGN_TRANSLATIONAL_PID_KP = 5.0;
    public static final double AUTOALIGN_TRANSLATIONAL_PID_KI = 0.0;
    public static final double AUTOALIGN_TRANSLATIONAL_PID_KD = 0.0;
    public static final double AUTOALIGN_ROTATIONAL_PID_KP = 1.75;
    public static final double AUTOALIGN_ROTATIONAL_PID_KI = 0.0;
    public static final double AUTOALIGN_ROTATIONAL_PID_KD = 0.0;
    public static final double AUTOALIGN_MAXIMUM_SPEED_RADIANS = Units.degreesToRadians(180);
    public static final double AUTOALIGN_MAXIMUM_ACCELERATION_RADIANS = Units.degreesToRadians(45);
}
