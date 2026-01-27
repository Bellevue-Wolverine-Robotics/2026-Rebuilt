package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    public static final double MAXIMUM_SPEED_METERS = 5.0;
    public static final Pose2d STARTING_POSE = new Pose2d(3.32994, 4.03479, Rotation2d.fromDegrees(0));
    public static final Pose2d AUTONOMOUS_POSE = new Pose2d(2.32994, 3.03479, Rotation2d.fromDegrees(180));
    public static final double PATHPLANNER_TRANSLATIONAL_PID_KP = 5.0;
    public static final double PATHPLANNER_TRANSLATIONAL_PID_KI = 0.0;
    public static final double PATHPLANNER_TRANSLATIONAL_PID_KD = 0.0;
    public static final double PATHPLANNER_ROTATIONAL_PID_KP = 5.0;
    public static final double PATHPLANNER_ROTATIONAL_PID_KI = 0.0;
    public static final double PATHPLANNER_ROTATIONAL_PID_KD = 0.0;
    public static final double PATHPLANNER_MAXIMUM_SPEED_METERS = 1.0;
    public static final double PATHPLANNER_MAXIMUM_ACCELERATION_METERS = 1.0;
    public static final double PATHPLANNER_MAXIMUM_SPEED_RADIANS = Units.degreesToRadians(540);
    public static final double PATHPLANNER_MAXIMUM_ACCELERATION_RADIANS = Units.degreesToRadians(720);
}
