package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveConstants {
    public static final double MAXIMUM_SPEED_METERS = 5;
    public static final Pose2d STARTING_POSE = new Pose2d(3.32994, 4.03479, Rotation2d.fromDegrees(0));
    public static final Pose2d AUTONOMOUS_POSE = new Pose2d(2.32994, 3.03479, Rotation2d.fromDegrees(180));
}
