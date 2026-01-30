package frc.robot.constants;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class VisionConstants {
    public static final String CAMERA_NAME = "main";
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 0.17);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.25, 0.25, 0.085);
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(new Translation3d(0.381, -0.114, 0.3048), new Rotation3d(0, 0, 0));

    private static final Pose3d ORIGIN_POSE = TAG_LAYOUT.getOrigin();
    private static final Transform2d ROBOT_CENTER_TO_FRONT = new Transform2d(new Translation2d(0.6985, 0), new Rotation2d(0));

    private static final Pose2d getAllianceSpecificTagPose(int blueTagId, int redTagId, Transform2d transform) {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue; 
        Pose2d tagPose = TAG_LAYOUT.getTagPose(isBlue ? blueTagId: redTagId).orElse(ORIGIN_POSE).toPose2d();
        return tagPose.transformBy(transform);
    }

    public static final Supplier<Pose2d> CLIMB_POSE_SUPPLIER = () -> {
        Transform2d transform = new Transform2d(new Translation2d(2.516, 0.0), Rotation2d.fromDegrees(180)).plus(ROBOT_CENTER_TO_FRONT);
        return getAllianceSpecificTagPose(31, 15, transform);
    };

    public static final Supplier<Pose2d> SHOOT_POSE_SUPPLIER = () -> {
        Transform2d transform = new Transform2d(new Translation2d(-2.58801, 0.0), new Rotation2d()).plus(ROBOT_CENTER_TO_FRONT);
        return getAllianceSpecificTagPose(20, 4, transform);
    };

    public static final Supplier<Pose2d> HUB_POSE_SUPPLIER = () -> {
        Transform2d transform = new Transform2d(new Translation2d(-0.58801, 0.0), new Rotation2d());
        return getAllianceSpecificTagPose(20, 4, transform);
    };
}
