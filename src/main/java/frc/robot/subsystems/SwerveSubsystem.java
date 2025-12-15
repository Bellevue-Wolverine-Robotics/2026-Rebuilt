package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;

import frc.robot.constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    SwerveDrive swerveDrive;

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        Pose2d startingPose = new Pose2d(new Translation2d(8, 4), Rotation2d.fromDegrees(0));

        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(SwerveConstants.MAXIMUM_SPEED_METERS, startingPose);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize swerve");
        }
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
        return run(() -> {
        swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                            angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                            true,
                            false);
        });
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
    }
}
