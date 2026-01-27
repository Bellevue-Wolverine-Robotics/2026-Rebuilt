package frc.robot.subsystems;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;

import frc.robot.commands.AlignPoseCommand;
import frc.robot.constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive swerveDrive;

    public SwerveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swervePractice");

        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(SwerveConstants.MAXIMUM_SPEED_METERS, SwerveConstants.STARTING_POSE);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);

        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                swerveDrive::getPose,
                swerveDrive::resetOdometry,
                swerveDrive::getRobotVelocity,
                (speeds, feedforwards) -> {
                    swerveDrive.drive(
                        speeds,
                        swerveDrive.kinematics.toSwerveModuleStates(speeds),
                        feedforwards.linearForces()
                    );
                },
                new PPHolonomicDriveController(
                    new PIDConstants(
                        SwerveConstants.PATHPLANNER_TRANSLATIONAL_PID_KP,
                        SwerveConstants.PATHPLANNER_TRANSLATIONAL_PID_KI,
                        SwerveConstants.PATHPLANNER_TRANSLATIONAL_PID_KD
                    ),
                    new PIDConstants(
                        SwerveConstants.PATHPLANNER_ROTATIONAL_PID_KP,
                        SwerveConstants.PATHPLANNER_ROTATIONAL_PID_KI,
                        SwerveConstants.PATHPLANNER_ROTATIONAL_PID_KD
                    )
                ),
                config,
                () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Drives the robot with a specified robot relative velocity.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Rotation of the robot to set
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> {
            swerveDrive.drive(
                new Translation2d(
                    translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
                ),
                angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                true,
                false
            );
        });
    }

    public Command zeroGyro() {
        return runOnce(() -> {
            swerveDrive.zeroGyro();
        });
    }

    /**
     * Uses PathPlanner to drive to the desired position on the field, while avoiding obstacles.
     * This is not accurate enough for full automatic alignment.
     */
    public Command driveToPoseCommand(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
            SwerveConstants.PATHPLANNER_MAXIMUM_SPEED_METERS,
            SwerveConstants.PATHPLANNER_MAXIMUM_ACCELERATION_METERS,
            SwerveConstants.PATHPLANNER_MAXIMUM_SPEED_RADIANS,
            SwerveConstants.PATHPLANNER_MAXIMUM_ACCELERATION_RADIANS
        );
        return AutoBuilder.pathfindToPose(pose, constraints, 0.0);
    }


    /*
     * Uses a PID controller to percisely hold the robot in a desired posiiton.
     */
    public Command alignToPoseCommand(Pose2d pose) {
        return new AlignPoseCommand(this, pose);
    }

    public void addVisionMeasurement(Pose2d estimatedPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        swerveDrive.addVisionMeasurement(estimatedPose, timestampSeconds, stdDevs);
    }

    /* Gets the simulated drivetrain position */
    public Pose2d getSimulationPose() {
        return swerveDrive.getSimulationDriveTrainPose().get();
    }

    /* Gets the position of the robot on the field */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }
}
