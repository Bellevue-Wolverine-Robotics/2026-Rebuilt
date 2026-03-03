package frc.robot.subsystems;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import frc.robot.commands.AlignPoseCommand;
import frc.robot.constants.PathPlannerConstants;
import frc.robot.constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final LEDSubsystem ledSubsystem;
    private final SwerveDrive swerveDrive;

    public SwerveSubsystem(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;

        File swerveJsonDirectory = new File(
            Filesystem.getDeployDirectory(),
            Preferences.getString("swerveDirectory", "swervePractice")
        );

        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(SwerveConstants.MAXIMUM_SPEED_METERS);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

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
                        PathPlannerConstants.TRANSLATIONAL_PID_KP,
                        PathPlannerConstants.TRANSLATIONAL_PID_KI,
                        PathPlannerConstants.TRANSLATIONAL_PID_KD
                    ),
                    new PIDConstants(
                        PathPlannerConstants.ROTATIONAL_PID_KP,
                        PathPlannerConstants.ROTATIONAL_PID_KI,
                        PathPlannerConstants.ROTATIONAL_PID_KD
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
     * 
     * @param chassisSpeeds The translative and rotational velocity to the drive at.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    /**
     * Drives the robot using field relative translative and angular velocities.
     * 
     * @param translation The translational velocity of the robot in meters per second.
     * @param rotation    The rotational velocity in radians per second.
     */
    public void drive(Translation2d translation, double angularRotation) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            translation = translation.rotateBy(Rotation2d.fromDegrees(180));
        }

        swerveDrive.drive(translation, angularRotation, true, false);
    }

    /**
     * Adds vision pose estiamte to the odometry.
     * 
     * @param estimatedPose The pose estimated by the coprocessor.
     * @param timestampSecond The time at which the processed frame was captured.
     * @param stdDevs The estimated standard deviation for the pose estimate, in meters and radians.
     */
    public void addVisionMeasurement(Pose2d estimatedPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        swerveDrive.addVisionMeasurement(estimatedPose, timestampSeconds, stdDevs);
    }

    /**
     * Provides the simulated drivetrain pose to update simulated cameras.
     *
     * @return Simulated pose.
     */
    public Pose2d getSimulationPose() {
        return swerveDrive.getSimulationDriveTrainPose().get();
    }

    /**
     * Provides the pose of the drivetrain on the field from the odometry.
     * 
     * @return Odometry pose.
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Converts normalized inputs [-1, 1] into a translational velocity for field relative driving.
     * 
     * @param xAxis X axis normalized input.
     * @param yAxis Y axis normalized input.
     */
    public Translation2d inputToTranslation(double xAxis, double yAxis) {
        double magnitude = Math.pow(Math.hypot(xAxis, yAxis), SwerveConstants.SMOOTHING_EXPONENT);
        double angle = Math.atan2(yAxis, xAxis);

        return new Translation2d(
            Math.cos(angle) * magnitude * swerveDrive.getMaximumChassisVelocity(),
            Math.sin(angle) * magnitude * swerveDrive.getMaximumChassisVelocity()
        );
    }

    /**
     * Provides a command to drive the robot using field relative translative values and heading as angular velocity.
     *
     * @param xAxis     The input axis that corresponds to movement along to the X axis.
     * @param yAxis     The input axis that corresponds to movement along the Y axis.
     * @param rotationAxis The input axis that corresponds to rotational movement.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotationAxis) {
        return run(() -> {
            ledSubsystem.setAligned(false);

            drive(
                inputToTranslation(xAxis.getAsDouble(), yAxis.getAsDouble()),
                Math.pow(rotationAxis.getAsDouble(), SwerveConstants.SMOOTHING_EXPONENT) * swerveDrive.getMaximumChassisAngularVelocity()
            );
        });
    }

    /** Provides the field relative robot velocity.
     * 
     * @return The field relative velocity.
     */
    public ChassisSpeeds getVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /** Provides the magnitude, or overall linear speed, of field relative robot velocity.
     * 
     * @return The field relative translational velocity in meters per second.
     */
    public double getTranslationalVelocity() {
        ChassisSpeeds velocities = swerveDrive.getFieldVelocity();
        return Math.sqrt(velocities.vxMetersPerSecond * velocities.vxMetersPerSecond +
                         velocities.vyMetersPerSecond * velocities.vyMetersPerSecond);
    }

    /** Provides the rotational velocity, in radians per second.
     * 
     * @return The rotational velocity, in radians in persecond.
     */
    public double getRotationalVelocity() {
        return swerveDrive.getRobotVelocity().omegaRadiansPerSecond;
    }

    /**
     * Provides a command that drives the robot to a specific pose on the field.
     *
     * @param pose The pose to go to.
     * @return Command that aligns with the pose.
     */
    public Command alignPoseCommand(Supplier<Pose2d> pose) {
        return new AlignPoseCommand(this, ledSubsystem, pose);
    }

    /**
     * Provies a command that resets the gyro.
     * This causes the current heading to be considered the forward direction for field relative driving purposes.
     * 
     * @return Command that zeros the gyro.
     */
    public Command zeroGyro() {
        return runOnce(swerveDrive::zeroGyro);
    }
}
