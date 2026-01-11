package frc.robot.subsystems;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;

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
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)
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

    @Override
    public void periodic() {
        // Display current encoder positions for debugging
        SwerveModule[] modules = swerveDrive.getModules();
        for (int i = 0; i < modules.length; i++) {
            SmartDashboard.putNumber("Module " + i + "/Absolute Encoder",
                    modules[i].getAbsolutePosition());
            SmartDashboard.putNumber("Module " + i + "/Angle",
                    modules[i].getState().angle.getDegrees());
        }
    }

    /**
     * Print current encoder values for calibration
     * Manually align all wheels forward first, then call this
     */
    public void printEncoderOffsets() {
        System.out.println("========================================");
        System.out.println("CURRENT ENCODER READINGS (for calibration):");
        System.out.println("Copy these values to your module JSON files");
        System.out.println("========================================");

        SwerveModule[] modules = swerveDrive.getModules();
        String[] moduleNames = {"frontleft", "frontright", "backleft", "backright"};

        for (int i = 0; i < modules.length; i++) {
            double absolutePosition = modules[i].getAbsolutePosition();
            System.out.println(moduleNames[i] + ".json -> \"absoluteEncoderOffset\": " +
                    String.format("%.5f", absolutePosition));
        }

        System.out.println("========================================");
    }

    /**
     * Lock all wheels pointing forward (0 degrees)
     * Use this to help manually align wheels for calibration
     */
    public void lockWheelsForward() {
        swerveDrive.drive(new ChassisSpeeds(0, 0, 0));
        swerveDrive.lockPose();
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
}