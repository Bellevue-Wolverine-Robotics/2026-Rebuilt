package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.AlignmentConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoShootCommand extends Command {
    private final PIDController thetaController = new PIDController(
        ShooterConstants.ROTATIONAL_PID_KP,
        ShooterConstants.ROTATIONAL_PID_KI,
        ShooterConstants.ROTATIONAL_PID_KD
    );

    private final SwerveSubsystem swerveSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final DoubleSupplier xAxis;
    private final DoubleSupplier yAxis;

    public AutoShootCommand(
        SwerveSubsystem swerveSubsystem,
        LEDSubsystem ledSubsystem,
        ShooterSubsystem shooterSubsystem,
        FeederSubsystem feederSubsystem,
        DoubleSupplier xAxis,
        DoubleSupplier yAxis
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        addRequirements(swerveSubsystem, shooterSubsystem, feederSubsystem);

        thetaController.setTolerance(AlignmentConstants.ROTATIONAL_TOLERANCE_RADIANS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Finds the approximate position of the robot when launched fuel will arrive at the hub.
     * 
     * @param current The current position of the robot.
     * @return The future position of the robot.
     */
    private Pose2d calculatePose(Pose2d current) {
        // By calculating the time of flight and current velocity, we are able
        // to estiamte where the robot will be when the launched fuel arrives at the hub.
        Pose2d future = current;
        ChassisSpeeds velocity = swerveSubsystem.getVelocity();
    
        // We need to know the time of flight from the position of the robot when the fuel lands
        // But to know the position of the robot, we also need to know the time of flight
        // To do this, we can repeat the process by finding the time of flight
        // at the new calculated future pose as many times as necessary to get an accurate estimate.
        for (int i = 0; i < ShooterConstants.MOVEMENT_CALCULATION_ITERATIONS; i++) {
            double distance = future.getTranslation().getDistance(VisionConstants.HUB_POSE_SUPPLIER.get().getTranslation());

            double time = ShooterConstants.DISTANCE_METERS_TO_TIME_OF_FLIGHT_SECONDS.get(distance);

            future = new Pose2d(
                current.getX() + (velocity.vxMetersPerSecond * time),
                current.getY() + (velocity.vyMetersPerSecond * time),
                current.getRotation()
            );
        }

        return future;
    }

    @Override
    public void initialize() {
        thetaController.reset();
    }

    @Override
    public void execute() {
        Pose2d current = swerveSubsystem.getPose();
        Pose2d future = calculatePose(current);

        Translation2d difference = VisionConstants.HUB_POSE_SUPPLIER.get().getTranslation().minus(future.getTranslation());

        double currentHeading = current.getRotation().getRadians();
        double desiredHeading = difference.getAngle().getRadians();

        if (thetaController.atSetpoint() && xAxis.getAsDouble() == 0 && yAxis.getAsDouble() == 0) {
            swerveSubsystem.lock();
        } else {
            swerveSubsystem.drive(
                xAxis.getAsDouble(),
                yAxis.getAsDouble(),
                thetaController.calculate(currentHeading, desiredHeading)
            );
        }

        double distance = difference.getNorm();
        shooterSubsystem.run(distance);

        if (thetaController.atSetpoint() && shooterSubsystem.atSpeed() && swerveSubsystem.getTranslationalVelocity() <= ShooterConstants.MAXIMUM_SHOOT_SPEED_METERS_PER_SECOND) {
            feederSubsystem.run();
        } else {
            feederSubsystem.stop();
        }

        ledSubsystem.setAligning(!thetaController.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.stop();
        shooterSubsystem.stop();
        ledSubsystem.setAligning(false);
    }
}
