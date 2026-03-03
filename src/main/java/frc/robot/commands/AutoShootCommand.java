package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.AlignmentConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoShootCommand extends Command {
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
        AlignmentConstants.ROTATIONAL_PID_KP,
        AlignmentConstants.ROTATIONAL_PID_KI,
        AlignmentConstants.ROTATIONAL_PID_KD,
        new TrapezoidProfile.Constraints(
            AlignmentConstants.MAXIMUM_SPEED_RADIANS,
            AlignmentConstants.MAXIMUM_ACCELERATION_RADIANS
        )
    );

    private final SwerveSubsystem swerveSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final DoubleSupplier xAxis;
    private final DoubleSupplier yAxis;
    private final Supplier<Pose2d> target;

    public AutoShootCommand(
        SwerveSubsystem swerveSubsystem,
        LEDSubsystem ledSubsystem,
        ShooterSubsystem shooterSubsystem,
        FeederSubsystem feederSubsystem,
        DoubleSupplier xAxis,
        DoubleSupplier yAxis,
        Supplier<Pose2d> target
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.target = target;
        addRequirements(swerveSubsystem, shooterSubsystem, feederSubsystem);

        thetaController.setTolerance(AlignmentConstants.ROTATIONAL_TOLERANCE_DISTANCE, AlignmentConstants.ROTATIONAL_TOLERANCE_VELOCITY);
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
        // We can repeat the process by finding the time of flight
        // at the new calculated future pose as many times as necessary to get an accurate estimate.
        Pose2d future = current;
        ChassisSpeeds velocity = swerveSubsystem.getVelocity();
    
        for (int i = 0; i <= ShooterConstants.MOVEMENT_CALCULATION_ITERATIONS; i++) {
            double distance = Math.hypot(
                target.get().getX() - future.getX(),
                target.get().getY() - future.getY()
            );

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
        thetaController.reset(swerveSubsystem.getPose().getRotation().getRadians(), swerveSubsystem.getRotationalVelocity());
    }

    @Override
    public void execute() {
        Pose2d current = swerveSubsystem.getPose();
        Pose2d future = calculatePose(current);

        double x = target.get().getX() - future.getX();
        double y = target.get().getY() - future.getY();

        double currentHeading = current.getRotation().getRadians();
        double desiredHeading = Math.atan2(y, x);

        ledSubsystem.setAligned(thetaController.atGoal());

        swerveSubsystem.drive(
            swerveSubsystem.inputToTranslation(xAxis.getAsDouble(), yAxis.getAsDouble()),
            thetaController.calculate(currentHeading, desiredHeading)
        );

        double distance = Math.hypot(x, y);
        shooterSubsystem.run(distance);

        if (thetaController.atGoal() && shooterSubsystem.atSpeed()) {
            feederSubsystem.run();
        } else {
            feederSubsystem.stop();
        }

    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.stop();
        shooterSubsystem.stop();
        ledSubsystem.setAligned(false);
    }
}
