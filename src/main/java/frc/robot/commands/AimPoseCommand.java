package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.AlignmentConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AimPoseCommand extends Command {
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
        AlignmentConstants.ROTATIONAL_PID_KP,
        AlignmentConstants.ROTATIONAL_PID_KI,
        AlignmentConstants.ROTATIONAL_PID_KD,
        new TrapezoidProfile.Constraints(
            AlignmentConstants.MAXIMUM_SPEED_RADIANS,
            AlignmentConstants.MAXIMUM_ACCELERATION_RADIANS
        )
    );

    private SwerveSubsystem swerveSubsystem;
    private DoubleSupplier xAxis;
    private DoubleSupplier yAxis;
    private Supplier<Pose2d> target;

    public AimPoseCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier xAxis, DoubleSupplier yAxis, Supplier<Pose2d> target) {
        this.swerveSubsystem = swerveSubsystem;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.target = target;
        addRequirements(swerveSubsystem);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        thetaController.reset(swerveSubsystem.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d current = swerveSubsystem.getPose();
        double x = target.get().getX() - current.getX();
        double y = target.get().getY() - current.getY();

        double currentHeading = current.getRotation().getRadians();
        double desiredHeading = Math.atan2(y, x);

        swerveSubsystem.drive(
            swerveSubsystem.getVelocity(xAxis.getAsDouble(), yAxis.getAsDouble()),
            thetaController.calculate(currentHeading, desiredHeading)
        );
    }
}
