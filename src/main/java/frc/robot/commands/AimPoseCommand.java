package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AimPoseCommand extends Command {
    private final ProfiledPIDController thetaConroller = new ProfiledPIDController(
        SwerveConstants.AUTOALIGN_ROTATIONAL_PID_KP,
        SwerveConstants.AUTOALIGN_ROTATIONAL_PID_KI,
        SwerveConstants.AUTOALIGN_ROTATIONAL_PID_KD,
        new TrapezoidProfile.Constraints(
            SwerveConstants.AUTOALIGN_MAXIMUM_SPEED_RADIANS,
            SwerveConstants.AUTOALIGN_MAXIMUM_ACCELERATION_RADIANS
        )
    );

    private SwerveSubsystem swerveSubsystem;
    private DoubleSupplier translationX;
    private DoubleSupplier translationY;
    private Supplier<Pose2d> target;

    public AimPoseCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY, Supplier<Pose2d> target) {
        this.swerveSubsystem = swerveSubsystem;
        this.translationX = translationX;
        this.translationY = translationY;
        this.target = target;
        addRequirements(swerveSubsystem);
        thetaConroller.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        Pose2d current = swerveSubsystem.getPose();
        double x = target.get().getX() - current.getX();
        double y = target.get().getY() - current.getY();

        double desiredHeading = Math.atan2(y, x);
        double currentHeading = current.getRotation().getRadians();
        System.out.print(currentHeading + " | " + desiredHeading);

        swerveSubsystem.drive(
            translationX.getAsDouble(),
            translationY.getAsDouble(),
            thetaConroller.calculate(currentHeading, desiredHeading)
        );
    }
}
    