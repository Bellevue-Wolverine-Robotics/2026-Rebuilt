package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.AlignmentConstants;
import frc.robot.subsystems.LEDSubsystem;
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

    private final SwerveSubsystem swerveSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final DoubleSupplier xAxis;
    private final DoubleSupplier yAxis;
    private final Supplier<Pose2d> target;

    public AimPoseCommand(
        SwerveSubsystem swerveSubsystem,
        LEDSubsystem ledSubsystem,
        DoubleSupplier xAxis,
        DoubleSupplier yAxis,
        Supplier<Pose2d> target
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        this.target = target;
        addRequirements(swerveSubsystem);

        thetaController.setTolerance(AlignmentConstants.ROTATIONAL_TOLERANCE_DISTANCE, AlignmentConstants.ROTATIONAL_TOLERANCE_VELOCITY);
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

        ledSubsystem.setAligning(true);

        swerveSubsystem.drive(
            swerveSubsystem.inputToTranslation(xAxis.getAsDouble(), yAxis.getAsDouble()),
            thetaController.calculate(currentHeading, desiredHeading)
        );
    }

    @Override
    public void end(boolean interrupted) {
        ledSubsystem.setAligning(false);
    }
}
