package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.AlignmentConstants;
import frc.robot.constants.FeederConstants;
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

    @Override
    public void initialize() {
        thetaController.reset(swerveSubsystem.getPose().getRotation().getRadians(), swerveSubsystem.getRotationalVelocity());
    }

    @Override
    public void execute() {
        Pose2d current = swerveSubsystem.getPose();
        double x = target.get().getX() - current.getX();
        double y = target.get().getY() - current.getY();

        double currentHeading = current.getRotation().getRadians();
        double desiredHeading = Math.atan2(y, x);

        ledSubsystem.setAligned(thetaController.atGoal());

        swerveSubsystem.drive(
            swerveSubsystem.inputToTranslation(xAxis.getAsDouble(), yAxis.getAsDouble()),
            thetaController.calculate(currentHeading, desiredHeading)
        );

        double distance = Math.hypot(x, y);
        shooterSubsystem.run(distance);

        if (
            swerveSubsystem.getTranslationalVelocity() <  FeederConstants.MAXIMUM_ACCEPTABLE_ROBOT_SPEED
            && thetaController.atGoal()
            && shooterSubsystem.atSpeed()
        ) {
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
