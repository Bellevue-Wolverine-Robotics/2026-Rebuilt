package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;

public class AlignPoseCommand extends Command {
    private final HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(5.0, 0.0, 0.0),
        new PIDController(5.0, 0.0, 0.0),
        new ProfiledPIDController(5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(6.28, 3.14))
    );

    private SwerveSubsystem swerveSubsystem;
    private Pose2d target;

    public AlignPoseCommand(SwerveSubsystem swerveSubsystem, Pose2d target) {
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        Pose2d current = swerveSubsystem.getPose();
        ChassisSpeeds chassisSpeeds = controller.calculate(current, target, 0.0, target.getRotation());
        swerveSubsystem.drive(chassisSpeeds);
    }
}
