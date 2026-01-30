package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignPoseCommand extends Command {
    private final HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(
            SwerveConstants.AUTOALIGN_TRANSLATIONAL_PID_KP,
            SwerveConstants.AUTOALIGN_TRANSLATIONAL_PID_KI,
            SwerveConstants.AUTOALIGN_TRANSLATIONAL_PID_KD
        ),
        new PIDController(
            SwerveConstants.AUTOALIGN_TRANSLATIONAL_PID_KP,
            SwerveConstants.AUTOALIGN_TRANSLATIONAL_PID_KI,
            SwerveConstants.AUTOALIGN_TRANSLATIONAL_PID_KD
        ),
        new ProfiledPIDController(
            SwerveConstants.AUTOALIGN_ROTATIONAL_PID_KP,
            SwerveConstants.AUTOALIGN_ROTATIONAL_PID_KI,
            SwerveConstants.AUTOALIGN_ROTATIONAL_PID_KD,
            new TrapezoidProfile.Constraints(
                SwerveConstants.AUTOALIGN_MAXIMUM_SPEED_RADIANS,
                SwerveConstants.AUTOALIGN_MAXIMUM_ACCELERATION_RADIANS
            )
        )
    );

    private SwerveSubsystem swerveSubsystem;
    private Supplier<Pose2d> target;

    public AlignPoseCommand(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> target) {
        this.swerveSubsystem = swerveSubsystem;
        this.target = target;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        Pose2d current = swerveSubsystem.getPose();
        ChassisSpeeds chassisSpeeds = controller.calculate(current, target.get(), 0.0, target.get().getRotation());
        swerveSubsystem.drive(chassisSpeeds);
    }
}
