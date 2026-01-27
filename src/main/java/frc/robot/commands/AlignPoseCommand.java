package frc.robot.commands;

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
            SwerveConstants.PATHPLANNER_TRANSLATIONAL_PID_KP,
            SwerveConstants.PATHPLANNER_TRANSLATIONAL_PID_KI,
            SwerveConstants.PATHPLANNER_TRANSLATIONAL_PID_KD
        ),
        new PIDController(
            SwerveConstants.PATHPLANNER_TRANSLATIONAL_PID_KP,
            SwerveConstants.PATHPLANNER_TRANSLATIONAL_PID_KI,
            SwerveConstants.PATHPLANNER_TRANSLATIONAL_PID_KD
        ),
        new ProfiledPIDController(
            SwerveConstants.PATHPLANNER_ROTATIONAL_PID_KP,
            SwerveConstants.PATHPLANNER_ROTATIONAL_PID_KI,
            SwerveConstants.PATHPLANNER_ROTATIONAL_PID_KD,
            new TrapezoidProfile.Constraints(
                SwerveConstants.PATHPLANNER_MAXIMUM_SPEED_RADIANS,
                SwerveConstants.PATHPLANNER_MAXIMUM_ACCELERATION_RADIANS
            )
        )
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
