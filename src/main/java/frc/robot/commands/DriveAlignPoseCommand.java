package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.SwerveSubsystem;

public class DriveAlignPoseCommand extends SequentialCommandGroup {
    public DriveAlignPoseCommand(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> pose) {
        addCommands(
            swerveSubsystem.drivePoseCommand(pose),
            swerveSubsystem.alignPoseCommand(pose)
        );
    }
}