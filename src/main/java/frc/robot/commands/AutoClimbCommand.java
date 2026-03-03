package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoClimbCommand extends SequentialCommandGroup {
    public AutoClimbCommand(
        SwerveSubsystem swerveSubsystem,
        ClimberSubsystem climberSubsystem,
        LEDSubsystem ledSubsystem,
        Supplier<Pose2d> approachPose,
        Supplier<Pose2d> finalPose
    ) {
        addCommands(
            new AlignPoseCommand(swerveSubsystem, ledSubsystem, approachPose),
            climberSubsystem.extendCommand(),
            new WaitCommand(ClimberConstants.EXTENSION_WAIT_SECONDS),
            new AlignPoseCommand(swerveSubsystem, ledSubsystem, finalPose),
            climberSubsystem.retractCommand()
        );
    }   
}
