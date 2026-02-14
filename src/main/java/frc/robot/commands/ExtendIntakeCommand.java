package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendIntakeCommand extends SequentialCommandGroup {
    public ExtendIntakeCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(armSubsystem.extendCommand(), intakeSubsystem.intakeCommand());
    }
}
