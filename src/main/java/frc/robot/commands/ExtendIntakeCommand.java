package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** A command that extends the intake arm if it is not already down, and then runs the rollers to intake fuel. */
public class ExtendIntakeCommand extends SequentialCommandGroup {
    /** Constructs a new ExtendIntakeCommand. */
    public ExtendIntakeCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(armSubsystem.extendCommand(), intakeSubsystem.intakeCommand());
    }
}
