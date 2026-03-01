package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FixedShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final double distance;

    public FixedShootCommand(
        ShooterSubsystem shooterSubsystem,
        FeederSubsystem feederSubsystem,
        double distance
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.feederSubsystem = feederSubsystem;
        this.distance = distance;

        addRequirements(shooterSubsystem, feederSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.run(distance);

        if (shooterSubsystem.atSpeed()) {
            feederSubsystem.run();
        } else {
            feederSubsystem.stop();
        }

    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.stop();
        shooterSubsystem.stop();
    }
}
