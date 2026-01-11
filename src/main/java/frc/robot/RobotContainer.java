package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.constants.DriverStationConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.DRIVER_CONTROLLER_PORT);

    // Speed control - start slow for testing
    private double speedMultiplier = 0.3;  // 30% speed

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
                () -> -driverController.getLeftY() * speedMultiplier,   // Forward/Backward
                () -> -driverController.getLeftX() * speedMultiplier,   // Left/Right
                () -> -driverController.getRightX() * speedMultiplier   // Rotation
        ));

        // Right Bumper: Hold for TURBO mode (100% speed)
        driverController.rightBumper().onTrue(Commands.runOnce(() -> {
            speedMultiplier = 1.0;
        })).onFalse(Commands.runOnce(() -> {
            speedMultiplier = 0.3;
        }));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}