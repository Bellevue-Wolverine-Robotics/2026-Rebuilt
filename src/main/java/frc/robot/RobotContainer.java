// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import frc.robot.constants.DriverStationConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LedLightSubsystem;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);
    private final LedLightSubsystem ledLightSubsystem = new LedLightSubsystem();

    private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(DriverStationConstants.OPERATOR_CONTROLLER_PORT);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Swerve drive controls
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()
        ));

        driverController.start().onTrue(swerveSubsystem.zeroGyro());

        // ====== LED TEST BUTTONS (for testing alignment function) ======
        // These buttons simulate alignment status for testing purposes

        // A button - Simulate ALIGNED (green)
        operatorController.a().onTrue(
                Commands.runOnce(() -> ledLightSubsystem.isAligned(true))
        );

        // B button - Simulate NOT ALIGNED (red)
        operatorController.b().onTrue(
                Commands.runOnce(() -> ledLightSubsystem.isAligned(false))
        );

        // ====== REAL USAGE EXAMPLE ======
        // In actual use, you would call isAligned() from your subsystem
        // based on real sensor data or calculations. Examples:

        /*
        // Example 1: Update LEDs based on vision alignment in a command
        Command alignToTarget = Commands.run(() -> {
            // Your alignment logic here
            boolean aligned = checkIfAlignedToTarget(); // Your method
            ledLightSubsystem.isAligned(aligned);
        });

        // Example 2: Continuous alignment check in another subsystem's periodic
        // In your VisionSubsystem.periodic() or SwerveSubsystem.periodic():
        // ledLightSubsystem.isAligned(Math.abs(angleError) < 2.0);

        // Example 3: Check alignment on button hold
        driverController.rightTrigger().whileTrue(
            Commands.run(() -> {
                boolean aligned = isRobotAlignedToTarget();
                ledLightSubsystem.isAligned(aligned);
            })
        );
        */
    }

    /**
     * Example method showing how to check alignment
     * Replace this with your actual alignment logic
     */
    private boolean checkIfAlignedToTarget() {
        // Example: Check if robot is pointed at target within tolerance
        // This is just a placeholder - use your actual alignment logic

        // Example using vision:
        // return Math.abs(visionSubsystem.getAngleToTarget()) < 2.0;

        // Example using pose:
        // Pose2d currentPose = swerveSubsystem.getPose();
        // return currentPose.getRotation().getDegrees() < 5.0;

        return false; // Placeholder
    }

    public Command getAutonomousCommand() {
        Pose2d targetPose = new Pose2d(5, 3, Rotation2d.fromDegrees(0));
        PathConstraints constraints = new PathConstraints(3, 4,  Units.degreesToRadians(540), Units.degreesToRadians(720));
        return AutoBuilder.pathfindToPose(targetPose, constraints, 0);
    }
}