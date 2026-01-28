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

        // LED Animation Controls (Operator Controller)
        // Default mode is already flowing gradient (set automatically in constructor)

        // A button - Flowing Gradient (smooth blue/yellow wave)
        operatorController.a().onTrue(
                Commands.runOnce(() -> ledLightSubsystem.setFlowingGradient())
        );

        // B button - Flowing Blocks (alternating blue/yellow blocks)
        operatorController.b().onTrue(
                Commands.runOnce(() -> ledLightSubsystem.setFlowingBlocks())
        );

        // X button - Flowing Wave (sine wave pattern)
        operatorController.x().onTrue(
                Commands.runOnce(() -> ledLightSubsystem.setFlowingWave())
        );

        // Y button - Solid Blue
        operatorController.y().onTrue(
                Commands.runOnce(() -> ledLightSubsystem.setBlue())
        );

        // Left Bumper - Solid Yellow
        operatorController.leftBumper().onTrue(
                Commands.runOnce(() -> ledLightSubsystem.setYellow())
        );

        // Right Bumper - Turn Off
        operatorController.rightBumper().onTrue(
                Commands.runOnce(() -> ledLightSubsystem.turnOff())
        );
    }

    public Command getAutonomousCommand() {
        Pose2d targetPose = new Pose2d(5, 3, Rotation2d.fromDegrees(0));
        PathConstraints constraints = new PathConstraints(3, 4,  Units.degreesToRadians(540), Units.degreesToRadians(720));
        return AutoBuilder.pathfindToPose(targetPose, constraints, 0);
    }
}