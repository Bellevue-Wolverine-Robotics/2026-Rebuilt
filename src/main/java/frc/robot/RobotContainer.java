// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.constants.DriverStationConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    // Vision subsystem is automatically registered by command scheduler
    @SuppressWarnings("unused")
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);
    public final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(DriverStationConstants.OPERATOR_CONTROLLER_PORT);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()
        ));

        driverController.rightTrigger().whileTrue(swerveSubsystem.driveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            VisionConstants.HUB_POSE_SUPPLIER
        ));

        driverController.start().onTrue(swerveSubsystem.zeroGyro());
        driverController.y().whileTrue(swerveSubsystem.alignPoseCommand(VisionConstants.NEUTRAL_POSE_SUPPLIER));
        driverController.a().whileTrue(swerveSubsystem.alignPoseCommand(VisionConstants.SHOOT_POSE_SUPPLIER));
        driverController.b().whileTrue(swerveSubsystem.alignPoseCommand(VisionConstants.CLIMB_POSE_SUPPLIER));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Basic");
    }
}
