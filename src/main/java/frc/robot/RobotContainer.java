// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import frc.robot.constants.DriverStationConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
            () -> -driverController.getLeftX(),
            () -> -driverController.getLeftY(),
            () -> -driverController.getRightX()
        ));
    } 

    public Command getAutonomousCommand() {
        Pose2d targetPose = new Pose2d(6, 4, Rotation2d.fromDegrees(0));
        PathConstraints constraints = new PathConstraints(3, 4,  Units.degreesToRadians(540), Units.degreesToRadians(720));
        return AutoBuilder.pathfindToPose(targetPose, constraints, 0);
    }
}
