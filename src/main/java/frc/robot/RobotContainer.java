// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.ShootCommand;
import frc.robot.constants.DriverStationConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(ledSubsystem);
    @SuppressWarnings("unused")  // Subsystems automatically register themselves to command scheduler
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);

    private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(DriverStationConstants.OPERATOR_CONTROLLER_PORT);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
            () -> -MathUtil.applyDeadband(driverController.getLeftY(), DriverStationConstants.DRIVER_CONTROLLER_LEFT_DEADBAND),
            () -> -MathUtil.applyDeadband(driverController.getLeftX(), DriverStationConstants.DRIVER_CONTROLLER_LEFT_DEADBAND),
            () -> -MathUtil.applyDeadband(driverController.getRightX(), DriverStationConstants.DRIVER_CONTROLLER_RIGHT_DEADBAND)
        ));

        driverController.rightTrigger().whileTrue(new ShootCommand(
            swerveSubsystem, 
            ledSubsystem,
            shooterSubsystem,
            feederSubsystem,
            () -> -driverController.getLeftY(), 
            () -> -driverController.getLeftX(), 
            VisionConstants.HUB_POSE_SUPPLIER
        ));

        driverController.start().onTrue(swerveSubsystem.zeroGyro());
        driverController.y().whileTrue(swerveSubsystem.alignPoseCommand(VisionConstants.NEUTRAL_POSE_SUPPLIER));
        driverController.a().whileTrue(swerveSubsystem.alignPoseCommand(VisionConstants.SHOOT_POSE_SUPPLIER));
        driverController.b().whileTrue(swerveSubsystem.alignPoseCommand(VisionConstants.CLIMB_POSE_SUPPLIER));

        operatorController.x().whileTrue(shooterSubsystem.shootDistanceCommand(ShooterConstants.PASS_SHOOT_METERS, feederSubsystem));
        operatorController.b().whileTrue(shooterSubsystem.shootDistanceCommand(ShooterConstants.SHORT_SHOOT_METERS, feederSubsystem));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Basic");
    }
}
