// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.constants.DriverStationConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(armSubsystem);
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(ledSubsystem);
    @SuppressWarnings("unused")  // Subsystems automatically register themselves to command scheduler
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);

    private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(DriverStationConstants.OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> sendableChooser = new SendableChooser<>();
    private boolean retract = true;

    public RobotContainer() {
        configureBindings();
        sendableChooser.setDefaultOption(
                "Left One Cycle",
                new PathPlannerAuto("ONE_CYCLE")
        );
        sendableChooser.addOption(
                "Left Two Cycle",
                new PathPlannerAuto("TWO_CYCLE")
        );
        sendableChooser.setDefaultOption(
                "Right One Cycle",
                new PathPlannerAuto("ONE_CYCLE", true)
        );
        sendableChooser.addOption(
                "Right Two Cycle",
                new PathPlannerAuto("TWO_CYCLE", true)
        );
        sendableChooser.addOption(
                "Depot Left Tower",
                new PathPlannerAuto("DEPOT_LEFT_TOWER")
        );
        sendableChooser.addOption(
                "Depot Right Tower",
                new PathPlannerAuto("DEPOT_RIGHT_TOWER")
        );

        SmartDashboard.putData("Auto Chooser", sendableChooser);
    }

    private void configureBindings() {
        /* Robot Coordinate    Joystick Coordinate
         *    Space:               Space:
         * 
         *       X+                  Y-
         *       |                   |
         *  Y+ ----- Y-         X- ----- X+
         *       |                   | 
         *       X-                  Y+
         * 
         * Therefore, the joystick's Y and X must be switched and
         * their sign must be inverted to work in swerve's coordinate system.
         */
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
            () -> -MathUtil.applyDeadband(driverController.getLeftY(), DriverStationConstants.DRIVER_CONTROLLER_LEFT_DEADBAND),
            () -> -MathUtil.applyDeadband(driverController.getLeftX(), DriverStationConstants.DRIVER_CONTROLLER_LEFT_DEADBAND),
            () -> -MathUtil.applyDeadband(driverController.getRightX(), DriverStationConstants.DRIVER_CONTROLLER_RIGHT_DEADBAND)
        ));

        driverController.rightTrigger().whileTrue(swerveSubsystem.driveCommand(
            () -> -MathUtil.applyDeadband(driverController.getLeftY(), DriverStationConstants.DRIVER_CONTROLLER_LEFT_DEADBAND),
            () -> -MathUtil.applyDeadband(driverController.getLeftX(), DriverStationConstants.DRIVER_CONTROLLER_LEFT_DEADBAND),
            VisionConstants.HUB_POSE_SUPPLIER
        ));

        driverController.start().onTrue(swerveSubsystem.zeroGyro());

        driverController.y().whileTrue(swerveSubsystem.alignPoseCommand(VisionConstants.NEUTRAL_POSE_SUPPLIER));
        driverController.a().whileTrue(swerveSubsystem.alignPoseCommand(VisionConstants.SHOOT_POSE_SUPPLIER));
        driverController.b().whileTrue(swerveSubsystem.alignPoseCommand(VisionConstants.CLIMB_POSE_SUPPLIER));

        operatorController.rightTrigger().whileTrue(armSubsystem.extendCommand().andThen(armSubsystem.waitCommand(
            () -> operatorController.setRumble(
                GenericHID.RumbleType.kRightRumble,
                DriverStationConstants.OPERATOR_ARM_WARNING_RUMBLE_POWER
            )
        ))).onFalse(armSubsystem.retractCommand().onlyIf(() -> retract).until(() -> !retract));

        operatorController.leftTrigger().whileTrue(
            intakeSubsystem.runCommand(() -> MathUtil.applyDeadband(operatorController.getLeftY(), DriverStationConstants.OPERATOR_CONTROLLER_LEFT_DEADBAND))
        );

        operatorController.rightBumper().onTrue(Commands.runOnce(() -> retract = !retract));

        new Trigger(
            () -> Math.abs(operatorController.getRightY()) > DriverStationConstants.OPERATOR_CONTROLLER_RIGHT_DEADBAND
        ).whileTrue(
            armSubsystem.moveCommand(() -> -MathUtil.applyDeadband(operatorController.getLeftY(), DriverStationConstants.OPERATOR_CONTROLLER_LEFT_DEADBAND))
        );
    }
    public Command getAutonomousCommand() {
        return sendableChooser.getSelected();
    }

}
