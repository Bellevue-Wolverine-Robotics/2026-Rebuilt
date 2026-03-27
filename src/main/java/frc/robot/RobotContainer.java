// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.AlignPoseCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.FixedShootCommand;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.DriverStationConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(ledSubsystem);
    @SuppressWarnings("unused")  // Subsystems automatically register themselves to command scheduler
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);

    private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(DriverStationConstants.OPERATOR_CONTROLLER_PORT);

    private final SendableChooser<Command> sendableChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configureAutonomous();
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

        driverController.rightTrigger().whileTrue(new AutoShootCommand(
            swerveSubsystem,
            ledSubsystem,
            shooterSubsystem,
            feederSubsystem,
            () -> -MathUtil.applyDeadband(driverController.getLeftY(), DriverStationConstants.DRIVER_CONTROLLER_LEFT_DEADBAND),
            () -> -MathUtil.applyDeadband(driverController.getLeftX(), DriverStationConstants.DRIVER_CONTROLLER_LEFT_DEADBAND)
        ));

        driverController.start().onTrue(swerveSubsystem.zeroGyro());

        driverController.y().whileTrue(swerveSubsystem.alignAndShootCommand(shooterSubsystem, feederSubsystem));

        driverController.x().whileTrue(getAutoClimbCommand(
            VisionConstants.LEFT_TOWER_APPROACH_POSE_SUPPLIER,
            VisionConstants.LEFT_TOWER_FINAL_POSE_SUPPLIER
        ));

        driverController.b().whileTrue(getAutoClimbCommand(
            VisionConstants.RIGHT_TOWER_APPROACH_POSE_SUPPLIER,
            VisionConstants.RIGHT_TOWER_FINAL_POSE_SUPPLIER
        ));

        driverController.a().whileTrue(new AlignPoseCommand(
            swerveSubsystem, 
            ledSubsystem, 
            VisionConstants.OUTPOST_POSE_SUPPLIER
        ));

        operatorController.leftBumper().whileTrue(armSubsystem.extendCommand());
        operatorController.rightBumper().whileTrue(armSubsystem.extendCommand());
        operatorController.rightTrigger().whileTrue(intakeSubsystem.intakeCommand());

        new Trigger(
            () -> Math.abs(operatorController.getLeftY()) > DriverStationConstants.OPERATOR_CONTROLLER_LEFT_DEADBAND
        ).whileTrue(
            intakeSubsystem.runCommand(() -> MathUtil.applyDeadband(operatorController.getLeftY(), DriverStationConstants.OPERATOR_CONTROLLER_LEFT_DEADBAND))
        );

        new Trigger(
            () -> Math.abs(operatorController.getRightY()) > DriverStationConstants.OPERATOR_CONTROLLER_RIGHT_DEADBAND
        ).whileTrue(
            armSubsystem.moveCommand(() -> -MathUtil.applyDeadband(operatorController.getRightY(), DriverStationConstants.OPERATOR_CONTROLLER_RIGHT_DEADBAND))
        );

        operatorController.pov(0).onTrue(climberSubsystem.extendCommand());
        operatorController.pov(180).onTrue(climberSubsystem.retractCommand());

        operatorController.a().whileTrue(new FixedShootCommand(
            shooterSubsystem,
            feederSubsystem,
            ShooterConstants.MANUAL_SHOOT_DISTANCE_METERS
        ));

        operatorController.b().whileTrue(new FixedShootCommand(
            shooterSubsystem,
            feederSubsystem,
            ShooterConstants.PASS_SHOOT_DISTANCE_METERS
        ));
    }

    private void configureAutonomous() {
        NamedCommands.registerCommand("extend", armSubsystem.extendCommand());
        NamedCommands.registerCommand("intake", intakeSubsystem.intakeCommand());
        NamedCommands.registerCommand("shoot", new AutoShootCommand(
            swerveSubsystem,
            ledSubsystem,
            shooterSubsystem,
            feederSubsystem,
            () -> 0,
            () -> 0
        ));

        sendableChooser.setDefaultOption(
            "Default",
            new PathPlannerAuto("DEFAULT")
        );
        sendableChooser.addOption(
            "Outpost",
            new PathPlannerAuto("OUTPOST")
        );
        sendableChooser.addOption(
            "Neutral Depot",
            new PathPlannerAuto("NEUTRAL_DEPOT")
        );
        sendableChooser.addOption(
            "Left Two Cycle",
            new PathPlannerAuto("TWO_CYCLE")
        );
        sendableChooser.addOption(
            "Right Two Cycle",
            new PathPlannerAuto("TWO_CYCLE", true)
        );
        sendableChooser.addOption(
            "Left Two Cycle Loop",
            new PathPlannerAuto("TWO_CYCLE_LOOP")
        );
        sendableChooser.addOption(
            "Right Two Cycle Loop",
            new PathPlannerAuto("TWO_CYCLE_LOOP", true)
        );
        sendableChooser.addOption(
            "Preload Depot",
            new PathPlannerAuto("PRELOAD_DEPOT")
        );

        SmartDashboard.putData("Auto Chooser", sendableChooser);
    }

    public Command getAutonomousCommand() {
        return sendableChooser.getSelected();
    }

    private Command getAutoClimbCommand(Supplier<Pose2d> approachPose, Supplier<Pose2d> finalPose) {
        return Commands.sequence(
            new AlignPoseCommand(swerveSubsystem, ledSubsystem, approachPose),
            climberSubsystem.extendCommand(),
            new WaitCommand(ClimberConstants.EXTENSION_WAIT_SECONDS),
            new AlignPoseCommand(swerveSubsystem, ledSubsystem, finalPose),
            climberSubsystem.retractCommand()
        ).beforeStarting(() -> ledSubsystem.setClimbing(true)).finallyDo(() -> ledSubsystem.setClimbing(false));
    }
}
