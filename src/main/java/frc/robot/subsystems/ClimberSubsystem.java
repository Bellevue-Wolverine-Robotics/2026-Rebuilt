package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(ClimberConstants.MOTOR_CAN_ID, MotorType.kBrushless);

    public ClimberSubsystem() {
        setDefaultCommand(idle());
    }

    public Command idle() {
        return Commands.run(() -> motor.stopMotor(), this);
    }

    public Command extend() {
        return Commands.run(() -> motor.set(ClimberConstants.EXTEND_SPEED), this);
    }

    public Command retract() {
        return Commands.run(() -> motor.set(ClimberConstants.RETRACT_SPEED), this);
    }
}
