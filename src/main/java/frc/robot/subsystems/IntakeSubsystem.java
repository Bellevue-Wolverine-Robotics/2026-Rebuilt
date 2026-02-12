package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(IntakeConstants.MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    public IntakeSubsystem() {
        motorConfig.idleMode(IdleMode.kBrake);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command intake() {
        return startEnd(
            () -> motor.set(IntakeConstants.INTAKE_SPEED),
            () -> motor.stopMotor()
        );
    }

    public Command unjam() {
        return startEnd(
            () -> motor.set(IntakeConstants.UNJAM_SPEED),
            () -> motor.stopMotor()
        );
    }
}
