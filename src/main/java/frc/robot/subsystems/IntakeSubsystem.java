package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.IntakeConstants;

/** Represents the intake mechanism, which draws fuel into the hopper. */
public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(IntakeConstants.MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    /** Constructs a new IntakeSubsystem. */
    public IntakeSubsystem() {
        motorConfig.idleMode(IdleMode.kCoast);
        motorConfig.inverted(IntakeConstants.MOTOR_INVERTED);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Provides a command that runs the motor to intake fuel.
     * 
     * @return The intake command.
     */
    public Command intakeCommand() {
        return runEnd(
            () -> motor.set(IntakeConstants.INTAKE_SPEED),
            motor::stopMotor
        );
    }

    /**
     * Provides a command sets the intake's motor to a manual speed.
     * This is used in instances which the encoder is broken, so that the arm and intake are still usable.
     * 
     * @param speed The speed to run the motor at as a percentage on [-1, 1].
     * @return The run command.
     */
    public Command runCommand(DoubleSupplier speed) {
        return runEnd(
            () -> motor.set(speed.getAsDouble()),
            motor::stopMotor
        );
    }
}
