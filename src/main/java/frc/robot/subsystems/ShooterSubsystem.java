package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final float ALLOWABLE_ZERO_DIST = 0.001f;

    private final TalonFX flywheelMasterMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID);
    private final TalonFX flywheelFollowerMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID);

    public ShooterSubsystem() {
        var invertedConfig = new TalonFXConfiguration();
        invertedConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        flywheelFollowerMotor.getConfigurator().apply(invertedConfig);
    }

    private double calculateFlywheelSpeed(double distance) {
        // TODO: Add flywheel speed calculations
        return -1.0f;
    }

    public Command shootCommand(DoubleSupplier distance, DoubleSupplier override) {
        return new RunCommand(
            () -> flywheelMasterMotor.set(
                Math.abs(override.getAsDouble()) < ALLOWABLE_ZERO_DIST ? 
                    calculateFlywheelSpeed(distance.getAsDouble()) :
                    override.getAsDouble()),
            this);
    }
}
