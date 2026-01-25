package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_flywheelMasterMotor = new TalonFX(ShooterConstants.kLeftMotorId);
    private final TalonFX m_flywheelFollowerMotor = new TalonFX(ShooterConstants.kRightMotorId);

    public ShooterSubsystem() {
        var invertedConfig = new TalonFXConfiguration();
        invertedConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_flywheelFollowerMotor.getConfigurator().apply(invertedConfig);
    }

    private double calculateFlywheelSpeed(double distance) {
        // TODO: Add flywheel speed calculations
        return -1.0f;
    }

    public Command shootCommand(double distance) {
        return new RunCommand(
            () -> m_flywheelMasterMotor.set(calculateFlywheelSpeed(distance)),
            this);
    }
}
