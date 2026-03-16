package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.ShooterConstants;

public class FeederSubsystem extends SubsystemBase {
    private TalonFX motor = new TalonFX(FeederConstants.MOTOR_ID, "canivore");

    public FeederSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = ShooterConstants.LEFT_MOTOR_INVERTED ? 
        InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(config);
    }

    public void run() {
        motor.set(FeederConstants.FEED_SPEED);
    }

    public void stop() {
        motor.stopMotor();
    }
}
