package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.ShooterConstants;

public class FeederSubsystem extends LoggedSubsystemBase {
    private boolean running = false;

    private TalonFX motor = new TalonFX(FeederConstants.MOTOR_ID, FeederConstants.CANBUS);

    public FeederSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = ShooterConstants.LEFT_MOTOR_INVERTED ? 
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(config);
    }

    public void run() {
        running = true;
        motor.set(FeederConstants.FEED_SPEED);
    }

    public void stop() {
        running = false;
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Feeder/Is Running", running);
        SmartDashboard.putNumber("Feeder/Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Feeder/Motor Current", motor.getSupplyCurrent().getValueAsDouble());
    }
}
