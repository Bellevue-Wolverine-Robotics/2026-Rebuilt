package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leftMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID, ShooterConstants.CANBUS);
    private final TalonFX rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID, ShooterConstants.CANBUS);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    // The internal setpoint of the motor controller does not clear when you stop the motor
    private double targetRPS = 0.0;

    public ShooterSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = ShooterConstants.PROPORTIONAL_GAIN;
        config.Slot0.kI = ShooterConstants.INTEGRAL_GAIN;
        config.Slot0.kD = ShooterConstants.DERIVATIVE_GAIN;
        config.Slot0.kS = ShooterConstants.STATIC_FRICTION_OVERCOME_VOLTAGE;
        config.Slot0.kV = ShooterConstants.VOLTS_PER_RPS;

        config.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = ShooterConstants.LEFT_MOTOR_INVERTED ?  InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        leftMotor.getConfigurator().apply(config);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        rightMotor.setNeutralMode(NeutralModeValue.Coast);

        SmartDashboard.putNumber("Shooter/RPS Override", -1);
    }

    private double calculateSpeed(double distance) {
        double override = SmartDashboard.getNumber("Shooter/RPS Override", -1);

        if (override >= 0) {
            return override;
        }
        return ShooterConstants.DISTANCE_METERS_TO_RPS.get(distance);
    }

    public boolean atSpeed() {
        double currentRPS = leftMotor.getVelocity().getValueAsDouble();
        return Math.abs(targetRPS - currentRPS) < (ShooterConstants.TOLERANCE_PERCENTAGE * targetRPS);
    }

    public void run(double distance) {
        targetRPS = calculateSpeed(distance);
        leftMotor.setControl(velocityRequest.withVelocity(targetRPS));
    }

    public void stop() {
        targetRPS = 0.0;
        leftMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Target Flywheel RPS", targetRPS);
        SmartDashboard.putNumber("Shooter/Current Flywheel RPS", leftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Applied Voltage", leftMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Supply Current", leftMotor.getSupplyCurrent().getValueAsDouble());
    }
}
