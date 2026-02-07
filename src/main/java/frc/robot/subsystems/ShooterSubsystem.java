package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final VelocityVoltage velocityRequest = new VelocityVoltage(-1.0d).withSlot(0);

    private final TalonFX rollerMotor = new TalonFX(ShooterConstants.ROLLER_MOTOR_ID);

    private final TalonFX flywheelMasterMotor = new TalonFX(ShooterConstants.LEFT_MOTOR_ID);
    private final TalonFX flywheelFollowerMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID);

    public ShooterSubsystem() {
        configureFlywheelMotors(flywheelMasterMotor, flywheelFollowerMotor);

        var brakeConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        rollerMotor.getConfigurator().apply(brakeConfig);
    }

    private static boolean withinTolerance(double value, double goal, double tolerance) {
        return Math.abs(value - goal) < tolerance;
    }

    private static void configureFlywheelMotors(TalonFX master, TalonFX follower) {
        Slot0Configs PIDConfig = new Slot0Configs();
        PIDConfig.kS = ShooterConstants.STATIC_FRICTION_OVERCOME_VOLTAGE;
        PIDConfig.kV = ShooterConstants.VOLTAGE_PER_MOTOR_RPS;
        PIDConfig.kP = ShooterConstants.PROPORTIONAL_GAIN;
        PIDConfig.kI = ShooterConstants.INTEGRAL_GAIN;
        PIDConfig.kD = ShooterConstants.DERIVATIVE_GAIN;

        master.getConfigurator().apply(PIDConfig);

        var invertedConfig = new TalonFXConfiguration();
        invertedConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        follower.getConfigurator().apply(invertedConfig);
    }

    private void updateFlywheelMotorDesiredSpeed(double rotationsPerSecond) {
        flywheelMasterMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    private double calculateFlywheelMotorSpeed(double distance) {
        return 
            // AX^2 + BX^1 + CX^0 or ax^2 + bx + c
            ShooterConstants.DIST_KX2 * distance * distance +
            ShooterConstants.DIST_KX1 * distance +
            ShooterConstants.DIST_KX0;
    }

    public Command shootCommand(DoubleSupplier distance) {
        return new RunCommand(
            () -> {
                updateFlywheelMotorDesiredSpeed(calculateFlywheelMotorSpeed(distance.getAsDouble()));
                if (withinTolerance(
                        flywheelMasterMotor.getVelocity().getValueAsDouble(), 
                        velocityRequest.Velocity, 
                        ShooterConstants.ACCEPTABLE_RPS_TOLERANCE)) {
                    rollerMotor.set(ShooterConstants.ROLLER_VELOCITY);
                }
            },
            this).finallyDo(
                () -> flywheelMasterMotor.setControl(velocityRequest.withVelocity(0.0d)));
    }
}
