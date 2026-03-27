package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.ArmConstants;

/** Represents the arm mechanism, which moves the linkage intake. */
public class ArmSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(ArmConstants.MOTOR_ID, MotorType.kBrushless);
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_PORT);
    private final RelativeEncoder relativeEncoder = motor.getEncoder();
    private final SparkClosedLoopController controller = motor.getClosedLoopController();

    /** Constructs a new ArmSubsystem. */
    public ArmSubsystem() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.inverted(ArmConstants.MOTOR_INVERTED);

        motorConfig.encoder.positionConversionFactor((2.0 * Math.PI) /  ArmConstants.GEAR_RATIO);
        // Divide by 60 so units are in radians per second instead of radians per minute
        motorConfig.encoder.velocityConversionFactor((2.0 * Math.PI) / ArmConstants.GEAR_RATIO / 60.0);

        motorConfig.closedLoop
            .p(ArmConstants.PROPORTIONAL_GAIN)
            .i(ArmConstants.INTEGRAL_GAIN)
            .d(ArmConstants.DERIVATIVE_GAIN);

        motorConfig.closedLoop.feedForward
            .kS(ArmConstants.STATIC_FRICTION_OVERCOME_VOLTAGE)
            .kV(ArmConstants.VOLTS_PER_RADIAN_PER_SECOND)
            .kA(ArmConstants.INTERTIA_OVERCOME_VOLTAGE)
            .kCos(ArmConstants.GRAVITY_OVERCOME_VOLTAGE)
            .kCosRatio(1.0);

        motorConfig.closedLoop.allowedClosedLoopError(ArmConstants.ERROR_TOLERANCE_RADIANS, ClosedLoopSlot.kSlot0);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        absoluteEncoder.setInverted(ArmConstants.ABSOLUTE_ENCODER_INVERTED);

        synchronize();
    }

    private void synchronize() {
        double position = (absoluteEncoder.get() - ArmConstants.ABSOLUTE_ENCODER_OFFSET_DUTY_CYCLE + 1) % 1;
        relativeEncoder.setPosition(position * (2.0 * Math.PI));
    }

    private void set(double setpoint) {
        controller.setSetpoint(setpoint, ControlType.kPosition);
    }

    /**
     * Provides a command that extends the arm until finished.
     * 
     * @return The extension command.
     */
    public Command extendCommand() {
        return runEnd(
            () -> set(ArmConstants.EXTENDED_ANGLE_RADIANS),
            motor::stopMotor
        ).beforeStarting(this::synchronize).until(controller::isAtSetpoint);
    }

    /**
     * Provides a command that retracts the arm until finished.
     * 
     * @return The retraction command.
     */
    public Command retractCommand() {
        return runEnd(
            () -> set(ArmConstants.RETRACTED_ANGLE_RADIANS),
            motor::stopMotor
        ).beforeStarting(this::synchronize).until(controller::isAtSetpoint);
    }

    /**
     * Provides a command sets the arm's motor to a manual speed.
     * This is used in instances which the encoder is broken, so that the arm and intake are still usable.
     * 
     * @param speed The speed to run the motor at as a percentage on [-1, 1].
     * @return The movement command.
     */
    public Command moveCommand(DoubleSupplier speed) {
        return runEnd(
            () -> motor.set(speed.getAsDouble() * ArmConstants.MANUAL_CONTROL_COFFICIENT),
            () -> motor.stopMotor()
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Absolute Encoder Position", absoluteEncoder.get());
        SmartDashboard.putNumber("Arm/Relative Encoder Position", relativeEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Relative Encoder Velocity", relativeEncoder.getVelocity());
        SmartDashboard.putNumber("Arm/Applied Motor Voltage", motor.getAppliedOutput() * motor.getBusVoltage());
    }
}
