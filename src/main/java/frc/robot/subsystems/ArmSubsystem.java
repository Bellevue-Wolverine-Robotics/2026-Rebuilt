package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.ArmConstants;

public class ArmSubsystem extends LoggedSubsystemBase {
    private final ProfiledPIDController controller = new ProfiledPIDController(
        ArmConstants.PROPORTIONAL_GAIN,
        ArmConstants.INTEGRAL_GAIN,
        ArmConstants.DERIVATIVE_GAIN,
        new TrapezoidProfile.Constraints(
            ArmConstants.MAXIMUM_VELOCITY,
            ArmConstants.MAXIMUM_ACCELERATION
        )
    );

    private final SparkMax motor = new SparkMax(ArmConstants.MOTOR_ID, MotorType.kBrushless);
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_PORT);

    public ArmSubsystem() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.inverted(ArmConstants.MOTOR_INVERTED);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        absoluteEncoder.setInverted(ArmConstants.ABSOLUTE_ENCODER_INVERTED);
    }

    private double getPosition() {
        return MathUtil.inputModulus(absoluteEncoder.get() - ArmConstants.ABSOLUTE_ENCODER_OFFSET_DUTY_CYCLE, -0.5, 0.5) * (2.0 * Math.PI);
    }

    private void set(double setpoint) {
        motor.set(controller.calculate(getPosition(), setpoint));
    }

    /**
     * Provides a command that extends the arm until finished.
     * 
     * @return The extension until finished command.
     */
    public Command extendUntilFinishedCommand() {
        return extendCommand()
            .withTimeout(ArmConstants.EXTENSION_DURATION_SECONDS)
            .withName("Extend Arm Until Finished");
    }

    /**
     * Provides a command that extends the arm until interrupted.
     * 
     * @return The extension command.
     */
    public Command extendCommand() {
        return startEnd(
            () ->  set(ArmConstants.EXTENDED_ANGLE_RADIANS),
            motor::stopMotor
        ).withName("Extend Arm");
    }

    /**
     * Provides a command that retracts the arm until interrupted.
     * 
     * @return The retraction command.
     */
    public Command retractCommand() {
        return startEnd(
            () -> set(ArmConstants.RETRACTED_ANGLE_RADIANS),
            motor::stopMotor
        ).withName("Retract Arm");
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
            motor::stopMotor
        ).withName("Move Arm");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Absolute Encoder Position Raw", absoluteEncoder.get());
        SmartDashboard.putNumber("Arm/Absolute Encoder Position Adjusted", getPosition());
        SmartDashboard.putNumber("Arm/Motor Voltage", motor.getAppliedOutput() * motor.getBusVoltage());
        SmartDashboard.putNumber("Arm/Motor Current", motor.getOutputCurrent());
    }
}
