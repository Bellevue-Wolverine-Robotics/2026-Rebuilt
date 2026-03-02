package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Robot;
import frc.robot.constants.ArmConstants;

/** Represents the arm mechanism, which moves the linkage intake */
public class ArmSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(ArmConstants.MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_PORT);
    private final RelativeEncoder relativeEncoder = motor.getEncoder();
    private final SparkClosedLoopController controller = motor.getClosedLoopController();

    private SingleJointedArmSim sim;
    private DutyCycleEncoderSim absoluteEncoderSim;
    private SparkRelativeEncoderSim relativeEncoderSim;

    /** Constructs a new ArmSubsystem. */
    public ArmSubsystem() {
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.inverted(ArmConstants.MOTOR_INVERTED);

        motorConfig.encoder.positionConversionFactor((2.0 * Math.PI) /  ArmConstants.GEAR_RATIO);
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

        if (Robot.isSimulation()) {
            sim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                ArmConstants.GEAR_RATIO,
                ArmConstants.MASS_KILOGRAMS * Math.pow(ArmConstants.LENGTH_METERS, 2) / 3,
                ArmConstants.LENGTH_METERS,
                ArmConstants.RETRACTED_ANGLE_RADIANS,
                ArmConstants.EXTENDED_ANGLE_RADIANS,
                false,
                ArmConstants.RETRACTED_ANGLE_RADIANS
            );

            absoluteEncoderSim = new DutyCycleEncoderSim(absoluteEncoder);
            relativeEncoderSim = new SparkRelativeEncoderSim(motor);
        }
    }

    private void synchronize() {
        double position = (absoluteEncoder.get() + ArmConstants.ABSOLUTE_ENCODER_OFFSET_DUTY_CYCLE + 1) % 1;
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
            () -> motor.set(speed.getAsDouble()),
            () -> motor.stopMotor()
        );
    }

    @Override
    public void simulationPeriodic() {
        sim.setInputVoltage(motor.get() * 12.0);
        sim.update(0.02);

        double position = sim.getAngleRads();
        double velocity = sim.getVelocityRadPerSec();

        absoluteEncoderSim.set(position / (2.0 * Math.PI));
        relativeEncoderSim.setPosition(position);
        relativeEncoderSim.setVelocity(velocity);
    }
}
