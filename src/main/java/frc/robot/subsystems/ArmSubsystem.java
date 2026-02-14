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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Robot;
import frc.robot.constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(ArmConstants.MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_PWM_PORT);
    private final SparkClosedLoopController controller = motor.getClosedLoopController();

    private SingleJointedArmSim sim;
    private DutyCycleEncoderSim absoluteEncoderSim;
    private SparkRelativeEncoderSim relativeEncoderSim;

    public ArmSubsystem() {
        motorConfig.idleMode(IdleMode.kBrake);

        motorConfig.encoder.positionConversionFactor(1.0 /  ArmConstants.GEAR_RATIO);
        motorConfig.encoder.velocityConversionFactor(1.0 / ArmConstants.GEAR_RATIO);

        motorConfig.closedLoop
            .p(ArmConstants.PID_KP)
            .i(ArmConstants.PID_KI)
            .d(ArmConstants.PID_KD);

        motorConfig.closedLoop.feedForward
            .kS(ArmConstants.FEEDFORWARD_KS)
            .kV(ArmConstants.FEEDFORWARD_KV)
            .kA(ArmConstants.FEEDFORWARD_KA)
            .kCos(ArmConstants.FEEDFORWARD_KG)
            .kCosRatio(1.0);

        motorConfig.closedLoop.allowedClosedLoopError(ArmConstants.SETPOINT_ERROR_TOLERANCE, ClosedLoopSlot.kSlot0);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.getEncoder().setPosition(absoluteEncoder.get());

        if (Robot.isSimulation()) {
            sim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                ArmConstants.GEAR_RATIO,
                ArmConstants.LENGTH_METERS * Math.pow(ArmConstants.LENGTH_METERS, 2) / 3,
                ArmConstants.LENGTH_METERS,
                ArmConstants.MINIMUM_ANGLE_RADIANS,
                ArmConstants.MAXIMUM_ANGLE_RADIANS,
                false,
                ArmConstants.MAXIMUM_ANGLE_RADIANS
            );
            absoluteEncoderSim = new DutyCycleEncoderSim(absoluteEncoder);
            relativeEncoderSim = new SparkRelativeEncoderSim(motor);
        }
    }

    public Command retractCommand() {
        return startEnd(
            () -> controller.setSetpoint(ArmConstants.RETRACTED_SETPOINT, ControlType.kPosition),
            () -> motor.stopMotor()
        ).until(() -> controller.isAtSetpoint());
    }

    public Command extendCommand() {
        return startEnd(
            () -> controller.setSetpoint(ArmConstants.EXTENDED_SETPOINT, ControlType.kPosition),
            () -> motor.stopMotor()
        ).until(() -> controller.isAtSetpoint());
    }

    public Command moveCommand(DoubleSupplier speed) {
        return run(() -> motor.set(speed.getAsDouble()));
    }

    @Override
    public void simulationPeriodic() {
        sim.setInputVoltage(motor.get() * 12.0);
        sim.update(0.02);

        double position = sim.getAngleRads() / (2.0 * Math.PI);
        double velocity = sim.getVelocityRadPerSec() / (2.0 * Math.PI) * 60;

        absoluteEncoderSim.set(position);
        relativeEncoderSim.setPosition(position);
        relativeEncoderSim.setVelocity(velocity);
    }
}
