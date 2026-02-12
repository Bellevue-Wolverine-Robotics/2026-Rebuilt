package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Robot;
import frc.robot.constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(ArmConstants.MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.ENCODER_PWM_PORT);
    private final PIDController controller = new PIDController(ArmConstants.PID_KP, ArmConstants.PID_KI, ArmConstants.PID_KD);

    private SingleJointedArmSim sim;
    private DutyCycleEncoderSim encoderSim;

    public ArmSubsystem() {
        motorConfig.idleMode(IdleMode.kBrake);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (Robot.isSimulation()) {
            sim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                ArmConstants.GEAR_RATIO,
                ArmConstants.LENGTH_METERS * Math.pow(ArmConstants.LENGTH_METERS, 2) / 3,
                ArmConstants.LENGTH_METERS,
                ArmConstants.MINIMUM_ANGLE_RADIANS,
                ArmConstants.MAXIMUM_ANGLE_RADIANS,
                false,
                ArmConstants.RETRACTED_SETPOINT * (2.0 * Math.PI)
            );
            encoderSim = new DutyCycleEncoderSim(encoder);
        }
    }

    private void movePosition(double setpoint) {
        double position = encoder.get();
        double speed = controller.calculate(position, setpoint);
        motor.set(speed);
    }

    public Command retract() {
        return startEnd(
            () -> movePosition(ArmConstants.RETRACTED_SETPOINT),
            () -> motor.stopMotor()
        );
    }

    public Command extend() {
        return startEnd(
            () -> movePosition(ArmConstants.EXTENDED_SETPOINT),
            () -> motor.stopMotor()
        );
    }

    @Override
    public void simulationPeriodic() {
        sim.setInput(motor.get() * 12.0);
        sim.update(0.02);
        encoderSim.set(sim.getAngleRads() / (2.0 * Math.PI));
    }
}
