package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax leftIntakeMotor = new SparkMax(IntakeConstants.LEFT_INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMax rightIntakeMotor = new SparkMax(IntakeConstants.RIGHT_INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMaxConfig rightIntakeMotorConfig = new SparkMaxConfig();

    private final SparkMax armMotor = new SparkMax(IntakeConstants.ARM_MOTOR_CAN_ID, MotorType.kBrushless);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(IntakeConstants.ARM_ENCODER_PWM_PORT);
    private final PIDController armPidController = new PIDController(IntakeConstants.ARM_PID_KP, IntakeConstants.ARM_PID_KI, IntakeConstants.ARM_PID_KD);
    private boolean extended = false;

    private SingleJointedArmSim armSim;
    private DutyCycleEncoderSim armEncoderSim;

    public IntakeSubsystem() {
        rightIntakeMotorConfig.follow(leftIntakeMotor, true).idleMode(IdleMode.kBrake);
        rightIntakeMotor.configure(rightIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        setDefaultCommand(idle());

        if (Robot.isSimulation()) {
            armSim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                IntakeConstants.ARM_GEAR_RATIO,
                IntakeConstants.ARM_LENGTH_METERS * Math.pow(IntakeConstants.ARM_LENGTH_METERS, 2) / 3,
                IntakeConstants.ARM_LENGTH_METERS,
                IntakeConstants.MINIMUM_ANGLE_RADIANS,
                IntakeConstants.MAXIMUM_ANGLE_RADIANS,
                false,
                IntakeConstants.ARM_RETRACTED_SETPOINT * (2.0 * Math.PI)
            );
            armEncoderSim = new DutyCycleEncoderSim(armEncoder);
        }
    }

    @Override
    public void periodic() {
        double setpoint = extended ? IntakeConstants.ARM_EXTENDED_SETPOINT : IntakeConstants.ARM_RETRACTED_SETPOINT;
        double position = armEncoder.get();
        double speed = armPidController.calculate(position, setpoint);
        armMotor.set(speed);
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInput(armMotor.get() * 12.0);
        armSim.update(0.02);
        armEncoderSim.set(armSim.getAngleRads() / (2.0 * Math.PI));
    }

    public Command retract() {
        return Commands.runOnce(() -> extended = false);
    }

    public Command extend() {
        return Commands.runOnce(() -> extended = true);
    }

    public Command intake() {
        return Commands.run(() -> leftIntakeMotor.set(IntakeConstants.INTAKE_SPEED), this);
    }

    public Command idle() {
        return Commands.run(() -> leftIntakeMotor.stopMotor(), this);
    }
}
