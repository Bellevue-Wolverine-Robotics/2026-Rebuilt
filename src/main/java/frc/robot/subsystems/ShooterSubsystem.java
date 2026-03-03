package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax leftMotor = new SparkMax(ShooterConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ShooterConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final SparkClosedLoopController leftController = leftMotor.getClosedLoopController();
    private double targetSpeed = 0.0;

    public ShooterSubsystem() {
        // TODO: Switch controllers from SparkMax to TalonFX on real robot
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.closedLoop.p(ShooterConstants.PROPORTIONAL_GAIN).i(ShooterConstants.INTEGRAL_GAIN).d(ShooterConstants.DERIVATIVE_GAIN);
        leftConfig.closedLoop.feedForward.kS(ShooterConstants.STATIC_FRICTION_OVERCOME_VOLTAGE).kV(ShooterConstants.VOLTAGE_PER_SHAFT_RPM);
        leftConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.GEAR_RATIO);
        leftConfig.idleMode(IdleMode.kCoast);
        leftConfig.inverted(ShooterConstants.LEFT_MOTOR_INVERTED); // TODO: Change based on real robot
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kCoast);
        rightConfig.follow(leftMotor, true);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private double calculateSpeed(double distance) {
        return ShooterConstants.DISTANCE_METERS_TO_RPM.get(distance);
    }

    public boolean atSpeed() {
        double currentSpeed = leftMotor.getEncoder().getVelocity();
        return Math.abs(targetSpeed - currentSpeed) < ShooterConstants.RPM_TOLERANCE;
    }

    public void run(double distance) {
        targetSpeed = calculateSpeed(distance);
        leftController.setSetpoint(targetSpeed, ControlType.kVelocity);
    }

    public void stop() {
        targetSpeed = 0.0;
        leftMotor.stopMotor();
    }

    public void periodic() {
        SmartDashboard.putNumber("Shooter/Target Flywheel Speed", targetSpeed);
        SmartDashboard.putNumber("Shooter/Current Flywheel Speed", leftMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Applied Voltage", leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Shooter/Applied Current", leftMotor.getOutputCurrent());
    }
}
