package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax leftMotor = new SparkMax(ShooterConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ShooterConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final SparkClosedLoopController leftController = leftMotor.getClosedLoopController();

    public ShooterSubsystem() {
        // TODO: Switch controllers from SparkMax to TalonFX on real robot
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.closedLoop.p(ShooterConstants.PROPORTIONAL_GAIN).i(ShooterConstants.INTEGRAL_GAIN).d(ShooterConstants.DERIVATIVE_GAIN);
        leftConfig.closedLoop.feedForward.kS(ShooterConstants.STATIC_FRICTION_VOLTAGE).kV(ShooterConstants.VOLTAGE_PER_MOTOR_RPM);
        leftConfig.encoder.velocityConversionFactor(1.0 / ShooterConstants.GEAR_RATIO);
        leftConfig.idleMode(IdleMode.kCoast);
        leftConfig.inverted(true); // TODO: Change based on real robot
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kCoast);
        rightConfig.follow(leftMotor, true);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private double calculateSpeed(double distance) {
        // AX^2 + BX^1 + CX^0 or ax^2 + bx + c
        return 
            ShooterConstants.DIST_KX2 * distance * distance +
            ShooterConstants.DIST_KX1 * distance +
            ShooterConstants.DIST_KX0;
    }

    public void run(double distance) {
        double speed = calculateSpeed(distance);
        leftController.setSetpoint(speed, ControlType.kVelocity);
    }

    public void stop() {
        leftMotor.stopMotor();
    }

    public boolean atSpeed(double distance) {
        double targetSpeed = calculateSpeed(distance);
        double currentSpeed = leftMotor.getEncoder().getVelocity();
        return Math.abs(targetSpeed - currentSpeed) < ShooterConstants.RPM_TOLERANCE;
    }
}
