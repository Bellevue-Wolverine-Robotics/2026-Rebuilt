package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
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
        if (distance <= ShooterConstants.SETPOINTS_METERS_TO_RPM.firstKey()) {
            return ShooterConstants.SETPOINTS_METERS_TO_RPM.firstEntry().getValue();
        }

        if (distance >= ShooterConstants.SETPOINTS_METERS_TO_RPM.lastKey()) {
            return ShooterConstants.SETPOINTS_METERS_TO_RPM.lastEntry().getValue();
        }

        var lower = ShooterConstants.SETPOINTS_METERS_TO_RPM.floorEntry(distance);
        var upper = ShooterConstants.SETPOINTS_METERS_TO_RPM.ceilingEntry(distance);

        if (lower.getKey().equals(upper.getKey())) {
            return lower.getValue();
        }

        double slope = (upper.getValue() - lower.getValue()) / (upper.getKey() - lower.getKey());
        double difference = distance - lower.getKey();
        return slope * difference + lower.getValue();
    }

    public boolean atSpeed(double distance) {
        double targetSpeed = calculateSpeed(distance);
        double currentSpeed = leftMotor.getEncoder().getVelocity();
        return Math.abs(targetSpeed - currentSpeed) < ShooterConstants.RPM_TOLERANCE;
    }

    public Command shootDistanceCommand(double distance, FeederSubsystem feeder) {
        return runEnd(() -> run(distance), () -> stop())
            .alongWith((feeder.feedCommand(() -> atSpeed(distance))));
    }

    public void run(double distance) {
        double speed = calculateSpeed(distance);
        leftController.setSetpoint(speed, ControlType.kVelocity);
    }

    public void stop() {
        leftMotor.stopMotor();
    }
}
